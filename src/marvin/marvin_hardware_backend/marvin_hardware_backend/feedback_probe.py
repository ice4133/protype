"""Read-only Marvin controller connection and feedback diagnostic."""

from __future__ import annotations

import argparse
import ipaddress
import json
import math
import time

from ament_index_python.packages import get_package_share_directory
import yaml

from .sdk_loader import load_marvin_sdk


def _default_config_path() -> str:
    share = get_package_share_directory("marvin_hardware_backend")
    return f"{share}/config/real.yaml"


def _robot_ip_from_config(path: str) -> str:
    with open(path, encoding="utf-8") as config_file:
        document = yaml.safe_load(config_file)
    try:
        value = document["marvin_hardware_bridge"]["ros__parameters"][
            "robot_ip"
        ]
    except (KeyError, TypeError) as exc:
        raise ValueError(
            f"配置文件缺少 marvin_hardware_bridge.ros__parameters.robot_ip: {path}"
        ) from exc
    return str(ipaddress.ip_address(str(value)))


def _finite_joints(values, label: str) -> list[float]:
    joints = [float(value) for value in values]
    if len(joints) != 7 or not all(math.isfinite(value) for value in joints):
        raise ValueError(f"{label} 必须包含 7 个有限关节角")
    return joints


def _feedback_sample(payload) -> dict:
    try:
        states = payload["states"]
        outputs = payload["outputs"]
        inputs = payload.get("inputs", ({}, {}))
        if len(states) != 2 or len(outputs) != 2 or len(inputs) != 2:
            raise ValueError("反馈必须包含 A/B 两臂数据")
        arms = []
        for index, name in enumerate(("A", "B")):
            arms.append(
                {
                    "arm": name,
                    "joints_deg": _finite_joints(
                        outputs[index]["fb_joint_pos"],
                        f"Arm {name} feedback",
                    ),
                    "frame_serial": int(outputs[index]["frame_serial"]),
                    "current_state": int(states[index]["cur_state"]),
                    "command_state": int(states[index]["cmd_state"]),
                    "error_code": int(states[index]["err_code"]),
                    "velocity_ratio": (
                        None
                        if "joint_vel_ratio" not in inputs[index]
                        else int(inputs[index]["joint_vel_ratio"])
                    ),
                    "acceleration_ratio": (
                        None
                        if "joint_acc_ratio" not in inputs[index]
                        else int(inputs[index]["joint_acc_ratio"])
                    ),
                }
            )
    except (IndexError, KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"SDK 返回了无效反馈: {exc}") from exc
    return {"arms": arms}


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Connect to Marvin and read feedback only. No state, limit, "
            "homing, or joint command API is called."
        )
    )
    parser.add_argument(
        "--confirm-readonly",
        action="store_true",
        help="Explicitly permit a read-only connection to the controller.",
    )
    parser.add_argument(
        "--config",
        default=_default_config_path(),
        help="real.yaml path used to obtain robot_ip.",
    )
    parser.add_argument(
        "--robot-ip",
        help="Override robot_ip from real.yaml.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="Read duration in seconds (2..300, default: 10).",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=5.0,
        help="Feedback print rate in Hz (0.2..50, default: 5).",
    )
    return parser


def main(args=None) -> int:
    options = _parser().parse_args(args)
    if not options.confirm_readonly:
        print(
            "拒绝连接：必须显式提供 --confirm-readonly。",
            flush=True,
        )
        return 2
    if not 2.0 <= options.duration <= 300.0:
        raise SystemExit("--duration 必须在 2..300 秒")
    if not 0.2 <= options.rate <= 50.0:
        raise SystemExit("--rate 必须在 0.2..50 Hz")
    if options.duration * options.rate < 2.0:
        raise SystemExit("--duration × --rate 至少应产生 2 个反馈样本")
    robot_ip = (
        str(ipaddress.ip_address(options.robot_ip))
        if options.robot_ip
        else _robot_ip_from_config(options.config)
    )

    dcss_type, robot_type, _ = load_marvin_sdk()
    robot = robot_type()
    dcss = dcss_type()
    connected = False
    samples = 0
    previous_serials: tuple[int, int] | None = None
    advanced = [False, False]
    interrupted = False

    print(
        json.dumps(
            {
                "mode": "readonly_feedback",
                "robot_ip": robot_ip,
                "duration_s": options.duration,
                "rate_hz": options.rate,
                "control_commands_enabled": False,
                "allowed_sdk_calls": [
                    "connect",
                    "subscribe",
                    "release_robot",
                ],
            },
            ensure_ascii=False,
            indent=2,
        ),
        flush=True,
    )
    try:
        if not bool(robot.connect(robot_ip)):
            print("错误：Marvin SDK connect() 返回失败。", flush=True)
            return 1
        connected = True
        print("只读连接成功，开始读取反馈。", flush=True)
        start = time.monotonic()
        deadline = start + options.duration
        period = 1.0 / options.rate
        next_sample = start
        while time.monotonic() < deadline:
            payload = robot.subscribe(dcss)
            sample = _feedback_sample(payload)
            serials = tuple(
                arm["frame_serial"] for arm in sample["arms"]
            )
            if previous_serials is not None:
                for index in range(2):
                    if serials[index] != previous_serials[index]:
                        advanced[index] = True
            previous_serials = serials
            samples += 1
            sample.update(
                {
                    "sample": samples,
                    "elapsed_s": round(time.monotonic() - start, 3),
                }
            )
            print(json.dumps(sample, ensure_ascii=False), flush=True)
            next_sample += period
            delay = min(
                next_sample - time.monotonic(),
                deadline - time.monotonic(),
            )
            if delay > 0.0:
                time.sleep(delay)
    except KeyboardInterrupt:
        interrupted = True
        print("收到 Ctrl+C，停止读取。", flush=True)
    except BaseException as exc:
        print(f"错误：只读反馈失败：{exc}", flush=True)
        return 1
    finally:
        if connected:
            try:
                released = bool(robot.release_robot())
                print(
                    f"SDK 连接已释放：{'成功' if released else '返回失败'}。",
                    flush=True,
                )
            except BaseException as exc:
                print(f"警告：release_robot() 异常：{exc}", flush=True)

    result = {
        "result": "ok" if all(advanced) else "feedback_not_advancing",
        "samples": samples,
        "frame_serial_advanced": {"A": advanced[0], "B": advanced[1]},
        "interrupted": interrupted,
        "control_commands_sent": 0,
    }
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    return 0 if all(advanced) else 1


if __name__ == "__main__":
    raise SystemExit(main())
