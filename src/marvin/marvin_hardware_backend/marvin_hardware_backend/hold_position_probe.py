"""First-write acceptance test: hold the measured Marvin joint positions."""

from __future__ import annotations

import argparse
import ipaddress
import json
import time

from ament_index_python.packages import get_package_share_directory
import numpy as np
import yaml

from .hardware_safety import HardwareSafetyController, HardwareSafetySettings
from .marvin_hardware import MarvinHardwareSession
from .sdk_loader import load_marvin_sdk


class HoldAcceptanceError(RuntimeError):
    """Raised when the measured-pose hold fails an acceptance condition."""


def _default_config_path() -> str:
    share = get_package_share_directory("marvin_hardware_backend")
    return f"{share}/config/real.yaml"


def _load_parameters(path: str) -> dict:
    with open(path, encoding="utf-8") as config_file:
        document = yaml.safe_load(config_file)
    try:
        parameters = document["marvin_hardware_bridge"]["ros__parameters"]
    except (KeyError, TypeError) as exc:
        raise ValueError(
            "配置文件缺少 marvin_hardware_bridge.ros__parameters: "
            f"{path}"
        ) from exc
    if not isinstance(parameters, dict):
        raise ValueError(f"无效的 Marvin 参数对象: {path}")
    return parameters


def _vector(parameters: dict, name: str) -> np.ndarray:
    try:
        values = np.asarray(parameters[name], dtype=np.float64)
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"配置参数 {name} 无效") from exc
    if values.shape != (7,) or not np.isfinite(values).all():
        raise ValueError(f"配置参数 {name} 必须包含 7 个有限数值")
    return values


def _observe_safety(
    safety: HardwareSafetyController,
    feedback,
    target_left: np.ndarray,
    target_right: np.ndarray,
    now: float,
) -> None:
    safety.observe_feedback(
        left_joints_deg=feedback.left_joints_deg,
        right_joints_deg=feedback.right_joints_deg,
        arm_states=feedback.arm_states,
        command_states=feedback.command_states,
        error_codes=feedback.error_codes,
        servo_error_reports=feedback.servo_error_reports,
        frame_serials=feedback.frame_serials,
        received_at=now,
    )
    safety.observe_teleop_state("teleop", received_at=now)
    safety.observe_command(
        "left",
        target_left,
        frame_id="left_base_marvin_degrees",
        received_at=now,
    )
    safety.observe_command(
        "right",
        target_right,
        frame_id="right_base_marvin_degrees",
        received_at=now,
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Read the current Marvin joint positions, enter state 1 at that "
            "same pose, and resend only the safety-approved hold target."
        )
    )
    parser.add_argument(
        "--confirm-hold",
        action="store_true",
        help="Explicitly permit state-1 enable and measured-pose commands.",
    )
    parser.add_argument(
        "--config",
        default=_default_config_path(),
        help="Path to the installed real.yaml.",
    )
    parser.add_argument("--robot-ip", help="Override robot_ip from real.yaml.")
    parser.add_argument(
        "--duration",
        type=float,
        default=5.0,
        help="Hold duration in seconds (2..30, default: 5).",
    )
    parser.add_argument(
        "--maximum-hold-error-deg",
        type=float,
        default=0.5,
        help="Maximum measured drift from the initial pose (default: 0.5).",
    )
    return parser


def main(args=None) -> int:
    options = _parser().parse_args(args)
    if not options.confirm_hold:
        print(
            json.dumps(
                {
                    "status": "HOLD_POSITION_REFUSED",
                    "success": False,
                    "reason": "missing_--confirm-hold",
                },
                ensure_ascii=False,
                indent=2,
            ),
            flush=True,
        )
        return 2
    if not 2.0 <= options.duration <= 30.0:
        raise SystemExit("--duration 必须在 2..30 秒")
    if not 0.05 <= options.maximum_hold_error_deg <= 2.0:
        raise SystemExit("--maximum-hold-error-deg 必须在 0.05..2.0 度")

    parameters = _load_parameters(options.config)
    robot_ip = str(
        ipaddress.ip_address(options.robot_ip or parameters["robot_ip"])
    )
    rate_hz = float(parameters["rate"])
    if rate_hz <= 0.0:
        raise ValueError("rate 必须为正数")
    lower = _vector(parameters, "lower_limits_deg")
    upper = _vector(parameters, "upper_limits_deg")
    velocity_ratio = int(parameters["velocity_ratio"])
    acceleration_ratio = int(parameters["acceleration_ratio"])
    hard_padding = float(parameters["feedback_hard_limit_padding_deg"])

    dcss_type, robot_type, _ = load_marvin_sdk()
    session = MarvinHardwareSession(
        robot=robot_type(),
        dcss_factory=dcss_type,
    )
    stage = "created"
    position_mode_entered = False
    soft_stop_requested = False
    shutdown_verified = False
    hold_batches_sent = 0
    maximum_error = 0.0
    initial_enable_error = None
    frame_advanced = [False, False]
    previous_serials = None
    last_feedback = None
    failure_reason = None

    print(
        json.dumps(
            {
                "status": "HOLD_POSITION_STARTING",
                "robot_ip": robot_ip,
                "duration_s": options.duration,
                "rate_hz": rate_hz,
                "maximum_hold_error_deg": options.maximum_hold_error_deg,
                "clear_errors": False,
                "target_source": "measured_feedback_before_enable",
            },
            ensure_ascii=False,
            indent=2,
        ),
        flush=True,
    )

    try:
        stage = "connect_and_prepare"
        reference, prepared = session.connect_and_prepare(
            robot_ip,
            velocity_ratio=velocity_ratio,
            acceleration_ratio=acceleration_ratio,
            lower_limits_deg=lower,
            upper_limits_deg=upper,
            hard_limit_padding_deg=hard_padding,
            clear_errors=False,
            include_position_reference=True,
        )
        position_mode_entered = True
        target_left = reference.left_joints_deg.copy()
        target_right = reference.right_joints_deg.copy()
        reference_all = np.concatenate([target_left, target_right])
        prepared_all = np.concatenate(
            [prepared.left_joints_deg, prepared.right_joints_deg]
        )
        initial_enable_error = float(
            np.max(np.abs(prepared_all - reference_all), initial=0.0)
        )
        maximum_error = initial_enable_error
        if initial_enable_error > options.maximum_hold_error_deg:
            raise HoldAcceptanceError(
                "position changed too much while entering state 1: "
                f"{initial_enable_error:.4f} deg"
            )

        settings = HardwareSafetySettings(
            command_timeout_s=float(parameters["command_timeout_s"]),
            state_timeout_s=float(parameters["state_timeout_s"]),
            feedback_timeout_s=float(parameters["feedback_timeout_s"]),
            maximum_pair_skew_s=float(parameters["maximum_pair_skew_s"]),
            maximum_output_step_deg=float(
                parameters["maximum_output_step_deg"]
            ),
            maximum_tracking_error_deg=float(
                parameters["maximum_tracking_error_deg"]
            ),
            return_minimum_duration_s=float(
                parameters["return_minimum_duration_s"]
            ),
            return_max_speed_deg_s=float(
                parameters["return_max_speed_deg_s"]
            ),
            home_tolerance_deg=float(parameters["home_tolerance_deg"]),
            feedback_hard_limit_padding_deg=hard_padding,
        )
        # For this acceptance test the measured pose is both the temporary
        # safety home and the only command. This prevents a first-frame move
        # toward the configured application Home.
        safety = HardwareSafetyController(
            left_home_deg=target_left,
            right_home_deg=target_right,
            lower_limits_deg=lower,
            upper_limits_deg=upper,
            settings=settings,
        )
        stage = "holding_measured_pose"
        previous_serials = prepared.frame_serials
        deadline = time.monotonic() + options.duration
        period = 1.0 / rate_hz
        next_print = time.monotonic()
        while time.monotonic() < deadline:
            loop_started = time.monotonic()
            feedback = session.read_feedback()
            last_feedback = feedback
            now = time.monotonic()
            for index, serial in enumerate(feedback.frame_serials):
                if serial != previous_serials[index]:
                    frame_advanced[index] = True
            previous_serials = feedback.frame_serials
            _observe_safety(
                safety, feedback, target_left, target_right, now
            )
            decision = safety.decide(now=now)
            if decision.action == "soft_stop":
                soft_stop_requested = (
                    session.soft_stop_once() or soft_stop_requested
                )
                raise HoldAcceptanceError(
                    f"safety bridge requested soft stop: {decision.reason}"
                )
            if decision.action != "send":
                raise HoldAcceptanceError(
                    "unexpected safety decision: "
                    f"{decision.action}:{decision.reason}"
                )
            session.send_joint_targets(
                decision.left_joints_deg,
                decision.right_joints_deg,
            )
            hold_batches_sent += 1
            measured_all = np.concatenate(
                [feedback.left_joints_deg, feedback.right_joints_deg]
            )
            hold_error = float(
                np.max(np.abs(measured_all - reference_all), initial=0.0)
            )
            maximum_error = max(maximum_error, hold_error)
            if hold_error > options.maximum_hold_error_deg:
                soft_stop_requested = (
                    session.soft_stop_once() or soft_stop_requested
                )
                raise HoldAcceptanceError(
                    "measured pose drift exceeded acceptance limit: "
                    f"{hold_error:.4f} deg"
                )
            if now >= next_print:
                print(
                    json.dumps(
                        {
                            "status": "HOLD_POSITION_RUNNING",
                            "batches_sent": hold_batches_sent,
                            "maximum_error_deg": round(maximum_error, 6),
                            "frame_serials": feedback.frame_serials,
                            "arm_states": feedback.arm_states,
                            "command_states": feedback.command_states,
                            "error_codes": feedback.error_codes,
                            "safety_decision": decision.reason,
                        },
                        ensure_ascii=False,
                    ),
                    flush=True,
                )
                next_print = now + 1.0
            delay = period - (time.monotonic() - loop_started)
            if delay > 0.0:
                time.sleep(delay)

        if not all(frame_advanced):
            raise HoldAcceptanceError(
                f"feedback frames did not advance for both arms: {frame_advanced}"
            )
        stage = "verified_shutdown"
        shutdown_feedback = session.shutdown_verified()
        last_feedback = shutdown_feedback
        shutdown_verified = (
            shutdown_feedback.arm_states == (0, 0)
            and shutdown_feedback.error_codes == (0, 0)
        )
        if not shutdown_verified:
            raise HoldAcceptanceError(
                "state 0 shutdown feedback was not verified"
            )
        session = None
    except KeyboardInterrupt:
        failure_reason = "interrupted_by_user"
    except BaseException as exc:
        failure_reason = str(exc)
    finally:
        if session is not None:
            if position_mode_entered and not soft_stop_requested:
                try:
                    soft_stop_requested = (
                        session.soft_stop_once() or soft_stop_requested
                    )
                except BaseException:
                    pass
            try:
                session.shutdown()
            except BaseException:
                pass

    success = failure_reason is None and shutdown_verified
    final_status = (
        "HOLD_POSITION_SUCCESS" if success else "HOLD_POSITION_FAILED"
    )
    result = {
        "status": final_status,
        "success": success,
        "failure_stage": None if success else stage,
        "reason": failure_reason,
        "position_mode_entered": position_mode_entered,
        "initial_enable_error_deg": initial_enable_error,
        "maximum_hold_error_deg": maximum_error,
        "frame_serial_advanced": {
            "A": frame_advanced[0],
            "B": frame_advanced[1],
        },
        "hold_command_batches_sent": hold_batches_sent,
        "safety_bridge_used": True,
        "soft_stop_requested": soft_stop_requested,
        "state_0_shutdown_verified": shutdown_verified,
        "final_feedback": (
            None
            if last_feedback is None
            else {
                "arm_states": last_feedback.arm_states,
                "command_states": last_feedback.command_states,
                "error_codes": last_feedback.error_codes,
                "frame_serials": last_feedback.frame_serials,
            }
        ),
    }
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
