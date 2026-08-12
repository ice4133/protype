"""Single-joint, small-amplitude Marvin hardware acceptance test."""

from __future__ import annotations

import argparse
import ipaddress
import json
import math
import time

import numpy as np

from .hardware_safety import HardwareSafetyController, HardwareSafetySettings
from .hold_position_probe import (
    _default_config_path,
    _load_parameters,
    _observe_safety,
    _vector,
)
from .home_trajectory import HomeTrajectory
from .marvin_hardware import MarvinHardwareSession
from .sdk_loader import load_marvin_sdk


MAXIMUM_DELTA_DEG = 2.0
MAXIMUM_TEST_SPEED_DEG_S = 1.0


class SmallMotionAcceptanceError(RuntimeError):
    """Raised when the small-motion acceptance conditions are not met."""


def build_motion_target(
    *,
    reference_left,
    reference_right,
    arm: str,
    joint: int,
    delta_deg: float,
    lower_limits_deg,
    upper_limits_deg,
    limit_margin_deg: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Return one-axis target vectors after strict limit validation."""
    left = np.asarray(reference_left, dtype=np.float64)
    right = np.asarray(reference_right, dtype=np.float64)
    lower = np.asarray(lower_limits_deg, dtype=np.float64)
    upper = np.asarray(upper_limits_deg, dtype=np.float64)
    if any(vector.shape != (7,) for vector in (left, right, lower, upper)):
        raise ValueError("joint vectors and limits must contain seven values")
    if not all(
        np.isfinite(vector).all() for vector in (left, right, lower, upper)
    ):
        raise ValueError("joint vectors and limits must be finite")
    if np.any(lower >= upper):
        raise ValueError("lower joint limits must be below upper limits")
    if arm not in {"A", "B"}:
        raise ValueError("arm must be A or B")
    if isinstance(joint, bool) or int(joint) != joint or not 1 <= joint <= 7:
        raise ValueError("joint must be an integer from 1 to 7")
    if not math.isfinite(delta_deg) or delta_deg == 0.0:
        raise ValueError("delta_deg must be finite and non-zero")
    if not 0.0 <= limit_margin_deg <= 10.0:
        raise ValueError("limit_margin_deg must be between 0 and 10")
    reference = np.concatenate([left, right])
    tiled_lower = np.tile(lower, 2)
    tiled_upper = np.tile(upper, 2)
    if np.any(reference < tiled_lower) or np.any(reference > tiled_upper):
        raise SmallMotionAcceptanceError(
            "measured reference is outside configured joint limits"
        )

    target_left = left.copy()
    target_right = right.copy()
    target = target_left if arm == "A" else target_right
    index = joint - 1
    requested = float(target[index] + delta_deg)
    safe_lower = float(lower[index] + limit_margin_deg)
    safe_upper = float(upper[index] - limit_margin_deg)
    if safe_lower >= safe_upper:
        raise ValueError("joint limit margin leaves no usable range")
    if not safe_lower <= requested <= safe_upper:
        raise SmallMotionAcceptanceError(
            f"requested endpoint {requested:.4f} deg for arm {arm} joint "
            f"{joint} is outside the configured limit margin "
            f"[{safe_lower:.4f}, {safe_upper:.4f}] deg"
        )
    target[index] = requested
    return target_left, target_right


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Move one Marvin joint by a small smooth offset, verify the "
            "endpoint, return to the measured reference, and verify state 0."
        )
    )
    parser.add_argument(
        "--confirm-motion",
        action="store_true",
        help="Explicitly permit this real-hardware position command test.",
    )
    parser.add_argument(
        "--arm",
        choices=("A", "B"),
        required=True,
        help="Arm to move. The other arm holds its measured reference.",
    )
    parser.add_argument(
        "--joint",
        type=int,
        required=True,
        help="One-based Marvin joint index (1..7).",
    )
    parser.add_argument(
        "--delta-deg",
        type=float,
        required=True,
        help="Signed offset in degrees; magnitude must be 0.1..2.0.",
    )
    parser.add_argument(
        "--move-duration",
        type=float,
        default=2.0,
        help="Minimum duration of each outbound/return leg (1..10 seconds).",
    )
    parser.add_argument(
        "--hold-duration",
        type=float,
        default=1.0,
        help="Endpoint verification hold duration (0.5..5 seconds).",
    )
    parser.add_argument(
        "--position-tolerance-deg",
        type=float,
        default=0.2,
        help="Maximum endpoint and return error (0.05..0.5 degrees).",
    )
    parser.add_argument(
        "--maximum-tracking-error-deg",
        type=float,
        default=0.5,
        help="Maximum dynamic command/feedback error (default: 0.5 degrees).",
    )
    parser.add_argument(
        "--limit-margin-deg",
        type=float,
        default=2.0,
        help="Required endpoint distance from configured limits (0..10).",
    )
    parser.add_argument(
        "--config",
        default=_default_config_path(),
        help="Path to the installed real.yaml.",
    )
    parser.add_argument("--robot-ip", help="Override robot_ip from real.yaml.")
    return parser


def _feedback_vector(feedback) -> np.ndarray:
    return np.concatenate(
        [feedback.left_joints_deg, feedback.right_joints_deg]
    )


def main(args=None) -> int:
    options = _parser().parse_args(args)
    if not options.confirm_motion:
        print(
            json.dumps(
                {
                    "status": "SMALL_MOTION_REFUSED",
                    "success": False,
                    "reason": "missing_--confirm-motion",
                },
                ensure_ascii=False,
                indent=2,
            ),
            flush=True,
        )
        return 2
    if not 1 <= options.joint <= 7:
        raise SystemExit("--joint 必须在 1..7")
    if not math.isfinite(options.delta_deg) or not (
        0.1 <= abs(options.delta_deg) <= MAXIMUM_DELTA_DEG
    ):
        raise SystemExit("--delta-deg 的绝对值必须在 0.1..2.0 度")
    if not 1.0 <= options.move_duration <= 10.0:
        raise SystemExit("--move-duration 必须在 1..10 秒")
    if not 0.5 <= options.hold_duration <= 5.0:
        raise SystemExit("--hold-duration 必须在 0.5..5 秒")
    if not 0.05 <= options.position_tolerance_deg <= 0.5:
        raise SystemExit("--position-tolerance-deg 必须在 0.05..0.5 度")
    if not 0.0 <= options.limit_margin_deg <= 10.0:
        raise SystemExit("--limit-margin-deg 必须在 0..10 度")

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
    maximum_output_step = float(parameters["maximum_output_step_deg"])
    if not (
        maximum_output_step < options.maximum_tracking_error_deg <= 2.0
    ):
        raise SystemExit(
            "--maximum-tracking-error-deg 必须大于 "
            f"maximum_output_step_deg={maximum_output_step} 且不超过 2 度"
        )
    if options.position_tolerance_deg >= options.maximum_tracking_error_deg:
        raise SystemExit(
            "--position-tolerance-deg 必须小于 "
            "--maximum-tracking-error-deg"
        )

    print(
        json.dumps(
            {
                "status": "SMALL_MOTION_STARTING",
                "robot_ip": robot_ip,
                "arm": options.arm,
                "joint": options.joint,
                "delta_deg": options.delta_deg,
                "minimum_move_duration_s": options.move_duration,
                "hold_duration_s": options.hold_duration,
                "maximum_test_speed_deg_s": MAXIMUM_TEST_SPEED_DEG_S,
                "maximum_tracking_error_deg": (
                    options.maximum_tracking_error_deg
                ),
                "position_tolerance_deg": options.position_tolerance_deg,
                "clear_errors": False,
                "target_source": "measured_feedback_before_enable",
            },
            ensure_ascii=False,
            indent=2,
        ),
        flush=True,
    )

    dcss_type, robot_type, _ = load_marvin_sdk()
    session = MarvinHardwareSession(
        robot=robot_type(),
        dcss_factory=dcss_type,
    )
    stage = "created"
    position_mode_entered = False
    outbound_target_reached = False
    returned_to_reference = False
    shutdown_verified = False
    soft_stop_requested = False
    command_batches_sent = 0
    maximum_tracking_error = 0.0
    maximum_tracking_error_joint = None
    initial_enable_error = None
    frame_advanced = [False, False]
    previous_serials = None
    last_feedback = None
    reference_left = None
    reference_right = None
    target_left = None
    target_right = None
    outbound_feedback = None
    outbound_error = None
    return_error = None
    failure_reason = None

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
        reference_left = reference.left_joints_deg.copy()
        reference_right = reference.right_joints_deg.copy()
        reference_all = np.concatenate([reference_left, reference_right])
        prepared_all = _feedback_vector(prepared)
        initial_enable_error = float(
            np.max(np.abs(prepared_all - reference_all), initial=0.0)
        )
        if initial_enable_error > options.maximum_tracking_error_deg:
            raise SmallMotionAcceptanceError(
                "position changed too much while entering state 1: "
                f"{initial_enable_error:.4f} deg"
            )
        target_left, target_right = build_motion_target(
            reference_left=reference_left,
            reference_right=reference_right,
            arm=options.arm,
            joint=options.joint,
            delta_deg=options.delta_deg,
            lower_limits_deg=lower,
            upper_limits_deg=upper,
            limit_margin_deg=options.limit_margin_deg,
        )
        target_all = np.concatenate([target_left, target_right])

        settings = HardwareSafetySettings(
            command_timeout_s=float(parameters["command_timeout_s"]),
            state_timeout_s=float(parameters["state_timeout_s"]),
            feedback_timeout_s=float(parameters["feedback_timeout_s"]),
            maximum_pair_skew_s=float(parameters["maximum_pair_skew_s"]),
            maximum_output_step_deg=maximum_output_step,
            maximum_tracking_error_deg=(
                options.maximum_tracking_error_deg
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
        safety = HardwareSafetyController(
            left_home_deg=reference_left,
            right_home_deg=reference_right,
            lower_limits_deg=lower,
            upper_limits_deg=upper,
            settings=settings,
        )
        previous_serials = list(prepared.frame_serials)
        period = 1.0 / rate_hz

        def run_leg(
            *,
            label: str,
            start_all: np.ndarray,
            goal_all: np.ndarray,
        ):
            nonlocal command_batches_sent
            nonlocal last_feedback
            nonlocal maximum_tracking_error
            nonlocal maximum_tracking_error_joint
            trajectory = HomeTrajectory(
                start_joints=start_all,
                home_joints=goal_all,
                start_time=time.monotonic(),
                minimum_duration=options.move_duration,
                max_speed_deg_s=MAXIMUM_TEST_SPEED_DEG_S,
            )
            endpoint_started = None
            next_print = time.monotonic()
            while True:
                loop_started = time.monotonic()
                sample = trajectory.sample(loop_started)
                planned_left = sample.joints[:7]
                planned_right = sample.joints[7:]
                feedback = session.read_feedback()
                last_feedback = feedback
                for index, serial in enumerate(feedback.frame_serials):
                    if serial != previous_serials[index]:
                        frame_advanced[index] = True
                    previous_serials[index] = serial
                _observe_safety(
                    safety,
                    feedback,
                    planned_left,
                    planned_right,
                    loop_started,
                )
                decision = safety.decide(now=loop_started)
                if decision.action == "soft_stop":
                    raise SmallMotionAcceptanceError(
                        "safety bridge requested soft stop: "
                        f"{decision.reason}"
                    )
                if decision.action != "send":
                    raise SmallMotionAcceptanceError(
                        "unexpected safety decision: "
                        f"{decision.action}:{decision.reason}"
                    )
                session.send_joint_targets(
                    decision.left_joints_deg,
                    decision.right_joints_deg,
                )
                command_batches_sent += 1
                output_all = np.concatenate(
                    [decision.left_joints_deg, decision.right_joints_deg]
                )
                signed_errors = output_all - _feedback_vector(feedback)
                flat_index = int(np.argmax(np.abs(signed_errors)))
                tracking_error = float(abs(signed_errors[flat_index]))
                if tracking_error > maximum_tracking_error:
                    maximum_tracking_error = tracking_error
                    side_index, joint_offset = divmod(flat_index, 7)
                    maximum_tracking_error_joint = {
                        "arm": "A" if side_index == 0 else "B",
                        "joint": joint_offset + 1,
                        "signed_error_deg": float(signed_errors[flat_index]),
                    }
                if tracking_error > options.maximum_tracking_error_deg:
                    raise SmallMotionAcceptanceError(
                        f"tracking error exceeded acceptance limit during "
                        f"{label}: {tracking_error:.4f} deg"
                    )

                endpoint_error = float(
                    np.max(
                        np.abs(_feedback_vector(feedback) - goal_all),
                        initial=0.0,
                    )
                )
                if loop_started >= next_print:
                    print(
                        json.dumps(
                            {
                                "status": "SMALL_MOTION_RUNNING",
                                "phase": label,
                                "command_batches_sent": command_batches_sent,
                                "endpoint_error_deg": round(
                                    endpoint_error, 6
                                ),
                                "maximum_tracking_error_deg": round(
                                    maximum_tracking_error, 6
                                ),
                                "frame_serials": feedback.frame_serials,
                                "arm_states": feedback.arm_states,
                                "error_codes": feedback.error_codes,
                                "safety_decision": decision.reason,
                            },
                            ensure_ascii=False,
                        ),
                        flush=True,
                    )
                    next_print = loop_started + 1.0
                if sample.complete:
                    if endpoint_started is None:
                        endpoint_started = loop_started
                    if (
                        loop_started - endpoint_started
                        >= options.hold_duration
                    ):
                        if endpoint_error > options.position_tolerance_deg:
                            raise SmallMotionAcceptanceError(
                                f"{label} endpoint was not reached: "
                                f"error={endpoint_error:.4f} deg"
                            )
                        return feedback, endpoint_error
                delay = period - (time.monotonic() - loop_started)
                if delay > 0.0:
                    time.sleep(delay)

        stage = "moving_outbound"
        outbound_feedback, outbound_error = run_leg(
            label="outbound",
            start_all=reference_all,
            goal_all=target_all,
        )
        outbound_target_reached = True
        stage = "returning_to_reference"
        last_feedback, return_error = run_leg(
            label="return",
            start_all=target_all,
            goal_all=reference_all,
        )
        returned_to_reference = True
        if not all(frame_advanced):
            raise SmallMotionAcceptanceError(
                "feedback frames did not advance for both arms: "
                f"{frame_advanced}"
            )
        stage = "verified_shutdown"
        shutdown_feedback = session.shutdown_verified()
        last_feedback = shutdown_feedback
        shutdown_verified = (
            shutdown_feedback.arm_states == (0, 0)
            and shutdown_feedback.error_codes == (0, 0)
        )
        if not shutdown_verified:
            raise SmallMotionAcceptanceError(
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

    success = (
        failure_reason is None
        and outbound_target_reached
        and returned_to_reference
        and shutdown_verified
    )
    selected_index = options.joint - 1

    def selected_value(values):
        if values is None:
            return None
        return float(values[selected_index])

    final_status = "SMALL_MOTION_SUCCESS" if success else "SMALL_MOTION_FAILED"
    result = {
        "status": final_status,
        "success": success,
        "failure_stage": None if success else stage,
        "reason": failure_reason,
        "arm": options.arm,
        "joint": options.joint,
        "delta_deg": options.delta_deg,
        "position_mode_entered": position_mode_entered,
        "initial_enable_error_deg": initial_enable_error,
        "outbound_target_reached": outbound_target_reached,
        "returned_to_reference": returned_to_reference,
        "outbound_endpoint_error_deg": outbound_error,
        "return_endpoint_error_deg": return_error,
        "reference_joint_deg": selected_value(
            reference_left if options.arm == "A" else reference_right
        ),
        "target_joint_deg": selected_value(
            target_left if options.arm == "A" else target_right
        ),
        "outbound_measured_joint_deg": (
            None
            if outbound_feedback is None
            else selected_value(
                outbound_feedback.left_joints_deg
                if options.arm == "A"
                else outbound_feedback.right_joints_deg
            )
        ),
        "final_measured_joint_deg": (
            None
            if last_feedback is None
            else selected_value(
                last_feedback.left_joints_deg
                if options.arm == "A"
                else last_feedback.right_joints_deg
            )
        ),
        "maximum_tracking_error_deg": maximum_tracking_error,
        "maximum_tracking_error_joint": maximum_tracking_error_joint,
        "frame_serial_advanced": {
            "A": frame_advanced[0],
            "B": frame_advanced[1],
        },
        "command_batches_sent": command_batches_sent,
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
