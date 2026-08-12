import contextlib
import io
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

from marvin_hardware_backend.marvin_hardware import MarvinFeedback
from marvin_hardware_backend.small_motion_probe import (
    SmallMotionAcceptanceError,
    build_motion_target,
    main,
)


class BuildMotionTargetTest(unittest.TestCase):
    def setUp(self):
        self.left = np.array([10.0, 20.0, 30.0, 40.0, 50.0, 0.0, 0.0])
        self.right = -self.left
        self.lower = np.full(7, -100.0)
        self.upper = np.full(7, 100.0)

    def build(self, **overrides):
        values = {
            "reference_left": self.left,
            "reference_right": self.right,
            "arm": "A",
            "joint": 3,
            "delta_deg": 1.0,
            "lower_limits_deg": self.lower,
            "upper_limits_deg": self.upper,
            "limit_margin_deg": 2.0,
        }
        values.update(overrides)
        return build_motion_target(**values)

    def test_changes_only_selected_joint_and_does_not_mutate_reference(self):
        target_left, target_right = self.build()

        np.testing.assert_allclose(
            target_left,
            [10.0, 20.0, 31.0, 40.0, 50.0, 0.0, 0.0],
        )
        np.testing.assert_allclose(target_right, self.right)
        np.testing.assert_allclose(
            self.left, [10.0, 20.0, 30.0, 40.0, 50.0, 0.0, 0.0]
        )

    def test_supports_negative_offset_on_arm_b(self):
        target_left, target_right = self.build(
            arm="B", joint=1, delta_deg=-1.5
        )

        np.testing.assert_allclose(target_left, self.left)
        self.assertAlmostEqual(target_right[0], -11.5)
        np.testing.assert_allclose(target_right[1:], self.right[1:])

    def test_rejects_endpoint_inside_required_limit_margin(self):
        with self.assertRaisesRegex(
            SmallMotionAcceptanceError, "outside the configured limit margin"
        ):
            self.build(
                joint=1,
                delta_deg=1.0,
                upper_limits_deg=np.array(
                    [12.5, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0]
                ),
            )

    def test_rejects_reference_outside_configured_limits(self):
        left = self.left.copy()
        left[0] = 101.0
        with self.assertRaisesRegex(
            SmallMotionAcceptanceError, "measured reference"
        ):
            self.build(reference_left=left)

    def test_rejects_invalid_joint(self):
        with self.assertRaisesRegex(ValueError, "joint must"):
            self.build(joint=0)


class _FakeClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.now += max(0.0, float(duration))


class _FakeSession:
    def __init__(self):
        self.left = np.zeros(7, dtype=np.float64)
        self.right = np.zeros(7, dtype=np.float64)
        self.frame = 1
        self.maximum_a_joint_1 = 0.0
        self.sent = 0
        self.shutdown_verified_called = False
        self.shutdown_called = False
        self.soft_stop_called = False

    def feedback(self, state=1):
        self.frame += 1
        return MarvinFeedback(
            left_joints_deg=self.left.copy(),
            right_joints_deg=self.right.copy(),
            arm_states=(state, state),
            command_states=(-1, -1),
            error_codes=(0, 0),
            frame_serials=(self.frame, self.frame),
            velocity_ratios=(10, 10),
            acceleration_ratios=(10, 10),
            servo_error_reports=("None", "None"),
        )

    def connect_and_prepare(self, *_args, **_kwargs):
        reference = self.feedback()
        prepared = self.feedback()
        return reference, prepared

    def read_feedback(self):
        return self.feedback()

    def send_joint_targets(self, left, right):
        self.left = np.asarray(left, dtype=np.float64).copy()
        self.right = np.asarray(right, dtype=np.float64).copy()
        self.maximum_a_joint_1 = max(
            self.maximum_a_joint_1, float(self.left[0])
        )
        self.sent += 1

    def shutdown_verified(self):
        self.shutdown_verified_called = True
        return self.feedback(state=0)

    def soft_stop_once(self):
        self.soft_stop_called = True
        return True

    def shutdown(self):
        self.shutdown_called = True


class SmallMotionMainTest(unittest.TestCase):
    def test_complete_simulated_outbound_return_and_shutdown(self):
        fake_clock = _FakeClock()
        fake_session = _FakeSession()
        config = (
            Path(__file__).resolve().parents[1] / "config" / "real.yaml"
        )
        output = io.StringIO()
        args = [
            "--confirm-motion",
            "--arm",
            "A",
            "--joint",
            "1",
            "--delta-deg",
            "1.0",
            "--move-duration",
            "1.0",
            "--hold-duration",
            "0.5",
            "--config",
            str(config),
        ]

        with (
            patch(
                "marvin_hardware_backend.small_motion_probe.load_marvin_sdk",
                return_value=(object, object, None),
            ),
            patch(
                "marvin_hardware_backend.small_motion_probe."
                "MarvinHardwareSession",
                return_value=fake_session,
            ),
            patch(
                "marvin_hardware_backend.small_motion_probe.time.monotonic",
                side_effect=fake_clock.monotonic,
            ),
            patch(
                "marvin_hardware_backend.small_motion_probe.time.sleep",
                side_effect=fake_clock.sleep,
            ),
            contextlib.redirect_stdout(output),
        ):
            result = main(args)

        self.assertEqual(result, 0, output.getvalue())
        self.assertIn('"status": "SMALL_MOTION_SUCCESS"', output.getvalue())
        self.assertGreater(fake_session.sent, 0)
        self.assertAlmostEqual(fake_session.maximum_a_joint_1, 1.0)
        self.assertAlmostEqual(float(fake_session.left[0]), 0.0)
        self.assertTrue(fake_session.shutdown_verified_called)
        self.assertFalse(fake_session.soft_stop_called)
        self.assertFalse(fake_session.shutdown_called)


if __name__ == "__main__":
    unittest.main()
