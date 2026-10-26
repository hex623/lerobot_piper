import math
import time
import unittest
from unittest.mock import Mock, patch

from lerobot.common.robot_devices.motors.configs import PiperMotorsBusConfig
from lerobot.common.robot_devices.motors.piper_shared import SharedCANTargets, SharedCANPiperBus


def pair(a, b):
    return a.to_bytes(4, "big", signed=True) + b.to_bytes(4, "big", signed=True)


def feed_targets(tracker, timestamp=100):
    for index, values in enumerate(((90000, 10000), (-30000, 4000), (5000, -6000))):
        tracker.ingest(tracker.joint_ids[index], pair(*values), timestamp + index * .001)
    tracker.ingest(tracker.gripper_id, (12000).to_bytes(4, "big") + b"\x03\xe8\x01\x00", timestamp + .003)


class SharedCANTests(unittest.TestCase):
    def test_target_is_actual_bus_command_in_radians_and_metres(self):
        tracker = SharedCANTargets()
        feed_targets(tracker)
        action, metadata = tracker.snapshot(100.1)
        self.assertAlmostEqual(action[0], math.pi / 2)
        self.assertAlmostEqual(action[2], -math.pi / 6)
        self.assertAlmostEqual(action[6], .012)
        self.assertAlmostEqual(metadata["joint_pair_skew_s"], .002)

    def test_stationary_target_retains_real_age(self):
        tracker = SharedCANTargets()
        feed_targets(tracker)
        a, first = tracker.snapshot(101)
        b, second = tracker.snapshot(110)
        self.assertEqual(a, b)
        self.assertEqual(first["sequence"], second["sequence"])
        self.assertEqual(second["joint_command_age_s"], 10)

    def test_independent_pair_updates_keep_unchanged_targets(self):
        tracker = SharedCANTargets()
        feed_targets(tracker)
        tracker.ingest(0x155, pair(10000, 20000), 101)
        action, metadata = tracker.snapshot(101)
        self.assertAlmostEqual(action[0], math.radians(10))
        self.assertAlmostEqual(action[2], math.radians(-30))
        self.assertGreater(metadata["joint_pair_skew_s"], .9)

    def test_independently_latched_pairs_expose_receipt_skew(self):
        tracker = SharedCANTargets()
        for index, frame_id in enumerate(tracker.joint_ids):
            tracker.ingest(frame_id, pair(10000, 20000), 100 + index)
        tracker.ingest(tracker.gripper_id, (12000).to_bytes(4, "big") + b"\x03\xe8\x01\x00", 102)
        _, metadata = tracker.snapshot(103)
        self.assertEqual(metadata["joint_pair_skew_s"], 2)

    def test_missing_gripper_is_not_follower_feedback(self):
        tracker = SharedCANTargets()
        for index, frame_id in enumerate(tracker.joint_ids):
            tracker.ingest(frame_id, pair(10000, 20000), 100 + index * .001)
        tracker.ingest(0x2A8, bytes(8), 100)
        with self.assertRaisesRegex(RuntimeError, "gripper"):
            tracker.snapshot(100.1)

    def test_non_joint_mode_and_estop_invalidate_targets(self):
        tracker = SharedCANTargets()
        feed_targets(tracker)
        tracker.ingest(0x150, b"\x01" + bytes(7), 100.2)
        with self.assertRaises(RuntimeError):
            tracker.snapshot(100.3)
        feed_targets(tracker, 101)
        tracker.ingest(0x151, b"\x01\x00" + bytes(6), 101.1)
        with self.assertRaises(RuntimeError):
            tracker.snapshot(101.2)

    def test_offsets_keep_command_and_feedback_namespaces_separate(self):
        tracker = SharedCANTargets(0x10)
        feed_targets(tracker)
        action, _ = tracker.snapshot(100.1)
        self.assertAlmostEqual(action[0], math.pi / 2)
        self.assertEqual(tracker.joint_ids, (0x165, 0x166, 0x167))

    def test_shared_bus_never_enables_or_writes(self):
        arm = SharedCANPiperBus(PiperMotorsBusConfig("can0", {}, shared_can_teaching=True))
        with patch("can.Bus") as bus, patch("lerobot.common.robot_devices.motors.piper.C_PiperInterface_V2") as sdk:
            with self.assertRaises(PermissionError):
                arm.connect(enable=True)
            with self.assertRaises(PermissionError):
                arm.write([0] * 7)
            arm.safe_disconnect()
            bus.assert_not_called()
            sdk.assert_not_called()

    def test_feedback_and_action_are_different_values(self):
        arm = SharedCANPiperBus(PiperMotorsBusConfig("can0", {}, shared_can_teaching=True))
        arm._bus = Mock()
        now = time.monotonic()
        for frame_id in arm.REQUIRED_IDS:
            data = b"\x01\x00\x01" + bytes(5) if frame_id == 0x2A1 else bytes(8)
            arm._accept_frame(frame_id, data, now)
        feed_targets(arm.targets, now)
        self.assertEqual(arm.read()["joint_1"], 0)
        self.assertAlmostEqual(arm.read_shared_action()[0][0], math.pi / 2)

    def test_constructor_rejects_accidental_motion_configuration(self):
        with self.assertRaises(ValueError):
            SharedCANPiperBus(PiperMotorsBusConfig("can0", {}, allow_motion=True, shared_can_teaching=True))

    def test_older_history_cannot_overwrite_a_live_command(self):
        tracker = SharedCANTargets()
        feed_targets(tracker, 100)
        tracker.ingest(0x155, pair(10000, 20000), 110)
        feed_targets(tracker, 101)
        action, _ = tracker.snapshot(111)
        self.assertAlmostEqual(action[0], math.radians(10))


if __name__ == "__main__":
    unittest.main()
