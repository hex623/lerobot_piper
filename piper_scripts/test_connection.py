"""Regression checks using synthetic CAN feedback. Never opens real hardware."""

import math
import time
import unittest
from unittest.mock import Mock, patch

from lerobot.common.robot_devices.motors.configs import PiperMotorsBusConfig
from lerobot.common.robot_devices.motors.piper import PiperMotorsBus


class ConnectionTests(unittest.TestCase):
    def arm(self, **kwargs):
        arm = PiperMotorsBus(PiperMotorsBusConfig("can0", {}, **kwargs))
        arm._bus = Mock()
        joints = (10000, 20000, -30000, 4000, 5000, -6000)
        frames = {0x2A1: bytes(8), 0x2A8: (2700).to_bytes(4, "big") + bytes(4)}
        for i, frame_id in enumerate(arm.JOINT_IDS):
            frames[frame_id] = b"".join(v.to_bytes(4, "big", signed=True) for v in joints[2*i:2*i+2])
        frames.update({i: bytes(5) + b"\x40\x00\x00" for i in range(0x261, 0x267)})
        arm._frames = {i: (time.monotonic(), data) for i, data in frames.items()}
        return arm

    def test_constructor_opens_no_hardware(self):
        with patch("can.Bus") as bus, patch("lerobot.common.robot_devices.motors.piper.C_PiperInterface_V2") as sdk:
            PiperMotorsBus(PiperMotorsBusConfig("can0", {}))
            bus.assert_not_called()
            sdk.assert_not_called()

    def test_units_are_radians_and_metres(self):
        state = self.arm().read()
        self.assertAlmostEqual(state["joint_1"], math.radians(10))
        self.assertAlmostEqual(state["joint_3"], math.radians(-30))
        self.assertAlmostEqual(state["gripper"], 0.0027)

    def test_each_joint_pair_must_be_fresh(self):
        arm = self.arm()
        timestamp, data = arm._frames[0x2A6]
        arm._frames[0x2A6] = (timestamp - 1, data)
        with self.assertRaisesRegex(RuntimeError, "0x2A6"):
            arm.read()

    def test_missing_gripper_is_not_zero(self):
        arm = self.arm()
        del arm._frames[0x2A8]
        with self.assertRaisesRegex(RuntimeError, "0x2A8"):
            arm.read()

    def test_readonly_rejects_write_and_enable(self):
        arm = self.arm()
        with self.assertRaises(PermissionError):
            arm.write(list(arm.read().values()))
        with self.assertRaises(PermissionError):
            arm.connect(enable=True)
        arm._bus.send.assert_not_called()

    def test_close_transmits_no_disable_or_home(self):
        arm = self.arm()
        arm.piper = Mock()
        sdk, bus = arm.piper, arm._bus
        arm.safe_disconnect()
        sdk.DisconnectPort.assert_called_once()
        sdk.JointCtrl.assert_not_called()
        sdk.DisableArm.assert_not_called()
        bus.send.assert_not_called()
        bus.shutdown.assert_called_once()

    def test_bad_actions_rejected(self):
        arm = self.arm(allow_motion=True)
        current = arm.read()
        for value in (float("nan"), float("inf"), math.radians(200)):
            target = list(current.values())
            target[0] = value
            with self.assertRaises(ValueError):
                arm._validate_target(target, current)
        target = list(current.values())
        target[0] += math.radians(2)
        with self.assertRaises(ValueError):
            arm._validate_target(target, current)
        target = list(current.values())
        target[6] += 0.0005
        with self.assertRaises(PermissionError):
            arm._validate_target(target, current)

    def test_estop_and_disabled_motor_block_motion(self):
        arm = self.arm(allow_motion=True)
        arm._frames[0x2A1] = (time.monotonic(), b"\x01\x01" + bytes(6))
        with self.assertRaisesRegex(RuntimeError, "not ready"):
            arm._check_motion_feedback()
        arm._frames[0x2A1] = (time.monotonic(), bytes(8))
        arm._frames[0x263] = (time.monotonic(), bytes(8))
        with self.assertRaisesRegex(RuntimeError, "disabled"):
            arm._check_motion_feedback()

    def test_fault_latches_after_rejected_write(self):
        arm = self.arm(allow_motion=True)
        arm._motion_enabled = True
        arm.piper = Mock()
        with self.assertRaises(ValueError):
            arm.write([float("nan")] * 7)
        with self.assertRaisesRegex(RuntimeError, "latched"):
            arm.write(list(arm.read().values()))
        arm.piper.JointCtrl.assert_not_called()

    def test_ambiguous_controller_blocks_enable_before_sdk_commands(self):
        arm = self.arm(allow_motion=True)
        with patch("lerobot.common.robot_devices.motors.piper.query_device", return_value={"single_reply_per_motor": False}), \
             patch("lerobot.common.robot_devices.motors.piper.C_PiperInterface_V2") as sdk:
            with self.assertRaisesRegex(RuntimeError, "ambiguous"):
                arm._enable_at_current_pose()
            sdk.assert_not_called()


if __name__ == "__main__":
    unittest.main()
