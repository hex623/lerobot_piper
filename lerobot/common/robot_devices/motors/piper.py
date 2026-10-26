"""Piper V2: passive connection by default; state/actions in radians/metres."""

import math
import threading
import time

import can
from piper_sdk import C_PiperInterface_V2

from lerobot.common.robot_devices.motors.configs import PiperMotorsBusConfig
from lerobot.common.robot_devices.motors.piper_diagnostics import query_device


class PiperMotorsBus:
    JOINT_IDS = (0x2A5, 0x2A6, 0x2A7)
    REQUIRED_IDS = (0x2A1, *JOINT_IDS, 0x2A8)
    STATUS_ID = 0x2A1
    GRIPPER_ID = 0x2A8
    # Existing local Piper X S-V1.9-0 profile. Verify for other arms/firmware.
    LIMITS_DEG = ((-150, 150), (0, 180), (-170, 0), (-89, 89), (-89, 89), (-170, 170))

    def __init__(self, config: PiperMotorsBusConfig):
        self.config, self.motors = config, config.motors
        self.piper = self._bus = self._thread = None
        self._stop, self._lock = threading.Event(), threading.Lock()
        self._frames = {}
        self._rx_error = self._fault = None
        self._motion_enabled = False
        if not 1 <= config.speed_percent <= 5:
            raise ValueError("speed_percent must be in [1, 5]")
        if not 0 < config.max_step_deg <= 1:
            raise ValueError("max_step_deg must be in (0, 1]")
        if not 0 < config.feedback_timeout_s <= 0.5:
            raise ValueError("feedback_timeout_s must be in (0, 0.5]")

    @property
    def motor_names(self):
        return list(self.motors)

    @property
    def motor_models(self):
        return [model for _, model in self.motors.values()]

    @property
    def motor_indices(self):
        return [index for index, _ in self.motors.values()]

    def connect(self, enable=False):
        if enable and not self.config.allow_motion:
            raise PermissionError("Motion is disabled in the Piper bus configuration")
        if self._bus is not None:
            raise RuntimeError("Piper is already connected")
        self._frames.clear()
        self._stop.clear()
        self._rx_error = self._fault = None
        self._bus = can.Bus(channel=self.config.can_name, interface="socketcan",
                            can_filters=[{"can_id": i, "can_mask": 0x7FF, "extended": False}
                                         for i in (*self.REQUIRED_IDS, *range(0x261, 0x267), *getattr(self, "EXTRA_IDS", ()))])
        self._thread = threading.Thread(target=self._receive, daemon=True)
        self._thread.start()
        try:
            deadline = time.monotonic() + 3
            while True:
                try:
                    self.read()
                    break
                except RuntimeError:
                    if time.monotonic() >= deadline:
                        raise
                    time.sleep(0.02)
            if enable:
                self._enable_at_current_pose()
        except BaseException:
            self.safe_disconnect()
            raise
        return True

    def _receive(self):
        try:
            while not self._stop.is_set():
                message = self._bus.recv(timeout=0.05)
                if message is None or message.is_error_frame or message.is_remote_frame or message.is_extended_id:
                    continue
                if len(message.data) == 8:
                    self._accept_frame(message.arbitration_id, bytes(message.data), time.monotonic())
        except Exception as exc:
            self._rx_error = str(exc)

    def _accept_frame(self, frame_id, data, timestamp):
        with self._lock:
            self._frames[frame_id] = (timestamp, data)

    def _snapshot(self, required=None):
        if self._bus is None:
            raise RuntimeError("Piper is not connected")
        if self._rx_error:
            raise RuntimeError(f"CAN receive failed: {self._rx_error}")
        now = time.monotonic()
        with self._lock:
            frames = dict(self._frames)
        for frame_id in (required or self.REQUIRED_IDS):
            if frame_id not in frames or now - frames[frame_id][0] > self.config.feedback_timeout_s:
                raise RuntimeError(f"Missing/stale Piper feedback: 0x{frame_id:03X}")
        return {key: value[1] for key, value in frames.items()}

    def read(self):
        frames, values = self._snapshot(), []
        for frame_id in self.JOINT_IDS:
            data = frames[frame_id]
            values.extend(math.radians(int.from_bytes(data[i:i + 4], "big", signed=True) / 1000)
                          for i in (0, 4))
        values.append(int.from_bytes(frames[self.GRIPPER_ID][:4], "big", signed=True) / 1_000_000)
        return dict(zip(("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "gripper"), values))

    def status(self):
        data = self._snapshot()[self.STATUS_ID]
        return {"control_mode": data[0], "arm_status": data[1],
                "error_code": int.from_bytes(data[6:8], "big"),
                "motion_enabled_by_this_connection": self._motion_enabled}

    def _check_motion_feedback(self, require_enabled=True):
        if self._fault:
            raise RuntimeError(f"Motion fault is latched: {self._fault}")
        state, status = self.read(), self.status()
        if status["arm_status"] != 0 or status["error_code"] != 0:
            raise RuntimeError(f"Piper is not ready: {status}")
        frames = self._snapshot((*self.REQUIRED_IDS, *range(0x261, 0x267)))
        if require_enabled and not all(frames[i][5] & 0x40 for i in range(0x261, 0x267)):
            raise RuntimeError("A Piper motor is disabled")
        self._validate_target(list(state.values()), state)
        return state

    def _enable_at_current_pose(self):
        current = self._check_motion_feedback(require_enabled=False)
        device = query_device(self.config.can_name)
        if not device["single_reply_per_motor"] or device["firmware_candidates"] != ["S-V1.9-0"]:
            raise RuntimeError("Piper motion blocked: ambiguous/missing device replies or unverified firmware; isolate the target controller first")
        for index, limits in enumerate(self.LIMITS_DEG):
            actual = device["joint_limits"][index]["variants"][0]
            if limits[0] < actual["min_deg"] or limits[1] > actual["max_deg"]:
                raise RuntimeError(f"Configured limits for joint {index + 1} exceed the device limits")
        current = self._check_motion_feedback(require_enabled=False)
        self.piper = C_PiperInterface_V2(self.config.can_name, start_sdk_joint_limit=False,
                                        start_sdk_gripper_limit=False)
        self.piper.ConnectPort(can_init=False, piper_init=False, start_thread=True)
        self._stream(list(current.values()), 0.10, require_enabled=False)
        self.piper.EnableArm(7, 2)
        deadline = time.monotonic() + 2
        while True:
            try:
                self._check_motion_feedback()
                break
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
                self._stream(list(current.values()), 0.02, require_enabled=False)
        self._motion_enabled = True

    def _validate_target(self, target, current):
        if len(target) != 7 or not all(math.isfinite(v) for v in target):
            raise ValueError("Expected 7 finite values: joint radians, gripper metres")
        for i, (low, high) in enumerate(self.LIMITS_DEG):
            if not low <= math.degrees(target[i]) <= high:
                raise ValueError(f"Joint {i + 1} exceeds the configured Piper X limits")
            if abs(target[i] - list(current.values())[i]) > math.radians(self.config.max_step_deg):
                raise ValueError(f"Joint {i + 1} step exceeds {self.config.max_step_deg} degree")
        if not 0 <= target[6] <= 0.08:
            raise ValueError("Gripper target must be in [0, 0.08] metres")
        if not self.config.allow_gripper and abs(target[6] - current["gripper"]) > 1e-5:
            raise PermissionError("Gripper motion requires allow_gripper=true after calibration")
        if abs(target[6] - current["gripper"]) > 0.001:
            raise ValueError("Gripper step exceeds 1 mm")

    def _stream(self, target, duration, require_enabled=True):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self._check_motion_feedback(require_enabled=require_enabled)
            self.piper.MotionCtrl_2(0x01, 0x01, self.config.speed_percent, 0x00, 0, 0x01)
            self.piper.JointCtrl(*(round(math.degrees(v) * 1000) for v in target[:6]))
            if self.config.allow_gripper:
                self.piper.GripperCtrl(round(target[6] * 1_000_000), 1000, 0x01, 0)
            time.sleep(0.005)

    def write(self, target_joint):
        if not self._motion_enabled or not self.config.allow_motion:
            raise PermissionError("Piper is read-only; configure and enable motion explicitly")
        try:
            current = self._check_motion_feedback()
            target = [float(v) for v in target_joint]
            self._validate_target(target, current)
            self._stream(target, 0.03)
        except Exception as exc:
            self._fault = str(exc)
            raise

    def apply_calibration(self):
        raise RuntimeError("Automatic homing is disabled; calibrate the arm explicitly")

    def safe_disconnect(self):
        # Never move the arm or remove holding torque while closing a connection.
        self._motion_enabled = False
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1)
        if self.piper is not None:
            self.piper.DisconnectPort()
            self.piper = None
        if self._bus is not None:
            self._bus.shutdown()
            self._bus = None

    def set_calibration(self):
        raise RuntimeError("Use the manufacturer calibration procedure")

    def revert_calibration(self):
        raise RuntimeError("Use the manufacturer calibration procedure")
