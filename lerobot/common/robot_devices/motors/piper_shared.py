"""Passive native master/slave teaching on one CAN bus. No command forwarding."""

import math
import threading
import time

from lerobot.common.robot_devices.motors.piper import PiperMotorsBus


class SharedCANTargets:
    """Latch actual joint/gripper control messages, never measured-state labels.

    The latched target remains a target while the master is stationary. Its
    real receipt age is exposed; a cached read never refreshes the timestamp.
    Each joint pair is updated independently by the device. Retain the last
    received target for unchanged pairs and expose every pair's real age.
    """

    def __init__(self, offset=0):
        if offset not in (0, 0x10, 0x20):
            raise ValueError("leader_control_offset must be 0, 16 or 32")
        self.joint_ids = tuple(i + offset for i in (0x155, 0x156, 0x157))
        self.gripper_id, self.mode_id = 0x159 + offset, 0x151 + offset
        self.ids = (*self.joint_ids, self.gripper_id, self.mode_id, 0x150)
        self.lock = threading.Lock()
        self.parts, self.counts, self.last_receipts = {}, {}, {}
        self.joints = self.gripper = self.mode = None
        self.sequence = 0
        self.stopped = False

    def ingest(self, frame_id, data, timestamp):
        if frame_id not in self.ids or len(data) != 8:
            return
        with self.lock:
            if timestamp < self.last_receipts.get(frame_id, float("-inf")):
                return
            if frame_id in (0x150, self.mode_id) and timestamp < max(self.last_receipts.values(), default=float("-inf")):
                return
            self.last_receipts[frame_id] = timestamp
            self.counts[frame_id] = self.counts.get(frame_id, 0) + 1
            if frame_id == 0x150 and data[0] == 1:
                self.stopped = True
                self.joints = None
                self.gripper = None
                self.parts.clear()
            elif frame_id == self.mode_id:
                self.mode = (timestamp, data)
                if data[0:2] != b"\x01\x01":
                    self.joints = None
                    self.gripper = None
                    self.parts.clear()
            elif frame_id == self.gripper_id:
                self.gripper = (timestamp, data)
            elif frame_id in self.joint_ids:
                self.parts[frame_id] = (timestamp, data)
                if len(self.parts) == 3:
                    frames = [self.parts[i] for i in self.joint_ids]
                    self.joints = tuple(frames)
                    self.sequence += 1
                    # A subsequent complete target may resume capture only
                    # when current follower status and mode also allow it.
                    self.stopped = False

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            if self.stopped or self.joints is None:
                raise RuntimeError("Waiting for all three master joint command pairs (0x155/156/157 + offset)")
            if self.mode is not None and self.mode[1][:2] != b"\x01\x01":
                raise RuntimeError("Master control mode is not joint position mode")
            if self.gripper is None:
                raise RuntimeError("Waiting for master gripper command (0x159 + offset); operate the master gripper once")
            grip_time, grip_data = self.gripper
            if grip_data[6] not in (1, 3) or grip_data[7] != 0:
                raise RuntimeError("Master gripper target is disabled or zero calibration is active")
            action = [math.radians(int.from_bytes(data[start:start+4], "big", signed=True) / 1000)
                      for _, data in self.joints for start in (0, 4)]
            action.append(int.from_bytes(grip_data[:4], "big", signed=True) / 1_000_000)
            times = [t for t, _ in self.joints]
            metadata = {"source": "observed_bus_joint_and_gripper_control", "sequence": self.sequence,
                        "joint_receipt_monotonic_s": times, "gripper_receipt_monotonic_s": grip_time,
                        "joint_command_age_s": now - min(times), "gripper_command_age_s": now - grip_time,
                        "joint_pair_skew_s": max(times) - min(times),
                        "assembly": "latest_observed_target_for_each_independent_joint_pair",
                        "latched_target": True, "sender_identity_proven_by_can": False,
                        "joint_raw_hex": [data.hex() for _, data in self.joints], "gripper_raw_hex": grip_data.hex()}
            return action, metadata

    def diagnostics(self):
        with self.lock:
            return {"frame_counts": {f"0x{k:03X}": v for k, v in self.counts.items()},
                    "joint_target_updates": self.sequence, "has_gripper_command": self.gripper is not None,
                    "known_joint_pairs": len(self.parts)}


class SharedCANPiperBus(PiperMotorsBus):
    def __init__(self, config):
        if config.allow_motion or config.allow_gripper:
            raise ValueError("Shared CAN teaching must use allow_motion=false and allow_gripper=false")
        super().__init__(config)
        if config.follower_feedback_offset not in (0, 0x10, 0x20):
            raise ValueError("follower_feedback_offset must be 0, 16 or 32")
        self.JOINT_IDS = tuple(i + config.follower_feedback_offset for i in (0x2A5, 0x2A6, 0x2A7))
        self.STATUS_ID = 0x2A1 + config.follower_feedback_offset
        self.GRIPPER_ID = 0x2A8 + config.follower_feedback_offset
        self.REQUIRED_IDS = (self.STATUS_ID, *self.JOINT_IDS, self.GRIPPER_ID)
        self.targets = SharedCANTargets(config.leader_control_offset)
        self.EXTRA_IDS = self.targets.ids

    def connect(self, enable=False):
        if enable:
            raise PermissionError("Shared CAN teaching never sends motor enable commands")
        if self._bus is not None:
            raise RuntimeError("Piper is already connected")
        self.targets = SharedCANTargets(self.config.leader_control_offset)
        return super().connect(enable=False)

    def _accept_frame(self, frame_id, data, timestamp):
        super()._accept_frame(frame_id, data, timestamp)
        self.targets.ingest(frame_id, data, timestamp)

    def read_shared_action(self, timeout=0.10):
        deadline = time.monotonic() + timeout
        while True:
            frames = self._snapshot()
            status = frames[self.STATUS_ID]
            if status[:3] != b"\x01\x00\x01" or int.from_bytes(status[6:8], "big"):
                raise RuntimeError("Follower is not in healthy CAN joint-control mode")
            try:
                return self.targets.snapshot()
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.001)

    def write(self, target_joint):
        raise PermissionError("Native shared CAN teaching is receive-only; the hardware master owns commands")

    def _enable_at_current_pose(self):
        raise PermissionError("Cannot enable or change modes on a shared master/slave teaching bus")
