"""
    Teleoperation Agilex Piper with a PS5 controller    
"""

import time
import torch
import numpy as np
from dataclasses import dataclass, field, replace

from lerobot.common.robot_devices.teleop.gamepad import SixAxisArmController
from lerobot.common.robot_devices.motors.utils import get_motor_names, make_motors_buses_from_configs
from lerobot.common.robot_devices.cameras.utils import make_cameras_from_configs
from lerobot.common.robot_devices.utils import RobotDeviceAlreadyConnectedError, RobotDeviceNotConnectedError
from lerobot.common.robot_devices.robots.configs import PiperRobotConfig

class PiperRobot:
    def __init__(self, config: PiperRobotConfig | None = None, **kwargs):
        if config is None:
            config = PiperRobotConfig()
        # Overwrite config arguments using kwargs
        self.config = replace(config, **kwargs)
        self.robot_type = self.config.type
        self.inference_time = self.config.inference_time # if it is inference time
        
        # build cameras
        self.cameras = make_cameras_from_configs(self.config.cameras)
        
        # build piper motors
        self.piper_motors = make_motors_buses_from_configs(self.config.follower_arm)
        self.arm = self.piper_motors['main']
        
        # Open hardware only in connect(); a read-only session needs no gamepad.
        self.teleop = None
        
        self.logs = {}
        self.is_connected = False

    @property
    def camera_features(self) -> dict:
        cam_ft = {}
        for cam_key, cam in self.cameras.items():
            key = f"observation.images.{cam_key}"
            cam_ft[key] = {
                "shape": (cam.height, cam.width, cam.channels),
                "names": ["height", "width", "channels"],
                "info": None,   
            }
        return cam_ft

    
    @property
    def motor_features(self) -> dict:
        action_names = get_motor_names(self.piper_motors)
        state_names = get_motor_names(self.piper_motors)
        return {
            "action": {
                "dtype": "float32",
                "shape": (len(action_names),),
                "names": action_names,
            },
            "observation.state": {
                "dtype": "float32",
                "shape": (len(state_names),),
                "names": state_names,
            },
        }
    
    @property
    def has_camera(self):
        return len(self.cameras) > 0

    @property
    def num_cameras(self):
        return len(self.cameras)


    def connect(self) -> None:
        """Connect piper and cameras"""
        if self.is_connected:
            raise RobotDeviceAlreadyConnectedError(
                "Piper is already connected. Do not run `robot.connect()` twice."
            )
        
        try:
            self.arm.connect(enable=self.arm.config.allow_motion)
            for camera in self.cameras.values():
                camera.connect()
            if not self.inference_time and not self.arm.config.shared_can_teaching:
                if not self.arm.config.allow_motion:
                    raise PermissionError("Gamepad teleoperation requires explicit allow_motion=true")
                self.teleop = SixAxisArmController(initial_state=list(self.arm.read().values()))
            self.is_connected = True
        except BaseException:
            self.disconnect()
            raise


    def disconnect(self) -> None:
        """Close devices without homing or changing motor enable state."""
        if self.teleop is not None:
            self.teleop.stop()
            self.teleop = None
        self.arm.safe_disconnect()
        for cam in self.cameras.values():
            if cam.is_connected:
                cam.disconnect()
        self.is_connected = False


    def run_calibration(self):
        """move piper to the home position"""
        if not self.is_connected:
            raise ConnectionError()
        
        raise RuntimeError("Automatic Piper homing is disabled; use explicit manufacturer calibration")



    def teleop_step(
        self, record_data=False
    ) -> None | tuple[dict[str, torch.Tensor], dict[str, torch.Tensor]]:
        if not self.is_connected:
            raise ConnectionError()
        
        if self.arm.config.shared_can_teaching:
            started = time.monotonic()
            target, provenance = self.arm.read_shared_action()
            observation = self.capture_observation()
            provenance.update(observation_started_monotonic_s=started,
                              observation_finished_monotonic_s=time.monotonic())
            provenance["camera_receipt_monotonic_s"] = {
                name: camera.logs.get("capture_monotonic_s") for name, camera in self.cameras.items()}
            now = time.monotonic()
            if any(t is None or now - t > .5 for t in provenance["camera_receipt_monotonic_s"].values()):
                raise RuntimeError("Camera feedback is stale during shared CAN teaching")
            self.logs["shared_can_action"] = provenance
            if record_data:
                return observation, {"action": torch.tensor(target, dtype=torch.float32)}
            return None

        if self.teleop is None:
            raise RuntimeError("No gamepad session; connect with inference_time=false and allow_motion=true")

        # read target pose state as 
        before_read_t = time.perf_counter()
        state = self.arm.read() # read current joint position from robot
        action = self.teleop.get_action() # target joint position from gamepad
        self.logs["read_pos_dt_s"] = time.perf_counter() - before_read_t

        # do action
        before_write_t = time.perf_counter()
        target_joints = list(action.values())
        self.arm.write(target_joints)
        self.logs["write_pos_dt_s"] = time.perf_counter() - before_write_t

        if not record_data:
            return
        
        state = torch.as_tensor(list(state.values()), dtype=torch.float32)
        action = torch.as_tensor(list(action.values()), dtype=torch.float32)

        # Capture images from cameras
        images = {}
        for name in self.cameras:
            before_camread_t = time.perf_counter()
            images[name] = self.cameras[name].async_read()
            images[name] = torch.from_numpy(images[name])
            self.logs[f"read_camera_{name}_dt_s"] = self.cameras[name].logs["delta_timestamp_s"]
            self.logs[f"async_read_camera_{name}_dt_s"] = time.perf_counter() - before_camread_t

        # Populate output dictionnaries
        obs_dict, action_dict = {}, {}
        obs_dict["observation.state"] = state
        action_dict["action"] = action
        for name in self.cameras:
            obs_dict[f"observation.images.{name}"] = images[name]

        return obs_dict, action_dict



    def send_action(self, action: torch.Tensor) -> torch.Tensor:
        """Write the predicted actions from policy to the motors"""
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                "Piper is not connected. You need to run `robot.connect()`."
            )

        # send to motors, torch to list
        target_joints = action.tolist()
        self.arm.write(target_joints)

        return action



    def capture_observation(self) -> dict:
        """capture current images and joint positions"""
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                "Piper is not connected. You need to run `robot.connect()`."
            )
        
        # read current joint positions
        before_read_t = time.perf_counter()
        state = self.arm.read()  # 6 joints + 1 gripper
        self.logs["read_pos_dt_s"] = time.perf_counter() - before_read_t

        state = torch.as_tensor(list(state.values()), dtype=torch.float32)

        # read images from cameras
        images = {}
        for name in self.cameras:
            before_camread_t = time.perf_counter()
            images[name] = self.cameras[name].async_read()
            images[name] = torch.from_numpy(images[name])
            self.logs[f"read_camera_{name}_dt_s"] = self.cameras[name].logs["delta_timestamp_s"]
            self.logs[f"async_read_camera_{name}_dt_s"] = time.perf_counter() - before_camread_t

        # Populate output dictionnaries and format to pytorch
        obs_dict = {}
        obs_dict["observation.state"] = state
        for name in self.cameras:
            obs_dict[f"observation.images.{name}"] = images[name]
        return obs_dict
    
    def teleop_safety_stop(self):
        """Keep the current pose; episode boundaries must never auto-home."""
        if self.arm.config.allow_motion:
            self.arm.write(list(self.arm.read().values()))

    
    def __del__(self):
        if getattr(self, "is_connected", False):
            self.disconnect()
