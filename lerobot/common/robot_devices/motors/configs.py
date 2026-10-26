import abc
from dataclasses import dataclass

import draccus


@dataclass
class MotorsBusConfig(draccus.ChoiceRegistry, abc.ABC):
    @property
    def type(self) -> str:
        return self.get_choice_name(self.__class__)


@MotorsBusConfig.register_subclass("dynamixel")
@dataclass
class DynamixelMotorsBusConfig(MotorsBusConfig):
    port: str
    motors: dict[str, tuple[int, str]]
    mock: bool = False


@MotorsBusConfig.register_subclass("feetech")
@dataclass
class FeetechMotorsBusConfig(MotorsBusConfig):
    port: str
    motors: dict[str, tuple[int, str]]
    mock: bool = False


@MotorsBusConfig.register_subclass("piper")
@dataclass
class PiperMotorsBusConfig(MotorsBusConfig):
    can_name: str
    motors: dict[str, tuple[int, str]]
    allow_motion: bool = False
    allow_gripper: bool = False
    speed_percent: int = 2
    feedback_timeout_s: float = 0.5
    max_step_deg: float = 1.0
    shared_can_teaching: bool = False
    leader_control_offset: int = 0
    follower_feedback_offset: int = 0
