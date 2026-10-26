"""Read real Piper observations through LeRobot; never enables/sends actions."""

import argparse
import json
import time
from datetime import datetime, timezone
from pathlib import Path

from PIL import Image

from lerobot.common.robot_devices.cameras.configs import OpenCVCameraConfig
from lerobot.common.robot_devices.robots.configs import PiperRobotConfig
from lerobot.common.robot_devices.robots.piper import PiperRobot


def tx_count(channel):
    return int((Path("/sys/class/net") / channel / "statistics/tx_packets").read_text())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--can", default=None)
    parser.add_argument("--profile", help="local hardware JSON; stable camera paths survive USB renumbering")
    parser.add_argument("--camera", action="append", default=[], help="name=/dev/videoN; repeat for each RGB camera")
    parser.add_argument("--mjpeg-camera", action="append", default=[], help="camera names requiring MJPG over USB/IP")
    parser.add_argument("--samples", type=int, default=30)
    parser.add_argument("--output", default="outputs/local-preflight")
    args = parser.parse_args()
    profile = json.loads(Path(args.profile).read_text(encoding="utf-8")) if args.profile else {}
    args.can = args.can or profile.get("can_name", "can0")
    if args.samples < 2:
        parser.error("--samples must be at least 2")
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    cameras = {name: OpenCVCameraConfig(camera_index=item["device"], fourcc=item.get("fourcc"),
                                       width=640, height=480, fps=30)
               for name, item in profile.get("cameras", {}).items()}
    for item in args.camera:
        name, node = item.split("=", 1)
        cameras[name] = OpenCVCameraConfig(camera_index=node,
                                           width=640, height=480, fps=30,
                                           fourcc="MJPG" if name in args.mjpeg_camera else None)
    config = PiperRobotConfig(inference_time=True, cameras=cameras)
    config.follower_arm["main"].can_name = args.can
    config.follower_arm["main"].shared_can_teaching = profile.get("shared_can_teaching", False)
    config.follower_arm["main"].leader_control_offset = profile.get("leader_control_offset", 0)
    config.follower_arm["main"].follower_feedback_offset = profile.get("follower_feedback_offset", 0)
    robot = PiperRobot(config)
    before_tx = tx_count(args.can)
    report = {"utc": datetime.now(timezone.utc).isoformat(), "can": args.can,
              "scope": "read_only_observations", "motion_validation": "not performed",
              "motion_commands_requested": False, "state_units": ["rad"] * 6 + ["m"],
              "cameras": {k: v.camera_index for k, v in cameras.items()}}
    samples = []
    started = time.monotonic()
    try:
        robot.connect()
        for index in range(args.samples):
            observation = robot.capture_observation()
            state = observation["observation.state"].tolist()
            samples.append({"index": index, "monotonic_s": time.monotonic(), "state": state})
            if index == args.samples - 1:
                report["status"] = robot.arm.status()
                report["observation_shapes"] = {k: list(v.shape) for k, v in observation.items()}
                for key, value in observation.items():
                    if key.startswith("observation.images."):
                        Image.fromarray(value.numpy()).save(output / (key.rsplit(".", 1)[-1] + ".png"))
            time.sleep(1 / 30)
        report["passed"] = True
        report["image_visual_review"] = "required; transport and tensor shape checks alone do not prove image quality"
    except Exception as exc:
        report.update(passed=False, error=str(exc))
        raise
    finally:
        robot.disconnect()
        report["samples"] = len(samples)
        report["elapsed_s"] = time.monotonic() - started
        report["interface_tx_packets_delta"] = tx_count(args.can) - before_tx
        report["last_state"] = samples[-1]["state"] if samples else None
        (output / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        (output / "observations.jsonl").write_text("".join(json.dumps(s) + "\n" for s in samples), encoding="utf-8")
        print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
