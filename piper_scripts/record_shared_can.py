"""Record native shared-CAN teaching into a LOCAL LeRobot dataset, with zero TX."""

import argparse
import json
import os
import time
from datetime import datetime, timezone
from pathlib import Path

from PIL import Image

from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
from lerobot.common.robot_devices.cameras.configs import OpenCVCameraConfig
from lerobot.common.robot_devices.robots.configs import PiperRobotConfig
from lerobot.common.robot_devices.robots.piper import PiperRobot


def tx_count(channel):
    return int((Path("/sys/class/net") / channel / "statistics/tx_packets").read_text())


def load_live_history(arm, path):
    """Seed only real targets from a currently growing trace on this host.

    Intended for connectivity checks after the operator already demonstrated
    movement. Normal recording waits for fresh commands after connection.
    Follower feedback is never seeded from history or used as an action.
    """
    if time.time() - path.stat().st_mtime > 2:
        raise ValueError("--bus-history must be the active trace from this session")
    size = path.stat().st_size
    with path.open(encoding="utf-8") as stream:
        text = stream.read(size)
    rows = text.splitlines()
    if text and not text.endswith("\n"):
        rows = rows[:-1]
    now = time.monotonic()
    count = 0
    for row in rows:
        item = json.loads(row)
        if item["t"] > now:
            raise ValueError("History is not from this host's current monotonic clock")
        if item["id"] in arm.targets.ids:
            arm.targets.ingest(item["id"], bytes.fromhex(item["data"]), item["t"])
            count += 1
    return count


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", default="piper_scripts/local-hardware.json")
    parser.add_argument("--output", help="new run folder; existing folders are never overwritten")
    parser.add_argument("--repo-id", default="local/piper_shared_can")
    parser.add_argument("--task", default="Manual master-slave demonstration")
    parser.add_argument("--duration", type=float, default=30)
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--wait-for-leader", type=float, default=120)
    parser.add_argument("--bus-history", type=Path, help="active raw trace for a connectivity check only")
    args = parser.parse_args()
    if args.duration <= 0 or not 1 <= args.fps <= 30 or args.wait_for_leader < 0:
        parser.error("Use duration>0, fps in [1,30], wait-for-leader>=0")
    profile = json.loads(Path(args.profile).read_text(encoding="utf-8"))
    config = PiperRobotConfig(inference_time=True, cameras={
        name: OpenCVCameraConfig(camera_index=item["device"], fourcc=item.get("fourcc"),
                                 width=640, height=480, fps=30)
        for name, item in profile["cameras"].items()})
    bus_config = config.follower_arm["main"]
    bus_config.can_name = profile["can_name"]
    bus_config.shared_can_teaching = True
    bus_config.leader_control_offset = profile.get("leader_control_offset", 0)
    bus_config.follower_feedback_offset = profile.get("follower_feedback_offset", 0)
    run = Path(args.output or ("data/shared_can_" + datetime.now().strftime("%Y%m%d_%H%M%S")))
    run.mkdir(parents=True, exist_ok=False)
    robot, dataset = PiperRobot(config), None
    before_tx = tx_count(bus_config.can_name)
    report = {"utc": datetime.now(timezone.utc).isoformat(), "topology": "native_master_slave_shared_can",
              "program_can_transmit_calls": 0, "task": args.task, "task_outcome": "not_assessed",
              "training_eligible": False, "review_status": "unreviewed", "frames": 0,
              "action_units": ["rad"] * 6 + ["m"],
              "action_source": "observed_bus_joint_and_gripper_control",
              "clock": "dataset uses nominal frame_index/fps; provenance contains actual host monotonic times",
              "connectivity_check_with_history": args.bus_history is not None}
    report["nominal_fps"] = args.fps
    preview_images, frame_starts = {}, []
    camera_receipts = {name: set() for name in config.cameras}
    # Make the environment's ffmpeg available even without shell activation.
    os.environ["PATH"] = str(Path(os.sys.executable).parent) + os.pathsep + os.environ["PATH"]
    try:
        robot.connect()
        # Start camera receive threads before the timed dataset clock.
        robot.capture_observation()
        time.sleep(.15)
        if args.bus_history:
            report["history_command_frames"] = load_live_history(robot.arm, args.bus_history)
        print("Waiting for master joint targets and one gripper command; receive-only.", flush=True)
        deadline = time.monotonic() + args.wait_for_leader
        while True:
            try:
                robot.arm.read_shared_action()
                break
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(.05)
        dataset = LeRobotDataset.create(args.repo_id, args.fps, root=run / "dataset", robot=robot,
                                       use_videos=True, image_writer_threads=4)
        print("Recording local episode; hardware master continues to control the follower.", flush=True)
        started = time.monotonic()
        with (run / "provenance.jsonl").open("w", encoding="utf-8") as provenance:
            for index in range(max(2, round(args.duration * args.fps))):
                frame_started = time.monotonic()
                frame_starts.append(frame_started)
                observation, action = robot.teleop_step(record_data=True)
                source = dict(robot.logs["shared_can_action"])
                source["frame_index"] = index
                source["actual_elapsed_s"] = time.monotonic() - started
                source["observation_state"] = observation["observation.state"].tolist()
                source["action"] = action["action"].tolist()
                for name, timestamp in source["camera_receipt_monotonic_s"].items():
                    camera_receipts[name].add(timestamp)
                dataset.add_frame({**observation, **action, "task": args.task})
                provenance.write(json.dumps(source) + "\n")
                report["frames"] += 1
                if index == 0:
                    for key, value in observation.items():
                        if key.startswith("observation.images."):
                            preview_images[key.rsplit(".", 1)[-1]] = value.numpy().copy()
                # Never issue a burst of cached frames to catch up after an
                # I/O stall; keep actual timing alongside nominal dataset time.
                time.sleep(max(0, frame_started + 1 / args.fps - time.monotonic()))
        report["actual_capture_elapsed_s"] = time.monotonic() - started
        intervals = [b - a for a, b in zip(frame_starts, frame_starts[1:])]
        report["actual_frame_start_intervals_s"] = {"minimum": min(intervals), "maximum": max(intervals),
                                                   "mean": sum(intervals) / len(intervals)}
        report["unique_camera_receipts"] = {name: len(values) for name, values in camera_receipts.items()}
        for name, value in preview_images.items():
            Image.fromarray(value).save(run / (name + ".png"))
        dataset.save_episode()
        report["saved_episodes"] = dataset.num_episodes
        report["dataset_root"] = str((run / "dataset").resolve())
        report["passed"] = True
    except BaseException as exc:
        report.update(passed=False, error=str(exc))
        raise
    finally:
        if dataset is not None:
            dataset.stop_image_writer()
        report["bus_diagnostics"] = robot.arm.targets.diagnostics()
        robot.disconnect()
        report["interface_tx_packets_delta"] = tx_count(bus_config.can_name) - before_tx
        (run / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
