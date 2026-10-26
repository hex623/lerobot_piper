"""Query firmware and limits; preserve ambiguous replies, never send motion."""
import json
from pathlib import Path

from lerobot.common.robot_devices.motors.piper_diagnostics import query_device


def main():
    report = query_device()
    profile_path = Path("piper_scripts/local-hardware.json")
    if profile_path.exists() and json.loads(profile_path.read_text(encoding="utf-8")).get("shared_can_teaching"):
        report["topology"] = "native_master_slave_shared_can"
        report["interpretation"] = "Broadcast queries can receive both arms' parameter values. Passive teaching uses separate command/feedback IDs."
    output = Path("outputs/device-query.json")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
