"""Non-motion Piper queries, retaining duplicate and conflicting CAN replies."""
import re
import time

import can


def query_device(channel="can0"):
    def request(bus, frame_id, data, response_id, duration):
        bus.send(can.Message(arbitration_id=frame_id, data=data, is_extended_id=False))
        replies = []
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            msg = bus.recv(timeout=0.02)
            if msg is not None and not msg.is_error_frame and msg.arbitration_id == response_id and len(msg.data) == 8:
                replies.append(bytes(msg.data))
        return replies

    with can.Bus(channel=channel, interface="socketcan", can_filters=[
        {"can_id": i, "can_mask": 0x7FF, "extended": False} for i in (0x4AF, 0x473)
    ]) as bus:
        firmware_frames = request(bus, 0x4AF, [1, 0, 0, 0, 0, 0, 0, 0], 0x4AF, 0.5)
        # Keep original raw frames as evidence. Deduplication only helps read
        # the observed version string; it cannot identify controller identity.
        chunks = b"".join(dict.fromkeys(firmware_frames))
        versions = sorted({m.decode("ascii") for m in re.findall(rb"S-V\d\.\d-\d", chunks)})
        limits = []
        for motor in range(1, 7):
            frames = request(bus, 0x472, [motor, 1, 0, 0, 0, 0, 0, 0], 0x473, 0.15)
            frames = [data for data in frames if data[0] == motor]
            unique = sorted(set(frames))
            limits.append({"motor": motor, "reply_count": len(frames), "variants": [
                {"min_deg": int.from_bytes(data[3:5], "big", signed=True) / 10,
                 "max_deg": int.from_bytes(data[1:3], "big", signed=True) / 10,
                 "max_speed_rad_s": int.from_bytes(data[5:7], "big") / 1000,
                 "raw_hex": data.hex()} for data in unique]})
    return {"can_name": channel, "query_only": True,
            "firmware_candidates": versions, "firmware_raw_frames": [f.hex() for f in firmware_frames],
            "joint_limits": limits,
            "single_reply_per_motor": all(m["reply_count"] == 1 for m in limits),
            "conflicting_limit_replies": any(len(m["variants"]) > 1 for m in limits)}
