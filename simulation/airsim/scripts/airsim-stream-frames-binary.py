#!/usr/bin/env python3
"""Stream AirSim scene frames using a compact binary protocol.

Frame protocol, little-endian (single version):

    magic[8]        = b'DEDFRM1\0'
    header_size     uint32  = 60
    version         uint32  = 3
    sequence        uint64
    timestamp_ns    int64
    width           uint32
    height          uint32
    channels        uint32  = 3
    pixel_format    uint32  = 1 (RGB8)
    payload_size    uint32
    ego_json_size   uint32  (0 when --include-ego not set)
    depth_size      uint32  (0 when --include-depth not set)

    payload         raw RGB bytes    (payload_size bytes)
    ego_json        UTF-8 JSON       (ego_json_size bytes)
    depth_binary    float32 LE       (depth_size bytes)

Binary depth:
  - Full-resolution AirSim DepthPlanar — no downsampling.
  - The C++ airsim_gt_vd provider receives full-res data and runs the same
    block-minimum grid as visual_onnx — both providers pipeline-identical.
  - Encoding: raw float32 little-endian, row-major.  0.0 = invalid/no-return.
  - Dimensions come from the ego_json sidecar: depth_width (int), depth_height (int).
"""

from __future__ import annotations

import argparse
import array
from contextlib import redirect_stdout
import json
import os
from pathlib import Path
import struct
import sys
import time
from typing import TextIO

import airsim

MAGIC = b"DEDFRM1\0"
VERSION = 3
HEADER_SIZE = 60
PIXEL_FORMAT_RGB8 = 1
MAVLINK_SAFETY_ARMED_FLAG = 128


def elapsed_ms(start_ns: int, end_ns: int) -> float:
    return (end_ns - start_ns) / 1_000_000.0


class TimingJsonlWriter:
    """Optional dependency-free bridge-internal timing writer."""

    def __init__(self, path: str | None):
        self._file: TextIO | None = None
        if path:
            output_path = Path(path)
            output_path.parent.mkdir(parents=True, exist_ok=True)
            self._file = output_path.open("w", encoding="utf-8")

    def close(self) -> None:
        if self._file is not None:
            self._file.close()
            self._file = None

    def write(self, record: dict[str, object]) -> None:
        if self._file is None:
            return
        self._file.write(json.dumps(record, separators=(",", ":")) + "\n")
        self._file.flush()


class MavlinkEgoTelemetryReader:
    """Best-effort non-blocking ego telemetry reader from MAVLink."""

    def __init__(self, endpoints: list[str]):
        self._connections: list[object] = []
        self._last_armed: bool | None = None
        self._last_local_position: tuple[float, float, float] | None = None
        self._last_local_velocity: tuple[float, float, float] | None = None
        self._mavutil = None

        if not endpoints:
            return

        try:
            from pymavlink import mavutil  # type: ignore
        except Exception as exc:  # pragma: no cover - depends on sim host deps
            print(
                f"airsim-stream-frames-binary: pymavlink unavailable; "
                f"MAVLink ego telemetry disabled: {exc}",
                file=sys.stderr,
            )
            return

        self._mavutil = mavutil
        for endpoint in endpoints:
            try:
                connection = mavutil.mavlink_connection(
                    endpoint,
                    autoreconnect=True,
                    source_system=255,
                )
                self._connections.append(connection)
                print(
                    f"airsim-stream-frames-binary: MAVLink ego telemetry listening on {endpoint}",
                    file=sys.stderr,
                )
            except Exception as exc:  # pragma: no cover - depends on live ports
                print(
                    f"airsim-stream-frames-binary: failed to open MAVLink endpoint "
                    f"{endpoint}: {exc}",
                    file=sys.stderr,
                )

    def sample(self) -> dict[str, object]:
        for connection in self._connections:
            while True:
                try:
                    msg = connection.recv_match(blocking=False)
                except Exception as exc:  # pragma: no cover - live transport only
                    print(
                        f"airsim-stream-frames-binary: MAVLink read failed: {exc}",
                        file=sys.stderr,
                    )
                    break
                if msg is None:
                    break

                msg_type = msg.get_type()
                if msg_type == "HEARTBEAT":
                    base_mode = int(getattr(msg, "base_mode", 0))
                    self._last_armed = bool(base_mode & MAVLINK_SAFETY_ARMED_FLAG)
                elif msg_type == "LOCAL_POSITION_NED":
                    self._last_local_position = (
                        float(getattr(msg, "x", 0.0)),
                        float(getattr(msg, "y", 0.0)),
                        float(getattr(msg, "z", 0.0)),
                    )
                    self._last_local_velocity = (
                        float(getattr(msg, "vx", 0.0)),
                        float(getattr(msg, "vy", 0.0)),
                        float(getattr(msg, "vz", 0.0)),
                    )

        payload: dict[str, object] = {}
        if self._last_armed is not None:
            payload["armed"] = self._last_armed
            payload["armed_valid"] = True
            payload["armed_source"] = "mavlink_heartbeat"
        if self._last_local_position is not None:
            payload["position"] = list(self._last_local_position)
            payload["position_valid"] = True
            payload["position_source"] = "mavlink_local_position_ned"
            payload["height_m"] = max(0.0, -self._last_local_position[2])
            payload["height_valid"] = True
            payload["landed_state"] = 2 if -self._last_local_position[2] > 0.25 else 1
        if self._last_local_velocity is not None:
            payload["velocity"] = list(self._last_local_velocity)
            payload["velocity_valid"] = True
            payload["velocity_source"] = "mavlink_local_position_ned"
        return payload


def parse_mavlink_endpoints(value: str) -> list[str]:
    return [endpoint.strip() for endpoint in value.split(",") if endpoint.strip()]


def rgb_bytes_from_response(response: object) -> bytes:
    width = int(response.width)
    height = int(response.height)
    raw = bytes(response.image_data_uint8)
    if width <= 0 or height <= 0:
        raise RuntimeError(f"invalid AirSim image dimensions: {width}x{height}")

    pixel_count = width * height
    if len(raw) == pixel_count * 3:
        # AirSim returns BGR (OpenCV convention) — swap R and B to get RGB.
        buf = bytearray(raw)
        for i in range(0, len(buf), 3):
            buf[i], buf[i + 2] = buf[i + 2], buf[i]
        return bytes(buf)
    if len(raw) == pixel_count * 4:
        # BGRA → RGB
        rgb_buffer = bytearray()
        for i in range(0, len(raw), 4):
            rgb_buffer += bytes([raw[i + 2], raw[i + 1], raw[i]])
        return bytes(rgb_buffer)

    raise RuntimeError(
        f"unexpected AirSim image byte count: got {len(raw)}, "
        f"expected {pixel_count * 3} or {pixel_count * 4}"
    )


def raw_depth_info(response: object) -> tuple[int, int, bytes]:
    """Return AirSim DepthPlanar as (width, height, float32_le_bytes).

    Sends the full-resolution depth buffer as raw float32 bytes — no downsampling.
    The C++ airsim_gt_vd detector receives full-res data and runs the same
    block-minimum grid as visual_onnx, making both providers pipeline-identical.

    Encoding uses array.array('f') for fast bulk conversion (~5 ms for 640×360)
    vs json.dumps which would take ~150 ms for the same count.

    Returns (0, 0, b"") on failure (missing/undersized AirSim response).
    """
    width = int(getattr(response, "width", 0))
    height = int(getattr(response, "height", 0))
    raw = getattr(response, "image_data_float", None) or []
    n = width * height
    if width <= 0 or height <= 0 or len(raw) < n:
        return 0, 0, b""
    buf = array.array("f", raw[:n])
    if sys.byteorder != "little":
        buf.byteswap()
    return width, height, buf.tobytes()


def ego_json_bytes(
    client: airsim.MultirotorClient,
    vehicle_name: str,
    timestamp_ns: int,
    mavlink_ego_reader: MavlinkEgoTelemetryReader | None = None,
    depth_dims: tuple[int, int] | None = None,
) -> bytes:
    """Build the ego sidecar JSON.

    depth_dims: (width, height) when a binary depth chunk follows this sidecar
    in the v3 frame; None when no binary depth is attached.  Only the dimensions
    are included in JSON — the float data itself is in the binary chunk.
    """
    state = client.getMultirotorState(vehicle_name=vehicle_name)
    kin = state.kinematics_estimated

    position = kin.position
    orientation = airsim.to_eularian_angles(kin.orientation)
    velocity = kin.linear_velocity
    angular_velocity = kin.angular_velocity

    landed_state = int(getattr(state, "landed_state", 0))
    armed_valid = hasattr(state, "armed")
    armed = bool(getattr(state, "armed", False)) if armed_valid else False
    armed_source = "airsim" if armed_valid else "none"

    payload = {
        "timestamp_ns": int(timestamp_ns),
        "position": [float(position.x_val), float(position.y_val), float(position.z_val)],
        "position_valid": False,
        "position_source": "airsim_multirotor_state",
        "height_m": max(0.0, -float(position.z_val)),
        "height_valid": False,
        "rotation_rpy": [float(orientation[0]), float(orientation[1]), float(orientation[2])],
        "velocity": [float(velocity.x_val), float(velocity.y_val), float(velocity.z_val)],
        "velocity_valid": False,
        "velocity_source": "airsim_multirotor_state",
        "angular_velocity": [
            float(angular_velocity.x_val),
            float(angular_velocity.y_val),
            float(angular_velocity.z_val),
        ],
        "landed_state": landed_state,
        "armed": armed,
        "armed_valid": armed_valid,
        "armed_source": armed_source,
    }

    if mavlink_ego_reader is not None:
        payload.update(mavlink_ego_reader.sample())

    if depth_dims is not None:
        # Dimensions only — the float32 data is in the binary depth chunk.
        payload["depth_width"] = depth_dims[0]
        payload["depth_height"] = depth_dims[1]

    return json.dumps(payload, separators=(",", ":"), allow_nan=False).encode("utf-8")


def write_frame(
    sequence: int,
    timestamp_ns: int,
    width: int,
    height: int,
    payload: bytes,
    ego_payload: bytes = b"",
    depth_bytes: bytes = b"",
) -> bool:
    """Write one binary frame to stdout (always protocol version 3)."""
    header = struct.pack(
        "<8sIIQqIIIIIII",
        MAGIC, HEADER_SIZE, VERSION,
        sequence, timestamp_ns, width, height,
        3, PIXEL_FORMAT_RGB8,
        len(payload), len(ego_payload), len(depth_bytes),
    )
    try:
        sys.stdout.buffer.write(header)
        sys.stdout.buffer.write(payload)
        if ego_payload:
            sys.stdout.buffer.write(ego_payload)
        if depth_bytes:
            sys.stdout.buffer.write(depth_bytes)
        sys.stdout.buffer.flush()
    except BrokenPipeError:
        return False
    return True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--rpc-port", type=int, default=41451)
    parser.add_argument("--vehicle-name", default="PX4")
    parser.add_argument("--camera-name", default="front_center")
    parser.add_argument("--count", type=int, default=0, help="0 means stream forever")
    parser.add_argument("--rate-hz", type=float, default=5.0)
    parser.add_argument("--include-ego", action="store_true",
                        help="Append ego telemetry JSON after each RGB payload (v2/v3).")
    parser.add_argument("--mavlink-armed-endpoints",
                        default=os.environ.get("DEDALUS_MAVLINK_ARMED_ENDPOINTS", ""),
                        help="Deprecated alias for --mavlink-ego-endpoints.")
    parser.add_argument("--mavlink-ego-endpoints",
                        default=os.environ.get("DEDALUS_MAVLINK_EGO_ENDPOINTS", ""),
                        help="Comma-separated pymavlink endpoints for ego telemetry.")
    parser.add_argument("--timing-jsonl", default="",
                        help="Optional path for bridge-internal timing JSONL records.")
    parser.add_argument("--include-depth", action="store_true",
                        help="Append full-resolution AirSim DepthPlanar as binary float32 "
                             "(protocol v3).  The C++ detector runs block-minimum sampling, "
                             "matching visual_onnx exactly.  Requires --include-ego.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    timing = TimingJsonlWriter(args.timing_jsonl or None)
    endpoint_string = args.mavlink_ego_endpoints or args.mavlink_armed_endpoints
    mavlink_ego_reader = MavlinkEgoTelemetryReader(parse_mavlink_endpoints(endpoint_string))
    try:
        client = airsim.MultirotorClient(ip=args.host, port=args.rpc_port)
        with redirect_stdout(sys.stderr):
            client.confirmConnection()

        period_s = 0.0 if args.rate_hz <= 0 else 1.0 / args.rate_hz
        next_deadline = time.perf_counter()
        sequence = 0
        while args.count == 0 or sequence < args.count:
            loop_start_ns = time.perf_counter_ns()

            image_start_ns = time.perf_counter_ns()
            requests = [airsim.ImageRequest(args.camera_name, airsim.ImageType.Scene, False, False)]
            if args.include_depth:
                requests.append(airsim.ImageRequest(args.camera_name, airsim.ImageType.DepthPlanar, True, False))
            responses = client.simGetImages(requests, vehicle_name=args.vehicle_name)
            image_end_ns = time.perf_counter_ns()
            if not responses:
                raise RuntimeError("AirSim returned no image responses")

            response = responses[0]
            sequence += 1
            timestamp_ns = int(getattr(response, "time_stamp", 0) or time.time_ns())

            rgb_start_ns = time.perf_counter_ns()
            payload = rgb_bytes_from_response(response)
            rgb_end_ns = time.perf_counter_ns()

            depth_start_ns = time.perf_counter_ns()
            depth_w, depth_h, depth_bytes = (
                raw_depth_info(responses[1])
                if args.include_depth and len(responses) > 1
                else (0, 0, b"")
            )
            depth_end_ns = time.perf_counter_ns()

            ego_start_ns = time.perf_counter_ns()
            ego_payload = (
                ego_json_bytes(
                    client, args.vehicle_name, timestamp_ns, mavlink_ego_reader,
                    (depth_w, depth_h) if depth_bytes else None,
                )
                if args.include_ego
                else b""
            )
            ego_end_ns = time.perf_counter_ns()

            write_start_ns = time.perf_counter_ns()
            write_ok = write_frame(
                sequence, timestamp_ns,
                int(response.width), int(response.height),
                payload, ego_payload, depth_bytes,
            )
            write_end_ns = time.perf_counter_ns()
            if not write_ok:
                return 0

            sleep_ms = 0.0
            target_period_ms = period_s * 1000.0
            behind_ms = 0.0
            if period_s > 0 and (args.count == 0 or sequence < args.count):
                next_deadline += period_s
                sleep_start_ns = time.perf_counter_ns()
                sleep_s = next_deadline - time.perf_counter()
                if sleep_s > 0.0:
                    time.sleep(sleep_s)
                else:
                    behind_ms = -sleep_s * 1000.0
                    next_deadline = time.perf_counter()
                sleep_end_ns = time.perf_counter_ns()
                sleep_ms = elapsed_ms(sleep_start_ns, sleep_end_ns)

            loop_end_ns = time.perf_counter_ns()
            timing.write(
                {
                    "sequence": sequence,
                    "timestamp_ns": timestamp_ns,
                    "width": int(response.width),
                    "height": int(response.height),
                    "payload_bytes": len(payload),
                    "ego_bytes": len(ego_payload),
                    "depth_width": depth_w,
                    "depth_height": depth_h,
                    "depth_bytes": len(depth_bytes),
                    "include_ego": bool(args.include_ego),
                    "include_depth": bool(args.include_depth),
                    "target_period_ms": target_period_ms,
                    "sim_get_images_ms": elapsed_ms(image_start_ns, image_end_ns),
                    "rgb_convert_ms": elapsed_ms(rgb_start_ns, rgb_end_ns),
                    "depth_encode_ms": elapsed_ms(depth_start_ns, depth_end_ns),
                    "ego_sample_ms": elapsed_ms(ego_start_ns, ego_end_ns),
                    "stdout_write_ms": elapsed_ms(write_start_ns, write_end_ns),
                    "sleep_ms": sleep_ms,
                    "producer_behind_ms": behind_ms,
                    "producer_compute_ms": elapsed_ms(loop_start_ns, write_end_ns),
                    "total_loop_ms": elapsed_ms(loop_start_ns, loop_end_ns),
                }
            )

        return 0
    finally:
        timing.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    finally:
        try:
            sys.stdout.close()
        except BrokenPipeError:
            pass
