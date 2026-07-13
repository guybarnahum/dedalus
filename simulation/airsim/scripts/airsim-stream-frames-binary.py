#!/usr/bin/env python3
"""Stream AirSim scene frames using a compact binary protocol.

Frame protocol, little-endian:

    magic[8]        = b'DEDFRM1\0'
    header_size     uint32, currently 56
    version         uint32, 1 for RGB-only, 2 for RGB + ego JSON
    sequence        uint64
    timestamp_ns    int64
    width           uint32
    height          uint32
    channels        uint32, currently 3
    pixel_format    uint32, currently 1 for RGB8
    payload_size    uint32
    reserved        uint32, version 2 uses this as ego_json_size
    payload         raw RGB bytes
    ego_json        optional UTF-8 JSON bytes when version == 2

This avoids JSON/base64/PPM overhead while keeping AirSim dependencies in the
Python bridge process instead of the C++ core stack.
"""

from __future__ import annotations

import argparse
from contextlib import redirect_stdout
import json
import math
import os
from pathlib import Path
import struct
import sys
import time
from typing import TextIO

import airsim

MAGIC = b"DEDFRM1\0"
VERSION_RGB_ONLY = 1
VERSION_RGB_EGO = 2
HEADER_SIZE = 56
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


def sample_depth_grid(response: object, cols: int, rows: int) -> dict[str, object]:
    """Sample AirSim DepthPlanar into a cols×rows block-minimum grid.

    Each cell (gc, gr) covers pixels x∈[gc*BW, (gc+1)*BW) × y∈[gr*BH, (gr+1)*BH).
    The minimum positive finite depth (= closest obstacle) is selected per cell,
    matching the block-minimum logic in the C++ project_depth_to_device_evidence()
    kernel.  depth_width==cols and depth_height==rows in the output, so the C++
    detector receives an N×M frame that aligns exactly with its configured grid.
    """
    width = int(getattr(response, "width", 0))
    height = int(getattr(response, "height", 0))
    raw = list(getattr(response, "image_data_float", []) or [])
    if width <= 0 or height <= 0 or len(raw) < width * height:
        return {
            "depth_width": 0,
            "depth_height": 0,
            "depth_m": [],
            "depth_valid_count": 0,
            "depth_min_m": 0.0,
            "depth_max_m": 0.0,
        }

    BW = width // cols
    BH = height // rows
    if BW <= 0 or BH <= 0:
        # Source resolution is smaller than the requested grid — configuration mismatch.
        # Return empty so the C++ detector logs a warning rather than crashing.
        return {
            "depth_width": 0,
            "depth_height": 0,
            "depth_m": [],
            "depth_valid_count": 0,
            "depth_min_m": 0.0,
            "depth_max_m": 0.0,
        }

    sampled: list[float] = []
    for gr in range(rows):
        y0 = gr * BH
        for gc in range(cols):
            x0 = gc * BW
            # Block minimum: closest positive finite depth in the cell.
            # 0.0 represents invalid / no-return (same as C++ sentinel).
            best = 0.0
            for y in range(y0, y0 + BH):
                for x in range(x0, x0 + BW):
                    v = float(raw[y * width + x])
                    if math.isfinite(v) and 0.0 < v < 60000.0:
                        if best == 0.0 or v < best:
                            best = v
            sampled.append(best)

    valid = [v for v in sampled if v > 0.0]
    usable_0_5m = [v for v in valid if v <= 5.0]
    usable_5_20m = [v for v in valid if 5.0 < v <= 20.0]
    usable_20_80m = [v for v in valid if 20.0 < v <= 80.0]
    far_over_80m = [v for v in valid if v > 80.0]
    return {
        "depth_width": cols,
        "depth_height": rows,
        "depth_m": sampled,
        "depth_valid_count": len(valid),
        "depth_0_5m_count": len(usable_0_5m),
        "depth_5_20m_count": len(usable_5_20m),
        "depth_20_80m_count": len(usable_20_80m),
        "depth_over_80m_count": len(far_over_80m),
        "depth_min_m": min(valid) if valid else 0.0,
        "depth_max_m": max(valid) if valid else 0.0,
    }


def ego_json_bytes(    client: airsim.MultirotorClient,
    vehicle_name: str,
    timestamp_ns: int,
    mavlink_ego_reader: MavlinkEgoTelemetryReader | None = None,
    depth_payload: dict[str, object] | None = None,
) -> bytes:
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

    if depth_payload:
        payload.update(depth_payload)

    return json.dumps(payload, separators=(",", ":"), allow_nan=False).encode("utf-8")


def write_frame(
    sequence: int,
    timestamp_ns: int,
    width: int,
    height: int,
    payload: bytes,
    ego_payload: bytes = b"",
) -> bool:
    version = VERSION_RGB_EGO if ego_payload else VERSION_RGB_ONLY
    header = struct.pack(
        "<8sIIQqIIIIII",
        MAGIC,
        HEADER_SIZE,
        version,
        sequence,
        timestamp_ns,
        width,
        height,
        3,
        PIXEL_FORMAT_RGB8,
        len(payload),
        len(ego_payload),
    )
    try:
        sys.stdout.buffer.write(header)
        sys.stdout.buffer.write(payload)
        if ego_payload:
            sys.stdout.buffer.write(ego_payload)
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
    parser.add_argument("--include-ego", action="store_true", help="Append ego telemetry JSON after each RGB payload using binary protocol version 2.")
    parser.add_argument("--mavlink-armed-endpoints", default=os.environ.get("DEDALUS_MAVLINK_ARMED_ENDPOINTS", ""), help="Deprecated alias for --mavlink-ego-endpoints.")
    parser.add_argument("--mavlink-ego-endpoints", default=os.environ.get("DEDALUS_MAVLINK_EGO_ENDPOINTS", ""), help="Comma-separated pymavlink endpoints used to derive ego telemetry from HEARTBEAT and LOCAL_POSITION_NED.")
    parser.add_argument("--timing-jsonl", default="", help="Optional path for bridge-internal timing JSONL records.")
    parser.add_argument("--include-depth", action="store_true", help="Append an N×M block-minimum AirSim DepthPlanar grid to the sidecar JSON.")
    parser.add_argument("--depth-grid-cols", type=int, default=40, help="Grid columns for --include-depth block-minimum sampling. Must match visual_onnx.depth_grid_cols in the C++ pipeline config.")
    parser.add_argument("--depth-grid-rows", type=int, default=22, help="Grid rows for --include-depth block-minimum sampling. Must match visual_onnx.depth_grid_rows in the C++ pipeline config.")
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
            responses = client.simGetImages(
                requests,
                vehicle_name=args.vehicle_name,
            )
            image_end_ns = time.perf_counter_ns()
            if not responses:
                raise RuntimeError("AirSim returned no image responses")

            response = responses[0]
            sequence += 1
            timestamp_ns = int(getattr(response, "time_stamp", 0) or time.time_ns())

            rgb_start_ns = time.perf_counter_ns()
            payload = rgb_bytes_from_response(response)
            rgb_end_ns = time.perf_counter_ns()

            response_index = 1
            depth_payload = (
                sample_depth_grid(responses[response_index], args.depth_grid_cols, args.depth_grid_rows)
                if args.include_depth and len(responses) > response_index
                else None
            )

            ego_start_ns = time.perf_counter_ns()
            ego_payload = (
                ego_json_bytes(client, args.vehicle_name, timestamp_ns, mavlink_ego_reader, depth_payload)
                if args.include_ego
                else b""
            )
            ego_end_ns = time.perf_counter_ns()

            write_start_ns = time.perf_counter_ns()
            write_ok = write_frame(sequence, timestamp_ns, int(response.width), int(response.height), payload, ego_payload)
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
                    "include_ego": bool(args.include_ego),
                    "include_depth": bool(args.include_depth),
                    "depth_grid_cols": int(args.depth_grid_cols),
                    "depth_grid_rows": int(args.depth_grid_rows),
                    "depth_width": int(depth_payload.get("depth_width", 0)) if depth_payload else 0,
                    "depth_height": int(depth_payload.get("depth_height", 0)) if depth_payload else 0,
                    "depth_valid_count": int(depth_payload.get("depth_valid_count", 0)) if depth_payload else 0,
                    "depth_0_5m_count": int(depth_payload.get("depth_0_5m_count", 0)) if depth_payload else 0,
                    "depth_5_20m_count": int(depth_payload.get("depth_5_20m_count", 0)) if depth_payload else 0,
                    "depth_20_80m_count": int(depth_payload.get("depth_20_80m_count", 0)) if depth_payload else 0,
                    "depth_over_80m_count": int(depth_payload.get("depth_over_80m_count", 0)) if depth_payload else 0,
                    "depth_min_m": float(depth_payload.get("depth_min_m", 0.0)) if depth_payload else 0.0,
                    "depth_max_m": float(depth_payload.get("depth_max_m", 0.0)) if depth_payload else 0.0,
                    "target_period_ms": target_period_ms,
                    "sim_get_images_ms": elapsed_ms(image_start_ns, image_end_ns),
                    "rgb_convert_ms": elapsed_ms(rgb_start_ns, rgb_end_ns),
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
