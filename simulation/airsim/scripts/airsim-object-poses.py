#!/usr/bin/env python3
"""Return AirSim scene object poses as compact JSON.

This bridge utility is used by the dependency-free C++ core to read selected
AirSim object poses without linking against the AirSim Python package directly.
It supports exact object names and Unreal/AirSim object-name patterns.

Default mode is one-shot: write one compact JSON response to stdout and exit.
Persistent mode is enabled with --stream-jsonl: keep one AirSim client alive and
write one compact JSON response per line at the requested stream rate.

It writes machine-readable JSON to stdout and human diagnostics to stderr.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import sys
import time
from typing import Any

try:
    import airsim
except ImportError as exc:
    raise SystemExit("airsim Python package is required: pip install airsim") from exc


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1", help="AirSim RPC host")
    parser.add_argument("--rpc-port", type=int, default=41451, help="AirSim RPC port")
    parser.add_argument("--object", action="append", default=[], dest="objects", help="Exact object name to query. May be repeated.")
    parser.add_argument("--dynamic-object", action="append", default=[], dest="dynamic_objects", help="Exact object name to refresh every stream frame. Other objects may use static refresh throttling.")
    parser.add_argument("--pattern", action="append", default=[], dest="patterns", help="AirSim/Unreal object-name pattern such as Tree* or Cable*. May be repeated.")
    parser.add_argument("--timestamp-ns", type=int, default=0, help="Optional caller timestamp to echo in the one-shot response.")
    parser.add_argument("--fail-on-missing", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--max-pattern-matches", type=int, default=256)
    parser.add_argument("--stream-jsonl", action="store_true", help="Persistent mode: emit one compact JSON pose frame per line until interrupted")
    parser.add_argument("--stream-rate-hz", type=float, default=30.0, help="Persistent stream output rate. Default: 30 Hz")
    parser.add_argument("--static-refresh-every-frames", type=int, default=10, help="Refresh non-dynamic objects every N stream frames. Cached poses are emitted between refreshes. Use 1 to refresh all objects every frame.")
    return parser.parse_args()


def pose_to_json(name: str, pose: Any, *, source_pattern: str | None = None) -> dict[str, Any]:
    position = pose.position
    orientation = pose.orientation
    payload: dict[str, Any] = {
        "name": name,
        "pose_available": True,
        "position_ned_m": [float(position.x_val), float(position.y_val), float(position.z_val)],
        "orientation_quat_xyzw": [
            float(orientation.x_val),
            float(orientation.y_val),
            float(orientation.z_val),
            float(orientation.w_val),
        ],
    }
    if source_pattern is not None:
        payload["source_pattern"] = source_pattern
    return payload


def unique_preserving_order(values: list[str]) -> list[str]:
    seen: set[str] = set()
    out: list[str] = []
    for value in values:
        if value not in seen:
            seen.add(value)
            out.append(value)
    return out


def list_pattern_matches(client: Any, pattern: str, max_matches: int) -> list[str]:
    names = client.simListSceneObjects(pattern)
    names = unique_preserving_order([str(name) for name in names])
    names.sort()
    if max_matches > 0:
        return names[:max_matches]
    return names


def build_query_names(client: Any, objects: list[str], patterns: list[str], max_pattern_matches: int) -> list[tuple[str, str | None]]:
    query_names: list[tuple[str, str | None]] = [(name, None) for name in objects]
    for pattern in patterns:
        for name in list_pattern_matches(client, pattern, max_pattern_matches):
            query_names.append((name, pattern))

    deduped: list[tuple[str, str | None]] = []
    seen: set[str] = set()
    for name, pattern in query_names:
        if name in seen:
            continue
        seen.add(name)
        deduped.append((name, pattern))
    return deduped


def read_object_pose(client: Any, name: str, pattern: str | None, *, fail_on_missing: bool) -> dict[str, Any]:
    try:
        pose = client.simGetObjectPose(name)
        return pose_to_json(name, pose, source_pattern=pattern)
    except Exception as exc:
        if fail_on_missing:
            raise
        payload = {"name": name, "pose_available": False, "error": str(exc)}
        if pattern is not None:
            payload["source_pattern"] = pattern
        return payload


def read_object_poses(
    client: Any,
    query_names: list[tuple[str, str | None]],
    *,
    timestamp_ns: int,
    fail_on_missing: bool,
) -> dict[str, Any]:
    objects: list[dict[str, Any]] = []
    errors: list[str] = []
    for name, pattern in query_names:
        try:
            objects.append(read_object_pose(client, name, pattern, fail_on_missing=fail_on_missing))
        except Exception as exc:
            error = f"{name}: {exc}"
            errors.append(error)
            payload = {"name": name, "pose_available": False, "error": str(exc)}
            if pattern is not None:
                payload["source_pattern"] = pattern
            objects.append(payload)

    if errors and fail_on_missing:
        raise RuntimeError("failed to read AirSim object pose(s): " + "; ".join(errors))

    return {
        "schema_version": 1,
        "source": "airsim_object_poses",
        "timestamp_ns": timestamp_ns if timestamp_ns != 0 else time.time_ns(),
        "objects": objects,
    }


def emit_payload(payload: dict[str, Any]) -> None:
    try:
        print(json.dumps(payload, separators=(",", ":")), flush=True)
    except BrokenPipeError:
        raise SystemExit(0)  # consumer closed the pipe — exit silently


def run_one_shot(args: argparse.Namespace, client: Any) -> int:
    if not args.objects and not args.patterns:
        raise ValueError("at least one --object or --pattern is required")
    query_names = build_query_names(client, args.objects, args.patterns, args.max_pattern_matches)
    emit_payload(
        read_object_poses(
            client,
            query_names,
            timestamp_ns=args.timestamp_ns,
            fail_on_missing=args.fail_on_missing,
        )
    )
    return 0


def run_stream(args: argparse.Namespace, client: Any) -> int:
    if args.patterns:
        raise ValueError("--stream-jsonl supports exact --object names only; expand patterns before starting the stream")
    if not args.objects:
        raise ValueError("--stream-jsonl requires at least one --object")
    if args.stream_rate_hz <= 0.0:
        raise ValueError("--stream-rate-hz must be positive")
    if args.static_refresh_every_frames <= 0:
        raise ValueError("--static-refresh-every-frames must be positive")

    query_names = build_query_names(client, args.objects, [], args.max_pattern_matches)
    query_by_name = {name: pattern for name, pattern in query_names}
    dynamic_names = set(args.dynamic_objects)
    cached: dict[str, dict[str, Any]] = {}
    frame_index = 0
    # Minimum 1 Hz to guard against zero/negative config values.
    # Dynamic objects (--dynamic-object) are refreshed every frame; static objects
    # are refreshed every --static-refresh-every-frames frames.  At 1 Hz with
    # static_refresh_every_n_frames=600 this gives: dynamic=1 s, static=600 s.
    effective_rate_hz = max(args.stream_rate_hz, 1.0)
    period_s = 1.0 / effective_rate_hz
    next_deadline = time.monotonic()
    while True:
        next_deadline += period_s
        frame_index += 1
        output_objects: list[dict[str, Any]] = []
        refresh_static = frame_index == 1 or args.static_refresh_every_frames == 1 or (frame_index % args.static_refresh_every_frames) == 0
        errors: list[str] = []
        for name, pattern in query_names:
            should_refresh = name in dynamic_names or refresh_static or name not in cached
            if should_refresh:
                try:
                    cached[name] = read_object_pose(client, name, pattern, fail_on_missing=args.fail_on_missing)
                except Exception as exc:
                    error = f"{name}: {exc}"
                    errors.append(error)
                    cached[name] = {"name": name, "pose_available": False, "error": str(exc)}
            output_objects.append(cached[name])
        if errors and args.fail_on_missing:
            raise RuntimeError("failed to read AirSim object pose(s): " + "; ".join(errors))
        emit_payload(
            {
                "schema_version": 1,
                "source": "airsim_object_poses",
                "timestamp_ns": time.time_ns(),
                "objects": output_objects,
            }
        )
        sleep_s = next_deadline - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            # If AirSim calls overrun the requested period, skip accumulated
            # deadlines instead of trying to catch up with stale burst output.
            next_deadline = time.monotonic()


def main() -> int:
    args = parse_args()
    client = airsim.MultirotorClient(ip=args.host, port=args.rpc_port)
    # AirSim's confirmConnection() prints a human banner to stdout. The C++
    # bridge consumes stdout as machine JSON, and persistent mode reads one line
    # at a time, so redirect the banner to stderr to keep stdout JSON-only.
    with contextlib.redirect_stdout(sys.stderr):
        client.confirmConnection()
    if args.stream_jsonl:
        return run_stream(args, client)
    return run_one_shot(args, client)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("airsim-object-poses: interrupted", file=sys.stderr)
        raise SystemExit(130)
    except Exception as exc:
        print(f"airsim-object-poses: {exc}", file=sys.stderr)
        raise SystemExit(1)
