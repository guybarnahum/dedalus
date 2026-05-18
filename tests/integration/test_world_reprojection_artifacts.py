#!/usr/bin/env python3
"""Integration test for world-agent reprojection artifacts and MP4 export dry-run."""

from __future__ import annotations

import json
import math
import shutil
import subprocess
import sys
from pathlib import Path

AGENT_COLOR = (80, 255, 120)


def write_config(path: Path, output_dir: Path) -> None:
    path.write_text(
        "\n".join(
            [
                "frame_source: synthetic_mission",
                "ego_provider: frame_hint",
                "detector: scripted",
                "camera_stabilizer: null",
                "tracker: simple_centroid",
                "identity_resolver: appearance_only",
                "projector: flat_ground",
                "ghost_targets_enabled: true",
                "ghost_targets_scenario: person_pair_crossing",
                "world_model: in_memory",
                "fallback_map_frame_id: map_local_0001",
                "frame_annotator: ppm_sequence",
                f"annotation_output_path: {output_dir}",
                "annotation_output_fps: 5",
                "pipeline_timing_enabled: false",
                "mission_controller: disabled",
                "flight_command_sink: disabled",
                "",
            ]
        ),
        encoding="utf-8",
    )


def read_ppm(path: Path) -> tuple[int, int, bytes]:
    data = path.read_bytes()
    if not data.startswith(b"P6\n"):
        raise AssertionError(f"annotated frame is not P6 PPM: {path}")
    cursor = 3
    tokens: list[bytes] = []
    while len(tokens) < 3:
        while data[cursor:cursor + 1].isspace():
            cursor += 1
        start = cursor
        while cursor < len(data) and not data[cursor:cursor + 1].isspace():
            cursor += 1
        tokens.append(data[start:cursor])
    while data[cursor:cursor + 1].isspace():
        cursor += 1
    width = int(tokens[0])
    height = int(tokens[1])
    max_value = int(tokens[2])
    if max_value != 255:
        raise AssertionError(f"unsupported PPM max value {max_value}: {path}")
    pixels = data[cursor:]
    if len(pixels) != width * height * 3:
        raise AssertionError(f"unexpected PPM payload size for {path}")
    return width, height, pixels


def project_point(width: int, height: int, point: tuple[float, float, float]) -> tuple[int, int]:
    x, y, z = point
    fx = (width * 0.5) / math.tan(math.radians(90.0) * 0.5)
    fy = fx
    cx = (width - 1.0) * 0.5
    cy = (height - 1.0) * 0.5
    u = fx * (y / x) + cx
    v = fy * (z / x) + cy
    return round(u), round(v)


def count_color_near(
    width: int,
    height: int,
    pixels: bytes,
    center: tuple[int, int],
    radius: int,
    color: tuple[int, int, int],
) -> int:
    cx, cy = center
    count = 0
    for y in range(max(0, cy - radius), min(height, cy + radius + 1)):
        for x in range(max(0, cx - radius), min(width, cx + radius + 1)):
            offset = (y * width + x) * 3
            if tuple(pixels[offset:offset + 3]) == color:
                count += 1
    return count


def require_close(actual: float, expected: float, tolerance: float, message: str) -> None:
    if abs(actual - expected) > tolerance:
        raise AssertionError(f"{message}: actual={actual} expected={expected} tolerance={tolerance}")


def require_vec2_close(actual: list[float], expected: tuple[float, float], tolerance: float, message: str) -> None:
    if len(actual) != 2:
        raise AssertionError(f"{message}: expected 2-vector, got {actual}")
    for index, expected_value in enumerate(expected):
        require_close(float(actual[index]), expected_value, tolerance, f"{message}[{index}]")


def require_vec3_close(actual: list[float], expected: tuple[float, float, float], tolerance: float, message: str) -> None:
    if len(actual) != 3:
        raise AssertionError(f"{message}: expected 3-vector, got {actual}")
    for index, expected_value in enumerate(expected):
        require_close(float(actual[index]), expected_value, tolerance, f"{message}[{index}]")


def find_agent(sidecar: dict, source_track_id: str) -> dict:
    for agent in sidecar.get("agents", []):
        if agent.get("source_track_id") == source_track_id:
            return agent
    raise AssertionError(f"missing sidecar agent for source_track_id={source_track_id}")


def residual_px(source_center: tuple[float, float], reprojection: tuple[float, float]) -> float:
    return math.hypot(reprojection[0] - source_center[0], reprojection[1] - source_center[1])


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: test_world_reprojection_artifacts.py <repo-root> <build-dir>", file=sys.stderr)
        return 2

    repo_root = Path(sys.argv[1]).resolve()
    build_dir = Path(sys.argv[2]).resolve()
    output_root = build_dir / "world-reprojection-artifacts"
    annotation_dir = output_root / "annotations"
    config_path = output_root / "core_stack_world_reprojection_ci.yaml"
    output_mp4 = output_root / "world_reprojection.mp4"

    if output_root.exists():
        shutil.rmtree(output_root)
    output_root.mkdir(parents=True)
    write_config(config_path, annotation_dir)

    app = build_dir / "apps" / "dedalus_core_stack"
    if not app.exists():
        raise AssertionError(f"missing app binary: {app}")

    result = subprocess.run(
        [str(app), "--config", str(config_path), "--max-frames", "3"],
        cwd=repo_root,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )

    snapshot = json.loads(result.stdout)
    source_track_ids = {agent.get("source_track_id", "") for agent in snapshot.get("agents", [])}
    expected_tracks = {"track_0001", "ghost_person_001", "ghost_person_002", "ghost_car_001"}
    if not expected_tracks <= source_track_ids:
        raise AssertionError(f"snapshot missing deterministic world-agent tracks: got {sorted(source_track_ids)}")

    manifest = annotation_dir / "manifest.txt"
    if not manifest.exists():
        raise AssertionError(f"missing annotation manifest: {manifest}")
    rows = manifest.read_text(encoding="utf-8").strip().splitlines()
    if len(rows) != 4:
        raise AssertionError(f"expected header + 3 manifest rows, got {len(rows)}")

    first_frame = annotation_dir / "frame_000001.ppm"
    first_sidecar = annotation_dir / "frame_000001.world_overlay.json"
    for index in range(1, 4):
        frame = annotation_dir / f"frame_{index:06d}.ppm"
        sidecar = annotation_dir / f"frame_{index:06d}.world_overlay.json"
        if not frame.exists() or frame.stat().st_size == 0:
            raise AssertionError(f"missing non-empty annotated frame: {frame}")
        if frame.read_bytes()[:2] != b"P6":
            raise AssertionError(f"annotated frame is not P6 PPM: {frame}")
        if not sidecar.exists() or sidecar.stat().st_size == 0:
            raise AssertionError(f"missing non-empty world overlay sidecar: {sidecar}")

    width, height, pixels = read_ppm(first_frame)
    # synthetic_mission first frame timestamp is 0.1s and initial ego height is 0,
    # so ghost_person_001 is at x=12.03, y=-4.0, z=0.0 in ego/body convention.
    expected_point = (12.03, -4.0, 0.0)
    expected_marker = project_point(width, height, expected_point)
    marker_pixels = count_color_near(width, height, pixels, expected_marker, radius=8, color=AGENT_COLOR)
    if marker_pixels < 8:
        raise AssertionError(
            f"expected projected world-agent marker pixels near {expected_marker}, found {marker_pixels}"
        )

    sidecar = json.loads(first_sidecar.read_text(encoding="utf-8"))
    if sidecar.get("frame_index") != 1:
        raise AssertionError(f"unexpected sidecar frame_index: {sidecar.get('frame_index')}")
    if sidecar.get("coordinate_products", {}).get("viewport") != "camera pixel coordinates":
        raise AssertionError("sidecar missing viewport coordinate product description")
    camera_model = sidecar.get("camera_model", {})
    if camera_model.get("width") != width or camera_model.get("height") != height:
        raise AssertionError(f"sidecar camera dimensions do not match PPM: {camera_model}")

    camera_agent = find_agent(sidecar, "track_0001")
    if camera_agent.get("source_detection_id") != "det_0001":
        raise AssertionError(f"camera-derived agent missing source_detection_id: {camera_agent}")
    if camera_agent.get("source_frame_id") != "synthetic_mission_000001":
        raise AssertionError(f"camera-derived agent missing source_frame_id: {camera_agent}")
    bbox = camera_agent.get("source_bbox_px", {})
    if bbox != {"x": 260.0, "y": 160.0, "width": 80.0, "height": 180.0}:
        raise AssertionError(f"unexpected source bbox: {bbox}")
    source_center = (300.0, 250.0)
    reprojected_center = (float(camera_agent.get("u_px")), float(camera_agent.get("v_px")))
    require_vec2_close(camera_agent.get("source_center_px", []), source_center, 0.001, "source_center_px")
    require_vec2_close(camera_agent.get("reprojected_center_px", []), reprojected_center, 0.001, "reprojected_center_px")
    require_close(
        float(camera_agent.get("residual_px")),
        residual_px(source_center, reprojected_center),
        0.001,
        "residual_px",
    )

    ghost = find_agent(sidecar, "ghost_person_001")
    if ghost.get("visible") is not True:
        raise AssertionError(f"expected ghost_person_001 visible in sidecar: {ghost}")
    if ghost.get("reason") != "visible":
        raise AssertionError(f"expected ghost_person_001 reason=visible, got {ghost.get('reason')}")
    if "source_bbox_px" in ghost or "residual_px" in ghost:
        raise AssertionError(f"ghost agent should not have camera-derived residual fields: {ghost}")
    require_close(float(ghost.get("u_px")), float(expected_marker[0]), 1.0, "sidecar u_px")
    require_close(float(ghost.get("v_px")), float(expected_marker[1]), 1.0, "sidecar v_px")
    require_vec3_close(ghost.get("position_ego_relative", []), expected_point, 0.02, "position_ego_relative")
    require_close(float(ghost.get("depth_m")), expected_point[0], 0.02, "sidecar depth_m")

    subprocess.run(
        [
            sys.executable,
            "scripts/export-ppm-sequence-to-mp4.py",
            "--annotation-dir",
            str(annotation_dir),
            "--output-mp4",
            str(output_mp4),
            "--dry-run",
        ],
        cwd=repo_root,
        check=True,
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
