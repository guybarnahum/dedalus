#!/usr/bin/env python3
"""Measure L0/L1 obstacle-map correctness against AirSim scene-inventory ground truth.

L1 (MissionLocalTraversabilityMap) is meant to fuse multi-viewpoint evidence to
reduce noise on top of L0 (MissionLocalObstacleMap)'s simple additive
accumulator. This tool makes that claim checkable: it voxelizes real object
positions/sizes from an AirSim scene-inventory JSON into a ground-truth
occupied-voxel set, then computes precision/recall/F1 for L0's and L1's own
occupied cells against it from the same mission run. The L1-minus-L0 delta is
the log-odds fusion layer's *measured* contribution — not an assumption.

Inputs, all written by a normal `run_mission.sh` invocation (no extra flags):
  L0: <run_dir>/mission_obstacle_map_deltas.sqlite  (table `cells`)
  L1: <run_dir>/mission_traversability_map_full.json
  ground truth: an AirSim scene-inventory *.objects.json (--scene-inventory)

Frame note: scene-inventory positions (`position_ned_m`) come from AirSim's
own simGetObjectPose RPC (AirSim world/NED origin). L0/L1 cell positions
(`center_map`/`center_x/y/z`) come from the mission's local frame, whose
origin is PX4's own LOCAL_POSITION_NED estimate when MAVLink ego is active
(see airsim-stream-frames-binary.py's ego_json_bytes) -- a different origin
than AirSim's world frame, not guaranteed to coincide. This tool measures that
offset empirically (find_anchor_offset) using a static object it can actually
see in the map, rather than assuming the two frames share an origin.
"""

from __future__ import annotations

import argparse
import json
import math
import sqlite3
import sys
import time
from pathlib import Path
from typing import Any


def progress(message: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {message}", file=sys.stderr, flush=True)

DEFAULT_CELL_SIZE_M = 0.5
DEFAULT_OCCUPIED_THRESHOLD = 1.0
DEFAULT_MAX_RANGE_M = 25.0
DEFAULT_ANCHOR_SEARCH_RADIUS_M = 8.0
MAX_PLAUSIBLE_ANCHOR_OFFSET_M = 50.0
DEFAULT_REGRESSION_TOLERANCE_PPT = 5.0
# Ground-truth candidates in these classes are excluded from both anchor
# selection and the ground-truth voxel set: their scene-inventory position is
# a one-shot pose fix taken before the mission runs, but circle_person.yaml
# binds this class to a live, moving ghost target (dynamic_refresh_every_n_frames
# vs. a ~600s static refresh for everything else) -- the inventory position is
# stale by the time the mission actually observes it.
DEFAULT_EXCLUDE_CLASSES = ("person",)


def is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def is_vec3_array(value: Any) -> bool:
    return isinstance(value, list) and len(value) == 3 and all(is_number(v) for v in value)


def load_scene_inventory(path: Path, exclude_classes: set[str]) -> list[dict[str, Any]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    objects = data.get("objects")
    if not isinstance(objects, list):
        raise ValueError(f"scene inventory {path} missing objects list")
    candidates: list[dict[str, Any]] = []
    for obj in objects:
        if not isinstance(obj, dict) or obj.get("pose_available") is not True:
            continue
        position = obj.get("position_ned_m")
        size = obj.get("recommended_size_m")
        if not is_vec3_array(position) or not is_vec3_array(size):
            continue
        canonical_class = str(obj.get("canonical_class", ""))
        if canonical_class in exclude_classes:
            continue
        candidates.append({
            "name": str(obj.get("name", "")),
            "canonical_class": canonical_class,
            "thin_obstacle": bool(obj.get("thin_obstacle", False)),
            "position_ned_m": tuple(float(v) for v in position),
            "recommended_size_m": tuple(float(v) for v in size),
        })
    return candidates


def snapshot_paths(run_dir: Path) -> list[Path]:
    manifest = run_dir / "snapshot_manifest.txt"
    if manifest.exists():
        paths: list[Path] = []
        for raw in manifest.read_text(encoding="utf-8").splitlines():
            stripped = raw.strip()
            if not stripped or stripped.startswith("#"):
                continue
            parts = stripped.split()
            name = parts[1] if len(parts) >= 2 and parts[0].isdigit() else parts[0]
            path = Path(name)
            paths.append(path if path.is_absolute() else run_dir / path)
        return paths
    return sorted(run_dir.glob("snapshot_*.json"))


def load_ego_trajectory(run_dir: Path) -> list[tuple[float, float, float]]:
    trajectory: list[tuple[float, float, float]] = []
    for path in snapshot_paths(run_dir):
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        ego = data.get("ego")
        if not isinstance(ego, dict):
            continue
        position = ego.get("position_local")
        if is_vec3_array(position):
            trajectory.append(tuple(float(v) for v in position))
    return trajectory


def load_l0_cells(db_path: Path, occupied_threshold: float) -> list[tuple[float, float, float]]:
    """Returns occupied cell centers, deduped to each cell's most recently observed score.

    `cells` is an append-only delta log (one row per historical batch touch,
    not one row per current cell), so the same (x,y,z) can appear many times
    across the mission -- keep only the row with the latest last_seen_unix_ns.
    """
    conn = sqlite3.connect(str(db_path))
    try:
        rows = conn.execute(
            "SELECT center_x, center_y, center_z, occupied_score, last_seen_unix_ns FROM cells"
        ).fetchall()
    finally:
        conn.close()

    latest: dict[tuple[float, float, float], tuple[float, int]] = {}
    for x, y, z, occupied_score, last_seen in rows:
        key = (round(x, 6), round(y, 6), round(z, 6))
        prior = latest.get(key)
        if prior is None or last_seen >= prior[1]:
            latest[key] = (occupied_score, last_seen)

    return [xyz for xyz, (score, _) in latest.items() if score >= occupied_threshold]


def load_l1_cells(json_path: Path) -> tuple[list[tuple[float, float, float]], float, float, bool]:
    """Returns (occupied cell centers, cell_size_m, vertical_cell_size_m, is_capped)."""
    data = json.loads(json_path.read_text(encoding="utf-8"))
    config = data.get("config", {})
    cell_size_m = float(config.get("cell_size_m", DEFAULT_CELL_SIZE_M))
    vertical_cell_size_m = float(config.get("vertical_cell_size_m", cell_size_m))
    capped = bool(data.get("export_summary", {}).get("source_cells_are_debug_capped", False))

    occupied: list[tuple[float, float, float]] = []
    for cell in data.get("cells", []):
        if cell.get("state") != "occupied":
            continue
        center = cell.get("center_map", {})
        occupied.append((float(center["x"]), float(center["y"]), float(center["z"])))
    return occupied, cell_size_m, vertical_cell_size_m, capped


def voxel_index(point: tuple[float, float, float], cell_size_m: float, vertical_cell_size_m: float) -> tuple[int, int, int]:
    x, y, z = point
    return (
        math.floor(x / cell_size_m),
        math.floor(y / cell_size_m),
        math.floor(z / vertical_cell_size_m),
    )


def voxelize_box(
    center: tuple[float, float, float],
    size_m: tuple[float, float, float],
    cell_size_m: float,
    vertical_cell_size_m: float,
) -> set[tuple[int, int, int]]:
    cx, cy, cz = center
    sx, sy, sz = size_m
    ix_min, iy_min, iz_min = voxel_index((cx - sx / 2.0, cy - sy / 2.0, cz - sz / 2.0), cell_size_m, vertical_cell_size_m)
    ix_max, iy_max, iz_max = voxel_index((cx + sx / 2.0, cy + sy / 2.0, cz + sz / 2.0), cell_size_m, vertical_cell_size_m)
    return {
        (ix, iy, iz)
        for ix in range(ix_min, ix_max + 1)
        for iy in range(iy_min, iy_max + 1)
        for iz in range(iz_min, iz_max + 1)
    }


def distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def voxelize_objects(
    objects: list[dict[str, Any]],
    offset: tuple[float, float, float],
    cell_size_m: float,
    vertical_cell_size_m: float,
) -> set[tuple[int, int, int]]:
    voxels: set[tuple[int, int, int]] = set()
    for obj in objects:
        corrected_position = tuple(p + o for p, o in zip(obj["position_ned_m"], offset))
        voxels |= voxelize_box(corrected_position, obj["recommended_size_m"], cell_size_m, vertical_cell_size_m)
    return voxels


def build_cell_buckets(
    cells: list[tuple[float, float, float]],
    bucket_size_m: float,
) -> dict[tuple[int, int, int], list[tuple[float, float, float]]]:
    buckets: dict[tuple[int, int, int], list[tuple[float, float, float]]] = {}
    for cell in cells:
        key = tuple(math.floor(c / bucket_size_m) for c in cell)
        buckets.setdefault(key, []).append(cell)
    return buckets


def nearby_cells(
    buckets: dict[tuple[int, int, int], list[tuple[float, float, float]]],
    bucket_size_m: float,
    position: tuple[float, float, float],
    radius_m: float,
) -> list[tuple[float, float, float]]:
    """Cells within radius_m of position, found via a 3x3x3 (or larger, if
    radius_m > bucket_size_m) bucket neighborhood instead of scanning every
    cell -- turns the per-candidate lookup from O(all cells) into O(cells
    actually near this one candidate), the same bucket-index technique used
    for MissionLocalPlanningMap's box queries in the C++ side of this map."""
    bx, by, bz = (math.floor(p / bucket_size_m) for p in position)
    span = math.ceil(radius_m / bucket_size_m)
    candidates: list[tuple[float, float, float]] = []
    for dx in range(-span, span + 1):
        for dy in range(-span, span + 1):
            for dz in range(-span, span + 1):
                bucket = buckets.get((bx + dx, by + dy, bz + dz))
                if bucket:
                    candidates.extend(bucket)
    return [c for c in candidates if distance(c, position) <= radius_m]


def find_anchor_offset(
    candidates: list[dict[str, Any]],
    real_cells: list[tuple[float, float, float]],
    search_radius_m: float,
    forced_anchor_name: str | None,
) -> tuple[tuple[float, float, float], dict[str, Any]] | None:
    """Empirically measure the translation between scene-inventory NED positions
    and the mission-local frame, using whichever static ground-truth object's
    raw position best explains a nearby cluster of real occupied cells.

    Returns ((dx, dy, dz), diagnostics) to add to every scene-inventory
    position, or None if no candidate has any real cells nearby at all --
    which means either the frames disagree by more than a simple translation,
    or there just isn't enough map data yet to check.
    """
    pool = candidates
    if forced_anchor_name is not None:
        pool = [c for c in candidates if c["name"] == forced_anchor_name]
        if not pool:
            return None

    buckets = build_cell_buckets(real_cells, search_radius_m)

    best: dict[str, Any] | None = None
    for obj in pool:
        raw_position = obj["position_ned_m"]
        nearby = nearby_cells(buckets, search_radius_m, raw_position, search_radius_m)
        if not nearby:
            continue
        centroid = tuple(sum(c[i] for c in nearby) / len(nearby) for i in range(3))
        offset = tuple(centroid[i] - raw_position[i] for i in range(3))
        if best is None or len(nearby) > best["nearby_count"]:
            best = {
                "name": obj["name"],
                "canonical_class": obj["canonical_class"],
                "nearby_count": len(nearby),
                "offset": offset,
                "offset_magnitude_m": distance(offset, (0.0, 0.0, 0.0)),
            }

    if best is None:
        return None
    return best["offset"], best


def compute_metrics(
    real_occupied: list[tuple[float, float, float]],
    ground_truth_voxels: set[tuple[int, int, int]],
    cell_size_m: float,
    vertical_cell_size_m: float,
) -> dict[str, Any]:
    real_voxels = {voxel_index(c, cell_size_m, vertical_cell_size_m) for c in real_occupied}
    true_positive = len(real_voxels & ground_truth_voxels)
    false_positive = len(real_voxels - ground_truth_voxels)
    false_negative = len(ground_truth_voxels - real_voxels)
    precision = true_positive / (true_positive + false_positive) if (true_positive + false_positive) else 0.0
    recall = true_positive / (true_positive + false_negative) if (true_positive + false_negative) else 0.0
    f1 = 2 * precision * recall / (precision + recall) if (precision + recall) else 0.0
    return {
        "occupied_cell_count": len(real_voxels),
        "true_positive": true_positive,
        "false_positive": false_positive,
        "false_negative": false_negative,
        "precision": precision,
        "recall": recall,
        "f1": f1,
    }


def print_report(l0: dict[str, Any], l1: dict[str, Any], l0_ground_truth_voxel_count: int, l1_ground_truth_voxel_count: int) -> None:
    if l0_ground_truth_voxel_count == l1_ground_truth_voxel_count:
        print(f"Ground-truth occupied voxels: {l0_ground_truth_voxel_count}")
    else:
        print(f"Ground-truth occupied voxels: L0 grid={l0_ground_truth_voxel_count}  L1 grid={l1_ground_truth_voxel_count}")
    print(f"{'metric':<12}{'L0':>10}{'L1':>10}{'L1 - L0':>10}")
    for key in ("occupied_cell_count", "true_positive", "false_positive", "false_negative"):
        print(f"{key:<12}{l0[key]:>10}{l1[key]:>10}{l1[key] - l0[key]:>10}")
    for key in ("precision", "recall", "f1"):
        delta = l1[key] - l0[key]
        print(f"{key:<12}{l0[key]:>10.3f}{l1[key]:>10.3f}{delta:>+10.3f}")


def print_class_breakdown(
    in_range: list[dict[str, Any]],
    offset: tuple[float, float, float],
    l0_occupied: list[tuple[float, float, float]],
    l1_occupied: list[tuple[float, float, float]],
    l0_cell_size_m: float,
    l0_vertical_cell_size_m: float,
    l1_cell_size_m: float,
    l1_vertical_cell_size_m: float,
) -> None:
    """Per-class recall, grouped by thin_obstacle -- answers "does L1 lose thin
    structures (cable/pole/tree, per the scene-inventory's own geometry_class
    taxonomy) more than solid ones (wall/fence/rock/...)?" directly, rather
    than burying it in one aggregate number."""
    by_class: dict[str, list[dict[str, Any]]] = {}
    for obj in in_range:
        by_class.setdefault(obj["canonical_class"], []).append(obj)

    print()
    print("Per-class recall (thin structures grouped first):")
    print(f"{'class':<18}{'thin':<6}{'objects':>8}{'gt_vox':>8}{'L0 recall':>11}{'L1 recall':>11}{'delta':>9}")

    rows: list[tuple[bool, str, dict[str, Any]]] = []
    for class_name, objs in by_class.items():
        thin = objs[0]["thin_obstacle"]
        l0_voxels = voxelize_objects(objs, offset, l0_cell_size_m, l0_vertical_cell_size_m)
        l1_voxels = voxelize_objects(objs, offset, l1_cell_size_m, l1_vertical_cell_size_m)
        l0_m = compute_metrics(l0_occupied, l0_voxels, l0_cell_size_m, l0_vertical_cell_size_m)
        l1_m = compute_metrics(l1_occupied, l1_voxels, l1_cell_size_m, l1_vertical_cell_size_m)
        rows.append((thin, class_name, {
            "objects": len(objs), "gt_voxels": len(l0_voxels),
            "l0_recall": l0_m["recall"], "l1_recall": l1_m["recall"],
        }))

    for thin, class_name, row in sorted(rows, key=lambda r: (not r[0], r[1])):
        delta = row["l1_recall"] - row["l0_recall"]
        print(f"{class_name:<18}{'yes' if thin else 'no':<6}{row['objects']:>8}{row['gt_voxels']:>8}"
              f"{row['l0_recall']:>11.3f}{row['l1_recall']:>11.3f}{delta:>+9.3f}")

    thin_objects = [obj for obj in in_range if obj["thin_obstacle"]]
    solid_objects = [obj for obj in in_range if not obj["thin_obstacle"]]
    print()
    print(f"{'group':<18}{'':<6}{'objects':>8}{'gt_vox':>8}{'L0 recall':>11}{'L1 recall':>11}{'delta':>9}")
    for label, objs in (("thin (cable/pole/tree)", thin_objects), ("solid (wall/fence/...)", solid_objects)):
        if not objs:
            continue
        l0_voxels = voxelize_objects(objs, offset, l0_cell_size_m, l0_vertical_cell_size_m)
        l1_voxels = voxelize_objects(objs, offset, l1_cell_size_m, l1_vertical_cell_size_m)
        l0_m = compute_metrics(l0_occupied, l0_voxels, l0_cell_size_m, l0_vertical_cell_size_m)
        l1_m = compute_metrics(l1_occupied, l1_voxels, l1_cell_size_m, l1_vertical_cell_size_m)
        delta = l1_m["recall"] - l0_m["recall"]
        print(f"{label:<18}{'':<6}{len(objs):>8}{len(l0_voxels):>8}"
              f"{l0_m['recall']:>11.3f}{l1_m['recall']:>11.3f}{delta:>+9.3f}")


def check_regression(baseline: dict[str, Any], current: dict[str, Any], tolerance_ppt: float) -> list[str]:
    failures: list[str] = []
    for layer in ("l0", "l1"):
        for metric in ("precision", "recall", "f1"):
            before = baseline.get(layer, {}).get(metric)
            after = current.get(layer, {}).get(metric)
            if not is_number(before) or not is_number(after):
                continue
            drop_ppt = (before - after) * 100.0
            if drop_ppt > tolerance_ppt:
                failures.append(
                    f"{layer}.{metric} regressed: baseline={before:.3f} current={after:.3f} "
                    f"(dropped {drop_ppt:.1f}pp, tolerance {tolerance_ppt:.1f}pp)"
                )
    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--scene-inventory", type=Path, required=True)
    parser.add_argument("--l0-sqlite", type=Path, default=None, help="default: <run_dir>/mission_obstacle_map_deltas.sqlite")
    parser.add_argument("--l1-json", type=Path, default=None, help="default: <run_dir>/mission_traversability_map_full.json")
    parser.add_argument("--cell-size-m", type=float, default=DEFAULT_CELL_SIZE_M, help="ground-truth voxelization resolution; L0/L1 grid resolution is read from L1's own artifact for the actual comparison")
    parser.add_argument("--occupied-threshold", type=float, default=DEFAULT_OCCUPIED_THRESHOLD)
    parser.add_argument("--max-range-m", type=float, default=DEFAULT_MAX_RANGE_M, help="exclude ground-truth objects the drone never flew within this range of")
    parser.add_argument("--anchor-search-radius-m", type=float, default=DEFAULT_ANCHOR_SEARCH_RADIUS_M)
    parser.add_argument("--anchor-object", default=None, help="force a specific scene-inventory object name as the frame-offset anchor instead of auto-selecting")
    parser.add_argument("--exclude-class", action="append", default=list(DEFAULT_EXCLUDE_CLASSES))
    parser.add_argument("--save-baseline", type=Path, default=None)
    parser.add_argument("--baseline", type=Path, default=None)
    parser.add_argument("--regression-tolerance-ppt", type=float, default=DEFAULT_REGRESSION_TOLERANCE_PPT)
    args = parser.parse_args()

    l0_sqlite = args.l0_sqlite or (args.run_dir / "mission_obstacle_map_deltas.sqlite")
    l1_json = args.l1_json or (args.run_dir / "mission_traversability_map_full.json")
    for path, label in ((args.scene_inventory, "scene inventory"), (l0_sqlite, "L0 sqlite"), (l1_json, "L1 json")):
        if not path.exists():
            print(f"ERROR: {label} not found: {path}", file=sys.stderr)
            return 2

    exclude_classes = set(args.exclude_class)
    progress(f"loading scene inventory: {args.scene_inventory}")
    candidates = load_scene_inventory(args.scene_inventory, exclude_classes)
    if not candidates:
        print(f"ERROR: no usable ground-truth objects in {args.scene_inventory} "
              f"(after excluding classes {sorted(exclude_classes)})", file=sys.stderr)
        return 2
    progress(f"scene inventory: {len(candidates)} candidate object(s) after excluding {sorted(exclude_classes)}")

    progress(f"loading L0 cells: {l0_sqlite}")
    l0_occupied = load_l0_cells(l0_sqlite, args.occupied_threshold)
    progress(f"L0: {len(l0_occupied)} occupied cell(s)")

    progress(f"loading L1 cells: {l1_json}")
    l1_occupied, l1_cell_size_m, l1_vertical_cell_size_m, l1_capped = load_l1_cells(l1_json)
    progress(f"L1: {len(l1_occupied)} occupied cell(s)")
    if l1_capped:
        print("WARNING: L1 artifact is debug-capped (export_summary.source_cells_are_debug_capped=true); "
              "metrics below undercount L1's true occupied set.", file=sys.stderr)

    progress(f"running frame sanity check against {len(candidates)} candidate(s) "
             f"x {len(l0_occupied) + len(l1_occupied)} cell(s) (bucketed, not a full scan)")
    anchor_result = find_anchor_offset(candidates, l0_occupied + l1_occupied, args.anchor_search_radius_m, args.anchor_object)
    if anchor_result is None:
        print("ERROR: frame sanity check failed -- no scene-inventory object has any real occupied "
              "cell within --anchor-search-radius-m of its raw NED position. Cannot establish the "
              "offset between the scene-inventory frame and the mission-local frame; refusing to "
              "report precision/recall built on an unverified frame assumption.", file=sys.stderr)
        return 2
    offset, anchor_info = anchor_result
    if anchor_info["offset_magnitude_m"] > MAX_PLAUSIBLE_ANCHOR_OFFSET_M:
        print(f"ERROR: frame sanity check found an implausible offset ({anchor_info['offset_magnitude_m']:.1f}m) "
              f"via anchor object {anchor_info['name']!r} -- likely a real frame/axis mismatch, not a "
              f"normal origin difference. Refusing to report metrics.", file=sys.stderr)
        return 2
    print(f"Frame sanity check: anchor={anchor_info['name']!r} ({anchor_info['canonical_class']}) "
          f"nearby_cells={anchor_info['nearby_count']} offset={tuple(round(v, 2) for v in offset)}m "
          f"magnitude={anchor_info['offset_magnitude_m']:.2f}m")

    progress(f"loading ego trajectory: {args.run_dir}")
    ego_trajectory = load_ego_trajectory(args.run_dir)
    progress(f"ego trajectory: {len(ego_trajectory)} sample(s); filtering candidates by --max-range-m {args.max_range_m}")
    in_range = [
        obj for obj in candidates
        if any(distance(tuple(p + o for p, o in zip(obj["position_ned_m"], offset)), ego) <= args.max_range_m
               for ego in ego_trajectory)
    ] if ego_trajectory else candidates
    if not in_range:
        print("ERROR: no ground-truth objects fell within --max-range-m of the recorded ego trajectory", file=sys.stderr)
        return 2
    progress(f"{len(in_range)}/{len(candidates)} candidate(s) within range; voxelizing ground truth")

    # Voxelize ground truth once per layer's own grid resolution -- L1's cell
    # size is read from its own artifact and isn't guaranteed to match
    # --cell-size-m, so reusing one voxel set for both comparisons would
    # silently misalign the grids if they ever differ.
    l0_ground_truth_voxels = voxelize_objects(in_range, offset, args.cell_size_m, args.cell_size_m)
    l1_ground_truth_voxels = voxelize_objects(in_range, offset, l1_cell_size_m, l1_vertical_cell_size_m)

    progress(f"ground truth: {len(l0_ground_truth_voxels)} voxel(s); computing L0/L1 metrics")
    l0_metrics = compute_metrics(l0_occupied, l0_ground_truth_voxels, args.cell_size_m, args.cell_size_m)
    l1_metrics = compute_metrics(l1_occupied, l1_ground_truth_voxels, l1_cell_size_m, l1_vertical_cell_size_m)
    print_report(l0_metrics, l1_metrics, len(l0_ground_truth_voxels), len(l1_ground_truth_voxels))
    print_class_breakdown(
        in_range, offset, l0_occupied, l1_occupied,
        args.cell_size_m, args.cell_size_m, l1_cell_size_m, l1_vertical_cell_size_m,
    )

    current = {"l0": l0_metrics, "l1": l1_metrics, "ground_truth_voxel_count": len(l0_ground_truth_voxels)}

    exit_code = 0
    if args.baseline is not None:
        if not args.baseline.exists():
            print(f"ERROR: --baseline path does not exist: {args.baseline}", file=sys.stderr)
            return 2
        baseline = json.loads(args.baseline.read_text(encoding="utf-8"))
        failures = check_regression(baseline, current, args.regression_tolerance_ppt)
        if failures:
            print(f"\nREGRESSION DETECTED ({len(failures)}):")
            for failure in failures:
                print(f"  - {failure}")
            exit_code = 1
        else:
            print("\nNo regression vs. baseline.")

    if args.save_baseline is not None:
        args.save_baseline.parent.mkdir(parents=True, exist_ok=True)
        args.save_baseline.write_text(json.dumps(current, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(f"\nSaved baseline: {args.save_baseline}")

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
