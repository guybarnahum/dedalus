#!/usr/bin/env bash
set -u

BUILD_DIR="${BUILD_DIR:-build-validation}"
CONFIG_PATH="${DEDALUS_CORE_STACK_CONFIG:-config/core_stack_ci.yaml}"
SNAPSHOT_PATH="${SNAPSHOT_PATH:-/tmp/world_snapshot.json}"
SUMMARY_TITLE="${SUMMARY_TITLE:-CI Validation Summary}"
BUILD_RUN_LABEL="${BUILD_RUN_LABEL:-Build & run}"

APP_PATH="./${BUILD_DIR}/apps/dedalus_core_stack"

run_status=0
json_status=0
missing_fields=0

mkdir -p "$(dirname "$SNAPSHOT_PATH")"

if ! "${APP_PATH}" --config "${CONFIG_PATH}" > "${SNAPSHOT_PATH}"; then
    run_status=$?
fi

if [[ -f "${SNAPSHOT_PATH}" ]]; then
    if ! python3 -m json.tool "${SNAPSHOT_PATH}" >/dev/null; then
        json_status=1
    fi
else
    json_status=1
fi

check_required_field() {
    local label="$1"
    local pattern="$2"
    if [[ ! -f "${SNAPSHOT_PATH}" ]] || ! grep -Fq "${pattern}" "${SNAPSHOT_PATH}"; then
        missing_fields=1
        return 1
    fi
    return 0
}

check_required_field "active_map_frame_id" '"active_map_frame_id": "map_local_0001"'
check_required_field "person agent" '"class": "person"'
check_required_field "car container" '"type": "car"'
check_required_field "map_frame_id in map_frames" '"map_frame_id": "map_local_0001"'
check_required_field "tactical_exclusion_zones" '"tactical_exclusion_zones"'
check_required_field "tactical exclusion zone" '"reason": "dynamic_observation_cone"'
check_required_field "uncertain regions" '"uncertain_regions"'
check_required_field "flight corridors" '"flight_corridors"'
check_required_field "rough flight corridor" '"corridor_id": "corridor_forward_0001"'
check_required_field "static_structures" '"static_structures"'
check_required_field "static structure" '"structure_id": "structure_building_0001"'
check_required_field "landmarks" '"landmarks"'
check_required_field "landmark" '"landmark_id": "landmark_building_corner_0001"'

write_summary_row() {
    local label="$1"
    local pattern="$2"
    if [[ -f "${SNAPSHOT_PATH}" ]] && grep -Fq "${pattern}" "${SNAPSHOT_PATH}"; then
        echo "| ${label} | ✅ present |" >> "${GITHUB_STEP_SUMMARY}"
    else
        echo "| ${label} | ❌ missing |" >> "${GITHUB_STEP_SUMMARY}"
    fi
}

if [[ -n "${GITHUB_STEP_SUMMARY:-}" ]]; then
    {
        echo "## ${SUMMARY_TITLE}"
        echo ""
        echo "| Check | Result |"
        echo "|---|---|"
    } >> "${GITHUB_STEP_SUMMARY}"

    if [[ -f "${SNAPSHOT_PATH}" && ${run_status} -eq 0 ]]; then
        echo "| ${BUILD_RUN_LABEL} | ✅ passed |" >> "${GITHUB_STEP_SUMMARY}"
    else
        echo "| ${BUILD_RUN_LABEL} | ❌ failed — snapshot not produced |" >> "${GITHUB_STEP_SUMMARY}"
    fi

    if [[ ${json_status} -eq 0 ]]; then
        echo "| JSON valid | ✅ passed |" >> "${GITHUB_STEP_SUMMARY}"
    else
        echo "| JSON valid | ❌ failed |" >> "${GITHUB_STEP_SUMMARY}"
    fi

    write_summary_row "active_map_frame_id" '"active_map_frame_id": "map_local_0001"'
    write_summary_row "person agent" '"class": "person"'
    write_summary_row "car container" '"type": "car"'
    write_summary_row "map_frame_id in map_frames" '"map_frame_id": "map_local_0001"'
    write_summary_row "tactical exclusion zone" '"reason": "dynamic_observation_cone"'
    write_summary_row "uncertain regions" '"uncertain_regions"'
    write_summary_row "rough flight corridor" '"corridor_id": "corridor_forward_0001"'
    write_summary_row "static structure" '"structure_id": "structure_building_0001"'
    write_summary_row "landmark" '"landmark_id": "landmark_building_corner_0001"'

    if [[ -f "${SNAPSHOT_PATH}" ]]; then
        {
            echo ""
            echo "### WorldSnapshot output"
            echo '```json'
            cat "${SNAPSHOT_PATH}"
            echo '```'
        } >> "${GITHUB_STEP_SUMMARY}"
    fi
fi

if [[ ${run_status} -ne 0 || ${json_status} -ne 0 || ${missing_fields} -ne 0 ]]; then
    exit 1
fi
