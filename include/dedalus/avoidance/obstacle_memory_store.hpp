#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace dedalus {

// Stable storage-neutral key for a quantized obstacle-memory cell.
struct ObstacleMemoryCellKey {
    std::string site_id;
    std::int32_t ix{0};
    std::int32_t iy{0};
    std::int32_t iz{0};
};

struct ObstacleMemoryPoint3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

enum class ObstacleMemoryCellStatus {
    Active,
    Probationary,
    Stale,
    Suppressed,
    Retired,
};

struct ObstacleMemoryCell {
    ObstacleMemoryCellKey key;
    ObstacleMemoryPoint3 center_site;

    double occupied_log_odds{0.0};
    double free_log_odds{0.0};

    std::uint32_t positive_observation_count{0};
    std::uint32_t free_observation_count{0};

    std::uint64_t first_seen_unix_ns{0};
    std::uint64_t last_seen_unix_ns{0};
    std::uint64_t last_confirmed_occupied_unix_ns{0};
    std::uint64_t last_observed_free_unix_ns{0};

    std::string last_source_kind;
    std::string last_source_provider;
};

struct ObstacleMemoryDerivedScore {
    ObstacleMemoryCellKey key;

    double occupancy_score{0.0};
    double freshness_score{0.0};
    double active_score{0.0};
    double cell_age_seconds{0.0};
    double site_staleness_seconds{0.0};
    double relative_gap_seconds{0.0};

    ObstacleMemoryCellStatus status{ObstacleMemoryCellStatus::Probationary};
    std::uint64_t scored_at_unix_ns{0};
};

struct ObstacleMemoryCellDelta {
    ObstacleMemoryCellKey key;
    ObstacleMemoryPoint3 center_site;

    double occupied_delta{0.0};
    double free_delta{0.0};
    double confidence{0.0};

    std::uint64_t timestamp_unix_ns{0};

    std::string mission_id;
    std::string source_kind;
    std::string source_provider;
};

struct ObstacleMemoryRegionQuery {
    std::string site_id;

    std::int32_t min_ix{0};
    std::int32_t max_ix{0};
    std::int32_t min_iy{0};
    std::int32_t max_iy{0};
    std::int32_t min_iz{0};
    std::int32_t max_iz{0};

    std::vector<ObstacleMemoryCellStatus> statuses;

    std::size_t limit{0};  // 0 means no explicit limit.
};

struct ObstacleMemoryLoadResult {
    std::vector<ObstacleMemoryCell> cells;
    std::vector<ObstacleMemoryDerivedScore> scores;
};

// Tier B: mission delta stream sink.
//
// Implementations may write JSONL, SQLite, LMDB, or binary logs. Callers must not
// depend on the on-disk format.
class ObstacleDeltaSink {
public:
    virtual ~ObstacleDeltaSink() = default;

    virtual void append_batch(const std::vector<ObstacleMemoryCellDelta>& deltas) = 0;
    virtual void flush() = 0;
};

// Tier C: persistent site-memory store.
//
// Implementations may be JSON debug storage, SQLite, LMDB, custom binary/mmap,
// or a remote/service-backed store. Runtime users should depend only on this
// query/merge contract.
class PersistentObstacleStore {
public:
    virtual ~PersistentObstacleStore() = default;

    virtual void merge_delta_batch(const std::vector<ObstacleMemoryCellDelta>& deltas) = 0;

    virtual ObstacleMemoryLoadResult load_region(const ObstacleMemoryRegionQuery& query) const = 0;

    virtual void recompute_scores(std::uint64_t now_unix_ns) = 0;

    virtual void flush() = 0;
};

// Debug/export codec.
//
// This is intentionally outside the hot runtime map path. It converts efficient
// stores/logs into human-readable artifacts and imports debug artifacts when
// useful for tests or migration.
class ObstacleMemoryDebugCodec {
public:
    virtual ~ObstacleMemoryDebugCodec() = default;

    virtual void export_debug_json(const std::string& output_path) const = 0;
    virtual void import_debug_json(const std::string& input_path) = 0;
};

}  // namespace dedalus
