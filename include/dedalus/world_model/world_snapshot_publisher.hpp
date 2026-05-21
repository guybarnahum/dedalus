#pragma once

#include "dedalus/runtime/pubsub.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {

using WorldSnapshotSubscriber = Subscriber<WorldSnapshot>;
using WorldSnapshotPublisher = Publisher<WorldSnapshot>;

}  // namespace dedalus
