#pragma once

#include "dedalus/runtime/provider_registry.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {

class CoreStackRunner {
public:
    explicit CoreStackRunner(CoreStackProviders providers);

    [[nodiscard]] bool run_once();
    [[nodiscard]] WorldSnapshot snapshot() const;

private:
    CoreStackProviders providers_;
};

}  // namespace dedalus
