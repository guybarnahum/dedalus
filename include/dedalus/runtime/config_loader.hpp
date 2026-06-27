#pragma once

#include <string>
#include <vector>

#include "dedalus/runtime/core_stack_runner.hpp"
#include "dedalus/runtime/provider_registry.hpp"

namespace dedalus {

struct CoreStackConfig {
    CoreStackProviderConfig providers;
    CoreStackRunnerConfig runner;
};

// Load from a single config file.  Files may use the `include:` key to compose
// from base files; later keys always override earlier ones.
CoreStackConfig load_core_stack_app_config(const std::string& path);

// Load from multiple config files merged left-to-right (later files override).
// Equivalent to: for each path, apply all its keys (including any include: chain)
// on top of the accumulated config.
CoreStackConfig load_core_stack_app_config(const std::vector<std::string>& paths);

CoreStackProviderConfig load_core_stack_config(const std::string& path);
void validate_provider_names(const CoreStackProviderConfig& config, const ProviderRegistry& registry);

}  // namespace dedalus
