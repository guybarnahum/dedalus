#pragma once

#include <string>

#include "dedalus/runtime/core_stack_runner.hpp"
#include "dedalus/runtime/provider_registry.hpp"

namespace dedalus {

struct CoreStackConfig {
    CoreStackProviderConfig providers;
    CoreStackRunnerConfig runner;
};

CoreStackConfig load_core_stack_app_config(const std::string& path);
CoreStackProviderConfig load_core_stack_config(const std::string& path);
void validate_provider_names(const CoreStackProviderConfig& config, const ProviderRegistry& registry);

}  // namespace dedalus
