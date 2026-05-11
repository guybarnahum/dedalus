#pragma once

#include <string>

#include "dedalus/runtime/provider_registry.hpp"

namespace dedalus {

CoreStackProviderConfig load_core_stack_config(const std::string& path);

}  // namespace dedalus
