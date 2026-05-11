#include <exception>
#include <iostream>
#include <string>

#include "dedalus/runtime/config_loader.hpp"
#include "dedalus/runtime/core_stack_runner.hpp"
#include "dedalus/runtime/provider_registry.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

namespace {

std::string config_path_from_args(int argc, char** argv) {
    std::string config_path = "config/core_stack_ci.yaml";

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--config") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--config requires a path");
            }
            config_path = argv[++i];
        } else {
            throw std::invalid_argument("unknown argument: " + arg);
        }
    }

    return config_path;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const auto config_path = config_path_from_args(argc, argv);
        const auto config = dedalus::load_core_stack_config(config_path);

        dedalus::ProviderRegistry registry;
        dedalus::CoreStackRunner runner{registry.create(config)};
        if (!runner.run_once()) {
            return 1;
        }

        std::cout << dedalus::to_json(runner.snapshot());
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "dedalus_core_stack: " << ex.what() << "\n";
        return 2;
    }
}
