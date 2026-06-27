#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

#include "dedalus/runtime/config_loader.hpp"
#include "dedalus/runtime/core_stack_runner.hpp"
#include "dedalus/runtime/provider_registry.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

namespace {

struct CliOptions {
    std::string config_path{"config/ci/core_stack_ci.yaml"};
    int max_frames{1};
};

CliOptions options_from_args(int argc, char** argv) {
    CliOptions options;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--config") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--config requires a path");
            }
            options.config_path = argv[++i];
        } else if (arg == "--max-frames") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--max-frames requires a value");
            }
            options.max_frames = std::stoi(argv[++i]);
            if (options.max_frames <= 0) {
                throw std::invalid_argument("--max-frames must be positive");
            }
        } else if (arg == "-h" || arg == "--help") {
            std::cout << "usage: dedalus_core_stack [--config PATH] [--max-frames N]\n";
            std::exit(0);
        } else {
            throw std::invalid_argument("unknown argument: " + arg);
        }
    }

    return options;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const auto options = options_from_args(argc, argv);
        auto app_config = dedalus::load_core_stack_app_config(options.config_path);
        const auto& config = app_config.providers;

        dedalus::ProviderRegistry registry;
        dedalus::CoreStackRunner runner{registry.create(config), std::move(app_config.runner)};
        int frames_processed = 0;
        for (; frames_processed < options.max_frames; ++frames_processed) {
            if (!runner.run_once()) {
                break;
            }
        }

        if (frames_processed == 0) {
            return 1;
        }

        std::cout << dedalus::to_json(runner.snapshot());
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "dedalus_core_stack: " << ex.what() << "\n";
        return 2;
    }
}
