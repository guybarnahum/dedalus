#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>

#include "dedalus/simulation/ghost_scenario.hpp"

namespace {

struct Options {
    std::string scenario_path;
    double time_s{0.0};
};

void usage(const char* program) {
    std::cerr
        << "usage: " << program << " --scenario <path> --time-s <seconds>\n"
        << "\n"
        << "Evaluate a ghost detection scenario using the shared Dedalus GhostScenario evaluator.\n";
}

Options parse_args(int argc, char** argv) {
    Options options;
    for (int index = 1; index < argc; ++index) {
        const std::string arg = argv[index];
        if (arg == "--scenario" && index + 1 < argc) {
            options.scenario_path = argv[++index];
        } else if (arg == "--time-s" && index + 1 < argc) {
            options.time_s = std::stod(argv[++index]);
        } else if (arg == "--help" || arg == "-h") {
            usage(argv[0]);
            std::exit(0);
        } else {
            throw std::invalid_argument("unknown or incomplete argument: " + arg);
        }
    }
    if (options.scenario_path.empty()) {
        throw std::invalid_argument("--scenario is required");
    }
    if (options.time_s < 0.0) {
        throw std::invalid_argument("--time-s must be >= 0");
    }
    return options;
}

void print_vec3(const dedalus::Vec3& value) {
    std::cout << '[' << value.x << ',' << value.y << ',' << value.z << ']';
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const auto options = parse_args(argc, argv);
        const auto scenario = dedalus::GhostScenario::load_from_file(options.scenario_path);
        const auto states = scenario.evaluate(options.time_s);

        std::cout << "{\n";
        std::cout << "  \"schema_version\": 1,\n";
        std::cout << "  \"scenario\": \"" << scenario.name() << "\",\n";
        std::cout << "  \"map_frame_id\": \"" << scenario.map_frame_id().value << "\",\n";
        std::cout << "  \"time_s\": " << options.time_s << ",\n";
        std::cout << "  \"detections\": [\n";
        for (std::size_t index = 0; index < states.size(); ++index) {
            const auto& state = states[index];
            std::cout << "    {\n";
            std::cout << "      \"source_track_id\": \"" << state.source_track_id.value << "\",\n";
            std::cout << "      \"class\": \"" << dedalus::to_string(state.class_label) << "\",\n";
            std::cout << "      \"confidence\": " << state.confidence << ",\n";
            std::cout << "      \"position_local_m\": ";
            print_vec3(state.position_local_m);
            std::cout << ",\n";
            std::cout << "      \"velocity_local_mps\": ";
            print_vec3(state.velocity_local_mps);
            std::cout << ",\n";
            std::cout << "      \"size_m\": ";
            print_vec3(state.size_m);
            std::cout << "\n";
            std::cout << "    }" << (index + 1U == states.size() ? "" : ",") << "\n";
        }
        std::cout << "  ]\n";
        std::cout << "}\n";
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "dedalus_ghost_scenario_eval: " << exc.what() << '\n';
        usage(argv[0]);
        return 1;
    }
}
