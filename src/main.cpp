/// \author Jian Wen
/// \date 2021/01/29

#include "simulator/router.hpp"
#include "simulator/demand_generator.hpp"
#include "simulator/platform.hpp"

#include <iostream>
#include <fmt/format.h>
#undef NDEBUG
#include <assert.h>

int main(int argc, const char *argv[]) {
    auto s_time_ms = getTimeStampMs();

    // Get the root directory.
    const int MAXPATH = 250;
    char buffer[MAXPATH];
    getcwd(buffer, MAXPATH);
    std::string build_file_directory = buffer;
    auto root_directory = build_file_directory.substr(0, build_file_directory.find("AMoD2") + 5);

    // Check the input arugment list.
    std::string path_to_config_file;
    if (argc == 1) {
        path_to_config_file = root_directory + "/config/platform_demo.yml";
    } else if (argc == 2) {
        CheckFileExistence((std::string &) argv[1]);
        path_to_config_file = argv[1];
    } else {
        fmt::print(stderr,
                   "[ERROR] \n"
                   "- Usage: <prog name> <arg1>. \n"
                   "  <arg1> is the path to the platform config file. \n"
                   "- Example: {} \"./config/platform_demo.yml\"  \n", argv[0]);
        return -1;
    }
    CheckFileExistence(path_to_config_file);
    auto platform_config = load_platform_config(path_to_config_file, root_directory);

    // Initiate the router.
    Router router{platform_config.data_file_path.path_to_network_nodes,
                  platform_config.data_file_path.path_to_vehicle_stations,
                  platform_config.data_file_path.path_to_shortest_path_table,
                  platform_config.data_file_path.path_to_mean_travel_time_table,
                  platform_config.data_file_path.path_to_travel_distance_table};

    // Create the demand generator based on the input demand file.
    DemandGenerator demand_generator{platform_config.data_file_path.path_to_taxi_data,
                                     platform_config.simulation_config.simulation_start_time,
                                     platform_config.mod_system_config.request_config.request_density};

    // Create the simulation platform with the config loaded from file.
    Platform<decltype(router), decltype(demand_generator)> platform{std::move(platform_config),
                                                                    std::move(router),
                                                                    std::move(demand_generator)};

    // Run simulation.
    platform.RunSimulation(getTimeStampMs(), (getTimeStampMs() - s_time_ms) / 1000.0);

    return 0;
}
