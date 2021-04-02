/// \author Jian Wen
/// \date 2021/01/29

#include "config.hpp"
#include "demand_generator.hpp"
#include "platform.hpp"
#include "router.hpp"
#include "types.hpp"


#include <iostream>
#include <cstddef>
#include <cstdlib>
#include <fmt/format.h>
#include <yaml-cpp/yaml.h>

int main(int argc, const char *argv[]) {
    // Check the input arugment list.
    std::string path_to_config_file;
    if (argc == 1){
        path_to_config_file = "./config/platform_demo.yml";
    } else if (argc == 2){
        CheckFileExistence((std::string &) argv[1]);
        path_to_config_file = argv[1];
    } else {
        fmt::print(stderr,
                   "[ERROR] \n"
                   "- Usage: <prog name> <arg1>. \n"
                   "  <arg1> is the path to the platform config file. \n"
                   "- Example: {} \"./config/platform_demo.yml\"  \n",argv[0]);
        return -1;
    }
    CheckFileExistence(path_to_config_file);
    auto platform_config = load_platform_config(path_to_config_file);

    // Initiate the router with the osrm data.
    Router router{platform_config.data_file_path};
    fmt::print("[INFO] Router is ready.\n");

    // Create the demand generator based on the input demand file.
    DemandGenerator demand_generator{platform_config.data_file_path.path_to_taxi_data,
                                     platform_config.simulation_config.simulation_start_time,
                                     platform_config.mod_system_config.request_config.request_density};
    fmt::print("[INFO] Demand Generator is ready.\n");

    // Create the simulation platform with the config loaded from file.
    Platform<decltype(router), decltype(demand_generator)> platform{
        std::move(platform_config), std::move(router), std::move(demand_generator)};
    fmt::print("[INFO] Platform is ready.\n");

    // Run simulation.
    platform.run_simulation();

    return 0;
}
