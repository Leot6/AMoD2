/// \author Jian Wen
/// \date 2021/01/29

#include "config.hpp"
#include "demand_generator.hpp"
#include "platform.hpp"
#include "router.hpp"


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

//    // debug code
//    fmt::print("[DEBUG] Routing benchmark test:\n");
//    Pos origin1 = {1499, -73.983177, 40.744562};
//    Pos destination1 = {2494, -73.962211, 40.770424};
//    Pos origin2 = {1, -74.017946, 40.706991};
//    Pos destination2 = {4091, -73.911227, 40.871655};
//    auto s_time = getTimeStamp();
//    auto route_response11 = router(origin1, destination1, RoutingType::FULL_ROUTE);
//    fmt::print("  table (full): route1 takes {}s and {}m. ({}ps)\n",
//               (float)route_response11.route.duration_ms/1000, (float)route_response11.route.distance_mm/1000,
//               float (getTimeStamp() - s_time)*pow(10, 12));
//
//    s_time = getTimeStamp();
//    auto route_response22 = router.Routing(origin1, destination1, RoutingType::FULL_ROUTE);
//    fmt::print("  osrm (full): route2 takes {}s and {}m. ({}ms)\n",
//               (float)route_response22.route.duration_ms/1000, (float)route_response22.route.distance_mm/1000,
//               float (getTimeStamp() - s_time));

//    fmt::print("[DEBUG] route 1 has {} legs, first leg ({}s, {}m) has {} steps\n",
//               route_response11.route.legs.size(),
//               (float)route_response11.route.legs[0].duration_ms/1000,
//               (float)route_response11.route.legs[0].distance_mm/1000,
//               route_response11.route.legs[0].steps.size());
//    auto & steps1 = route_response11.route.legs[0].steps;
//    for (int i = 0; i< steps1.size(); i++) {
//        fmt::print("[DEBUG] printing step {} ({} poses), t = {}s, d = {}m\n",
//                   i+1, steps1[i].poses.size(), (float)steps1[i].duration_ms/1000, (float)steps1[i].distance_mm/1000);
//        auto & poses = steps1[i].poses;
//        for (int j = 0; j < poses.size(); j++) {
//            fmt::print("      printing pos {} ({}, {}) \n", j+1, poses[j].lon, poses[j].lat);
//        }
//    }
//    fmt::print("[DEBUG] route 2 has {} legs, first leg ({}s, {}m) has {} steps\n",
//               route_response22.route.legs.size(),
//               (float)route_response22.route.legs[0].duration_ms/1000,
//               (float)route_response22.route.legs[0].distance_mm/1000,
//               route_response22.route.legs[0].steps.size());
//    auto & steps2 = route_response22.route.legs[0].steps;
//    for (int i = 0; i< steps2.size(); i++) {
//        fmt::print("[DEBUG] printing step {} ({} poses), t = {}s, d = {}m\n",
//                   i+1, steps2[i].poses.size(), (float)steps2[i].duration_ms/1000, (float)steps2[i].distance_mm/1000);
//        auto & poses = steps2[i].poses;
//        for (int j = 0; j < poses.size(); j++) {
//            fmt::print("      printing pos {} ({}, {}) \n", j+1, poses[j].lon, poses[j].lat);
//        }
//    }
//
//    fmt::print("[INFO] Done\n");
//    exit(0);


    // Create the demand generator based on the input demand file.
    DemandGenerator demand_generator{platform_config.data_file_path.path_to_taxi_data,
                                     platform_config.simulation_config.simulation_start_time,
                                     platform_config.mod_system_config.request_config.request_density};


    // Create the simulation platform with the config loaded from file.
    Platform<decltype(router), decltype(demand_generator)> platform{
        std::move(platform_config), std::move(router), std::move(demand_generator)};


    // Run simulation.
    platform.run_simulation();

    return 0;
}
