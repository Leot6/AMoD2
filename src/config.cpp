/// \author Jian Wen
/// \date 2021/02/01

#include "config.hpp"

#include <fmt/format.h>

PlatformConfig load_platform_config(const std::string &path_to_platform_config) {
    auto platform_config_yaml = YAML::LoadFile(path_to_platform_config);

    PlatformConfig platform_config;

    platform_config.data_file_path.path_to_osrm =
            platform_config_yaml["data_file_path"]["osrm"].as<std::string>();
    platform_config.data_file_path.path_to_vehicle_stations =
            platform_config_yaml["data_file_path"]["vehicle_stations"].as<std::string>();
    platform_config.data_file_path.path_to_network_nodes =
            platform_config_yaml["data_file_path"]["network_nodes"].as<std::string>();
    platform_config.data_file_path.path_to_shortest_path_table =
            platform_config_yaml["data_file_path"]["shortest_path_table"].as<std::string>();
    platform_config.data_file_path.path_to_mean_travel_time_table =
            platform_config_yaml["data_file_path"]["mean_travel_time_table"].as<std::string>();
    platform_config.data_file_path.path_to_travel_distance_table =
            platform_config_yaml["data_file_path"]["travel_distance_table"].as<std::string>();
    platform_config.data_file_path.path_to_taxi_data =
            platform_config_yaml["data_file_path"]["taxi_data"].as<std::string>();

    platform_config.area_config.lon_min =
            platform_config_yaml["area_config"]["lon_min"].as<float>();
    platform_config.area_config.lon_max =
            platform_config_yaml["area_config"]["lon_max"].as<float>();
    platform_config.area_config.lat_min =
            platform_config_yaml["area_config"]["lat_min"].as<float>();
    platform_config.area_config.lat_max =
            platform_config_yaml["area_config"]["lat_max"].as<float>();

    platform_config.mod_system_config.fleet_config.fleet_size =
            platform_config_yaml["mod_system_config"]["fleet_config"]["fleet_size"].as<size_t>();
    platform_config.mod_system_config.fleet_config.veh_capacity =
            platform_config_yaml["mod_system_config"]["fleet_config"]["veh_capacity"].as<size_t>();

    platform_config.mod_system_config.request_config.request_density =
            platform_config_yaml["mod_system_config"]["request_config"]["request_density"]
                    .as<float>();
    platform_config.mod_system_config.request_config.max_pickup_wait_time_s =
            platform_config_yaml["mod_system_config"]["request_config"]["max_pickup_wait_time_s"]
                    .as<double>();

    platform_config.simulation_config.simulation_start_time =
            platform_config_yaml["simulation_config"]["simulation_start_time"].as<std::string>();
    platform_config.simulation_config.cycle_s =
            platform_config_yaml["simulation_config"]["cycle_s"].as<double>();
    platform_config.simulation_config.simulation_duration_s =
            platform_config_yaml["simulation_config"]["simulation_duration_s"].as<double>();
    platform_config.simulation_config.warmup_duration_s =
            platform_config_yaml["simulation_config"]["warmup_duration_s"].as<double>();
    platform_config.simulation_config.winddown_duration_s =
            platform_config_yaml["simulation_config"]["winddown_duration_s"].as<double>();

    platform_config.output_config.datalog_config.output_datalog =
            platform_config_yaml["output_config"]["datalog_config"]["output_datalog"].as<bool>();
    platform_config.output_config.datalog_config.path_to_output_datalog =
            platform_config_yaml["output_config"]["datalog_config"]["path_to_output_datalog"]
                    .as<std::string>();
    platform_config.output_config.video_config.render_video =
            platform_config_yaml["output_config"]["video_config"]["render_video"].as<bool>();
    platform_config.output_config.video_config.path_to_output_video =
            platform_config_yaml["output_config"]["video_config"]["path_to_output_video"]
                    .as<std::string>();
    platform_config.output_config.video_config.frames_per_cycle =
            platform_config_yaml["output_config"]["video_config"]["frames_per_cycle"].as<size_t>();
    platform_config.output_config.video_config.replay_speed =
            platform_config_yaml["output_config"]["video_config"]["replay_speed"].as<double>();

    fmt::print("[INFO] Loaded the platform configuration yaml file from {}.\n",
               path_to_platform_config);

    // Sanity check of the input config.
    if (platform_config.output_config.datalog_config.output_datalog) {
        assert(platform_config.output_config.datalog_config.path_to_output_datalog != "" &&
               "Config must have non-empty path_to_output_datalog if output_datalog is true!");
    }
    if (platform_config.output_config.video_config.render_video) {
        assert(platform_config.output_config.datalog_config.output_datalog &&
               "Config must have output_datalog config on if render_video is true!");
        assert(platform_config.output_config.video_config.path_to_output_video != "" &&
               "Config must have non-empty path_to_output_video if render_video is true!");
        assert(platform_config.output_config.video_config.frames_per_cycle > 0 &&
               "Config must have positive frames_per_cycle if render_video is true!");
        assert(platform_config.output_config.video_config.replay_speed > 0 &&
               "Config must have positive frames_per_cycle if render_video is true!");
    }

    return platform_config;
}
