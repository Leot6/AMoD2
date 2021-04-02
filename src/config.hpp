/// \author Jian Wen
/// \date 2021/02/01

#pragma once

#include "types.hpp"

/// \brief Config that describes the data file path.
struct DataFilePath {
    std::string path_to_osrm = "";
    std::string path_to_vehicle_stations = "";
    std::string path_to_network_nodes = "";
    std::string path_to_shortest_path_table = "";
    std::string path_to_mean_travel_time_table = "";
    std::string path_to_travel_distance_table = "";
    std::string path_to_taxi_data = "";
};

/// \brief Config that describes the simulated area.
struct AreaConfig {
    float lon_min = 0.0; // max longitude accepted
    float lon_max = 0.0; // min longitude accepted
    float lat_min = 0.0; // max latitude accepted
    float lat_max = 0.0; // min latitude accepted
};

/// \brief Config that describes the fleet.
struct FleetConfig {
    size_t fleet_size = 10;  // fleet size
    size_t veh_capacity = 2; // vehicle capacity, 1 = non-shared, >2 is shared
};

/// \brief Config that describes the requests.
struct RequestConfig {
    float request_density = 1.0; // the percentage of taxi data considered
    double max_pickup_wait_time_s = 600; // the max wait time allowed between a request is generated
                                         // and the traveler is picked up
};

/// \brief Config that describes the simulated MoD system.
struct MoDSystemConfig {
    FleetConfig fleet_config;
    RequestConfig request_config;
};

/// \brief Config that describes the simulation parameters.
struct SimulationConfig {
    std::string simulation_start_time = ""; // the time of the day that simulation starts
    double cycle_s = 60; // the cycle every x second the platform dispatches the requests in batch
    double simulation_duration_s =
        600; // the main period during which the simulated data is used for analysis
    double warmup_duration_s = 1200;   // the period before the main sim to build up states
    double winddown_duration_s = 1200; // the period after the main sim to close trips
};

/// \brief Config for the output datalog.
struct DatalogConfig {
    bool output_datalog = false;             // true if we output datalog
    std::string path_to_output_datalog = ""; // the path to the output datalog, empty if no output
};

/// \brief Config for video rendering.
struct VideoConfig {
    bool render_video = false;             // true if we render video
    std::string path_to_output_video = ""; // the path to the output video, empty if no rendering
    size_t frames_per_cycle = 10;          // the number of frames in each cycle
    double replay_speed = 60; // the speed of the video replay as compared to the actual system time
};

/// \brief Config that describes the output modes for datalog and video.
struct OutputConfig {
    DatalogConfig datalog_config;
    VideoConfig video_config;
};

/// \brief The set of config parameters for the simulation platform.
struct PlatformConfig {
    DataFilePath data_file_path;
    AreaConfig area_config;
    MoDSystemConfig mod_system_config;
    SimulationConfig simulation_config;
    OutputConfig output_config;
};

/// \brief Load yaml platform config and convert into the C++ data struct.
PlatformConfig load_platform_config(const std::string &path_to_platform_config);
