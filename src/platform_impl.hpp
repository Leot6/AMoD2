/// \author Jian Wen
/// \date 2021/02/02

#pragma once

#include "dispatch.hpp"
#include "platform.hpp"

#include <fmt/format.h>

template <typename RouterFunc, typename DemandGeneratorFunc>
Platform<RouterFunc, DemandGeneratorFunc>::Platform(PlatformConfig _platform_config,
                                                    RouterFunc _router_func,
                                                    DemandGeneratorFunc _demand_generator_func)
    : platform_config_(std::move(_platform_config)), router_func_(std::move(_router_func)),
      demand_generator_func_(std::move(_demand_generator_func)) {
    // Initialize the fleet.
    auto s_time = getTimeStamp();
    const auto &fleet_config = platform_config_.mod_system_config.fleet_config;
    auto num_of_stations = router_func_.getNumOfVehicleStations();
    Vehicle vehicle{0, {0, 0, 0}, fleet_config.veh_capacity,
                    0, {}, 0, 0, 0, 0};
    for (auto i = 0; i < fleet_config.fleet_size; i++) {
        size_t station_idx = i * num_of_stations / fleet_config.fleet_size;
        vehicle.id = i;
        vehicle.pos = router_func_.getNodePos(router_func_.getVehicleStationId(station_idx));
        vehicles_.emplace_back(vehicle);
    }
    fmt::print("[INFO] ({}s) Generated {} vehicles.\n",
               float (getTimeStamp() - s_time)/1000,vehicles_.size());

    // Initialize the simulation times.
    system_time_ms_ = 0;
    cycle_ms_ = static_cast<uint64_t>(platform_config_.simulation_config.cycle_s * 1000);
    if (platform_config_.output_config.video_config.render_video) {
        assert(cycle_ms_ % platform_config_.output_config.video_config.frames_per_cycle == 0 &&
               "The cycle time (in milliseconds) must be divisible by frames_per_cycle!");
        frame_ms_ = cycle_ms_ / platform_config_.output_config.video_config.frames_per_cycle;
    } else {
        frame_ms_ = cycle_ms_;
    }
    main_sim_start_time_ms_ =
        static_cast<uint64_t>(platform_config_.simulation_config.warmup_duration_s * 1000);
    main_sim_end_time_ms_ =
        main_sim_start_time_ms_ +
        static_cast<uint64_t>(platform_config_.simulation_config.simulation_duration_s * 1000);
    system_shutdown_time_ms_ =
        main_sim_end_time_ms_ +
        static_cast<uint64_t>(platform_config_.simulation_config.winddown_duration_s * 1000);

    // Open the output datalog file.
    const auto &datalog_config = platform_config_.output_config.datalog_config;
    if (datalog_config.output_datalog) {
        datalog_ofstream_.open(datalog_config.path_to_output_datalog);

        fmt::print("[INFO] Opened the output datalog file at {}.\n",
                   datalog_config.path_to_output_datalog);
    }
}

template <typename RouterFunc, typename DemandGeneratorFunc>
Platform<RouterFunc, DemandGeneratorFunc>::~Platform() {
    // Close the datalog stream.
    if (datalog_ofstream_.is_open()) {
        datalog_ofstream_.close();

        fmt::print("[INFO] Closed the datalog. Program ends.\n");
    }
}

template <typename RouterFunc, typename DemandGeneratorFunc>
void Platform<RouterFunc, DemandGeneratorFunc>::run_simulation() {
    fmt::print("[INFO] Simulation started. Running for total {} seconds.\n",
               system_shutdown_time_ms_ / 1000.0);
    auto start = std::chrono::system_clock::now();

    // Run simulation cycle by cycle.
    while (system_time_ms_ < system_shutdown_time_ms_) {
        run_cycle();
    }

    // Create report.
    std::chrono::duration<double> runtime = std::chrono::system_clock::now() - start;
    fmt::print("[INFO] Simulation completed. Creating report.\n");
    create_report(runtime.count());

    return;
};

template <typename RouterFunc, typename DemandGeneratorFunc>
void Platform<RouterFunc, DemandGeneratorFunc>::run_cycle() {
    fmt::print("[INFO] T = {}s: Cycle {} is running.\n",
               system_time_ms_ / 1000.0,
               system_time_ms_ / cycle_ms_);

    // Advance the vehicles frame by frame in main simulation.
    if (system_time_ms_ >= main_sim_start_time_ms_ && system_time_ms_ < main_sim_end_time_ms_) {
        for (auto ms = 0; ms < cycle_ms_; ms += frame_ms_) {
            advance_vehicles(frame_ms_);

            if (ms < cycle_ms_ - frame_ms_ &&
                platform_config_.output_config.datalog_config.output_datalog) {
                write_to_datalog();
            }
        }
    }
    // Advance the vehicles by the whole cycle in main simulation.
    else {
        advance_vehicles(cycle_ms_);
    }

    // Generate orders.
    const auto pending_order_ids = generate_orders();

    // Dispatch the pending orders.
    dispatch(pending_order_ids);

    if (system_time_ms_ > main_sim_start_time_ms_ && system_time_ms_ <= main_sim_end_time_ms_ &&
        platform_config_.output_config.datalog_config.output_datalog) {
        write_to_datalog();
    }

    return;
}

template <typename RouterFunc, typename DemandGeneratorFunc>
void Platform<RouterFunc, DemandGeneratorFunc>::advance_vehicles(uint64_t time_ms) {
    // Do it for each of the vehicles independently.
    for (auto &vehicle : vehicles_) {
        advance_vehicle(vehicle,
                        orders_,
                        system_time_ms_,
                        time_ms,
                        system_time_ms_ >= main_sim_start_time_ms_ &&
                            system_time_ms_ < main_sim_end_time_ms_);
    }

    // Increment the system time.
    system_time_ms_ += time_ms;

    fmt::print(
        "[DEBUG] T = {}s: Advanced vehicles by {}s.\n", system_time_ms_ / 1000.0, time_ms / 1000.0);

    return;
}

template <typename RouterFunc, typename DemandGeneratorFunc>
std::vector<size_t> Platform<RouterFunc, DemandGeneratorFunc>::generate_orders() {
    // Get order requests generated during the past cycle.
    auto requests = demand_generator_func_(system_time_ms_);

    fmt::print("[DEBUG] T = {}s: Generated {} request(s) in this cycle:\n",
               system_time_ms_ / 1000.0,
               requests.size());

    // Add the requests into the order list as well as the pending orders.
    std::vector<size_t> pending_order_ids;
    for (auto &request : requests) {
        Order order;
        order.id = orders_.size();
        order.origin = router_func_.getNodePos(request.origin_node_id);
        order.destination = router_func_.getNodePos(request.destination_node_id);
        order.status = OrderStatus::REQUESTED;
        order.request_time_ms = request.request_time_ms;
        order.request_time_date = request.request_time_date;
//        strcpy(order.request_time_date, request.request_time_date);
        order.max_pickup_time_ms =
            request.request_time_ms +
            static_cast<uint64_t>(
                platform_config_.mod_system_config.request_config.max_pickup_wait_time_s * 1000);
        pending_order_ids.emplace_back(orders_.size());
        orders_.emplace_back(std::move(order));

        fmt::print("[DEBUG] Order #{} requested at {}, from origin ({}, {}) to destination "
                   "({}, {}):\n",
                   orders_.back().id,
                   orders_.back().request_time_date,
                   orders_.back().origin.lon,
                   orders_.back().origin.lat,
                   orders_.back().destination.lon,
                   orders_.back().destination.lat);
    }

    return pending_order_ids;
}

template <typename RouterFunc, typename DemandGeneratorFunc>
void Platform<RouterFunc, DemandGeneratorFunc>::dispatch(
    const std::vector<size_t> &pending_order_ids) {
    fmt::print("[DEBUG] T = {}s: Dispatching {} pending order(s) to vehicles.\n",
               system_time_ms_ / 1000.0,
               pending_order_ids.size());

    // Assign pending orders to vehicles.
    assign_orders_through_insertion_heuristics(
        pending_order_ids, orders_, vehicles_, system_time_ms_, router_func_);

    // Reoptimize the assignments for better level of service.
    // (TODO)

    // Rebalance empty vehicles.
    // (TODO)

    return;
}

template <typename RouterFunc, typename DemandGeneratorFunc>
void Platform<RouterFunc, DemandGeneratorFunc>::write_to_datalog() {
    YAML::Node node;

    node["system_time_ms"] = system_time_ms_;

    // For each of the vehicles, we write the relavant data in yaml format.
    for (const auto &vehicle : vehicles_) {
        YAML::Node veh_node;

        YAML::Node pos_node;
        pos_node["lon"] = fmt::format("{:.6f}", vehicle.pos.lon);
        pos_node["lat"] = fmt::format("{:.6f}", vehicle.pos.lat);
        veh_node["pos"] = std::move(pos_node);

        YAML::Node waypoints_node;
        for (const auto &waypoint : vehicle.schedule) {
            YAML::Node waypoint_node;
            auto count = 0;
            for (const auto &leg : waypoint.route.legs) {
                for (const auto &step : leg.steps) {
                    for (const auto &pose : step.poses) {
                        YAML::Node leg_node;
                        leg_node["lon"] = fmt::format("{:.6f}", pose.lon);
                        leg_node["lat"] = fmt::format("{:.6f}", pose.lat);
                        waypoint_node.push_back(std::move(leg_node));
                    }
                }
            }
            waypoints_node.push_back(std::move(waypoint_node));
        }
        veh_node["waypoints"] = std::move(waypoints_node);
        node["vehicles"].push_back(std::move(veh_node));
    }

    // For each of the orders, we write the relavant data in yaml format.
    for (const auto &order : orders_) {
        YAML::Node origin_pos_node;
        origin_pos_node["lon"] = fmt::format("{:.6f}", order.origin.lon);
        origin_pos_node["lat"] = fmt::format("{:.6f}", order.origin.lat);

        YAML::Node destination_pos_node;
        destination_pos_node["lon"] = fmt::format("{:.6f}", order.destination.lon);
        destination_pos_node["lat"] = fmt::format("{:.6f}", order.destination.lat);

        YAML::Node order_node;
        order_node["id"] = order.id;
        order_node["origin"] = std::move(origin_pos_node);
        order_node["destination"] = std::move(destination_pos_node);
        order_node["status"] = to_string(order.status);
        order_node["request_time_ms"] = order.request_time_ms;
        order_node["max_pickup_time_ms"] = order.max_pickup_time_ms;
        order_node["pickup_time_ms"] = order.pickup_time_ms;
        order_node["dropoff_time_ms"] = order.dropoff_time_ms;

        node["orders"].push_back(std::move(order_node));
    }

    datalog_ofstream_ << node << std::endl << "---\n";

    fmt::print("[DEBUG] T = {}s: Wrote to datalog.\n", system_time_ms_ / 1000.0);

    return;
}

template <typename RouterFunc, typename DemandGeneratorFunc>
void Platform<RouterFunc, DemandGeneratorFunc>::create_report(double total_runtime_s) {
    fmt::print("-----------------------------------------------------------------------------------"
               "-----------------------------\n");

    // Report the platform configurations
    fmt::print("# System Configurations\n");
    fmt::print(
        " - Simulation Config: simulation_duration = {}s (main simulation between {}s and {}s).\n",
        system_shutdown_time_ms_ / 1000.0,
        main_sim_start_time_ms_ / 1000.0,
        main_sim_end_time_ms_ / 1000.0);
    fmt::print(" - Fleet Config: fleet_size = {}, vehicle_capacity = {}.\n",
               platform_config_.mod_system_config.fleet_config.fleet_size,
               platform_config_.mod_system_config.fleet_config.veh_capacity);
    fmt::print(" - Request Config: max_wait_time = {}s.\n",
               platform_config_.mod_system_config.request_config.max_pickup_wait_time_s);
    fmt::print(" - Output Config: output_datalog = {}, render_video = {}.\n",
               platform_config_.output_config.datalog_config.output_datalog,
               platform_config_.output_config.video_config.render_video);

    // Simulation Runtime
    fmt::print("# Simulation Runtime\n");
    fmt::print(" - Runtime: total_runtime = {}s, average_runtime_per_simulated_second = {}.\n",
               total_runtime_s,
               total_runtime_s * 1000 / system_shutdown_time_ms_);

    // Report order status
    auto order_count = 0;
    auto dispatched_order_count = 0;
    auto completed_order_count = 0;
    auto total_wait_time_ms = 0;
    auto total_travel_time_ms = 0;

    for (const auto &order : orders_) {
        if (order.request_time_ms < main_sim_start_time_ms_) {
            continue;
        } else if (order.request_time_ms >= main_sim_end_time_ms_) {
            break;
        }

        order_count++;

        if (order.status == OrderStatus::WALKAWAY) {
            continue;
        }

        dispatched_order_count++;

        if (order.status == OrderStatus::DROPPED_OFF) {
            completed_order_count++;
            total_wait_time_ms += order.pickup_time_ms - order.request_time_ms;
            total_travel_time_ms += order.dropoff_time_ms - order.pickup_time_ms;
        }
    }

    fmt::print("# Orders\n");
    fmt::print(
        " - Total Orders: requested = {} (of which {} dispatched [{}%] + {} walked away [{}%]).\n",
        order_count,
        dispatched_order_count,
        100.0 * dispatched_order_count / order_count,
        order_count - dispatched_order_count,
        100.0 - 100.0 * dispatched_order_count / order_count);
    fmt::print(" - Travel Time: completed = {}.", completed_order_count);
    if (completed_order_count > 0) {
        fmt::print(" average_wait_time = {}s, average_travel_time = {}s.\n",
                   total_wait_time_ms / 1000.0 / completed_order_count,
                   total_travel_time_ms / 1000.0 / completed_order_count);
    } else {
        fmt::print(" PLEASE USE LONGER SIMULATION DURATION TO BE ABLE TO COMPLETE ORDERS!\n");
    }

    // Report vehicle status
    auto total_dist_traveled_mm = 0;
    auto total_loaded_dist_traveled_mm = 0;

    for (const auto &vehicle : vehicles_) {
        total_dist_traveled_mm += vehicle.dist_traveled_mm;
        total_loaded_dist_traveled_mm += vehicle.loaded_dist_traveled_mm;
    }

    fmt::print("# Vehicles\n");
    fmt::print(
        " - Distance: average_distance_traveled = {}m. average_distance_traveled_per_hour = {}m.\n",
        total_dist_traveled_mm / 1000.0 / vehicles_.size(),
        total_dist_traveled_mm / vehicles_.size() * 3600.0 /
            (main_sim_end_time_ms_ - main_sim_start_time_ms_));
    fmt::print(" - Load: average_load = {}.\n",
               total_loaded_dist_traveled_mm * 1.0 / total_dist_traveled_mm);

    fmt::print("-----------------------------------------------------------------------------------"
               "-----------------------------\n");
    return;
}
