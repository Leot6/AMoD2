/// \author Jian Wen
/// \date 2021/02/02

#pragma once

#include "platform.hpp"
#include <fmt/format.h>
#undef NDEBUG
#include <assert.h>

template <typename RouterFunc, typename DemandGeneratorFunc>
Platform<RouterFunc, DemandGeneratorFunc>::Platform(PlatformConfig _platform_config,
                                                    RouterFunc _router_func,
                                                    DemandGeneratorFunc _demand_generator_func)
    : platform_config_(std::move(_platform_config)), router_func_(std::move(_router_func)),
      demand_generator_func_(std::move(_demand_generator_func)) {

    // Initialize the fleet.
    auto s_time_ms = getTimeStampMs();
    const auto &fleet_config = platform_config_.mod_system_config.fleet_config;
    auto num_of_stations = router_func_.getNumOfVehicleStations();
    Vehicle vehicle;
    vehicle.capacity = fleet_config.veh_capacity;
    size_t station_idx;
    for (auto i = 0; i < fleet_config.fleet_size; i++) {
        station_idx = i * num_of_stations / fleet_config.fleet_size;
        vehicle.id = i;
        vehicle.pos = router_func_.getNodePos(router_func_.getVehicleStationId(station_idx));
        vehicles_.push_back(vehicle);
    }

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

    // Initialize the dispatcher and the rebalancer.
    if (platform_config_.mod_system_config.dispatch_config.dispatcher == "GI") {
        dispatcher_ = DispatcherMethod::GI;
    } else if (platform_config_.mod_system_config.dispatch_config.dispatcher == "SBA") {
        dispatcher_ = DispatcherMethod::SBA;
    } else if (platform_config_.mod_system_config.dispatch_config.dispatcher == "OSP") {
        dispatcher_ = DispatcherMethod::OSP;
    }
    if (platform_config_.mod_system_config.dispatch_config.rebalancer == "NONE") {
       rebalancer_ = RebalancerMethod::NONE;
    } else if (platform_config_.mod_system_config.dispatch_config.rebalancer == "NR") {
        rebalancer_ = RebalancerMethod::NR;
    }

    // Open the output datalog file.
    const auto &datalog_config = platform_config_.output_config.datalog_config;
    if (datalog_config.output_datalog) {
        datalog_ofstream_.open(datalog_config.path_to_output_datalog);

        fmt::print("[INFO] Opened the output datalog file at {}.\n",
                   datalog_config.path_to_output_datalog);
    }
    fmt::print("[INFO] Platform is ready.\n");

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
void Platform<RouterFunc, DemandGeneratorFunc>::RunSimulation(std::time_t simulation_start_time_stamp,
                                                               float total_init_time_s) {
    CreateReport(simulation_start_time_stamp, total_init_time_s, 0.0);
    std::time_t main_sim_start_time_stamp;
    std::time_t main_sim_end_time_stamp;

    // Run simulation cycle by cycle.
    tqdm bar1("AMoD",system_shutdown_time_ms_ / cycle_ms_);
    std::string progress_phase = "Warm Up";
    while (system_time_ms_ < system_shutdown_time_ms_) {
        if (system_time_ms_ < main_sim_start_time_ms_) {
            progress_phase = "Warm Up";
        } else if (system_time_ms_ >= main_sim_start_time_ms_ && system_time_ms_ < main_sim_end_time_ms_) {
            progress_phase = "Main Study";
        } else {
            progress_phase = "Cool Down";
        }
        if (!DEBUG_PRINT) { bar1.progress(progress_phase); }
        if (system_time_ms_ == main_sim_start_time_ms_) { main_sim_start_time_stamp = getTimeStampMs(); }
        RunCycle(progress_phase);
        if (system_time_ms_ == main_sim_end_time_ms_) { main_sim_end_time_stamp = getTimeStampMs(); }
    }
    if (!DEBUG_PRINT) { bar1.finish(); }

    // Create report.
    auto main_sim_runtime_s = (main_sim_end_time_stamp - main_sim_start_time_stamp) / 1000.0;
    fmt::print("[INFO] Simulation completed. Creating report.\n");
    CreateReport(simulation_start_time_stamp, total_init_time_s, main_sim_runtime_s);
};

template <typename RouterFunc, typename DemandGeneratorFunc>
void Platform<RouterFunc, DemandGeneratorFunc>::RunCycle(std::string progress_phase) {
    TIMER_START(t)
    if (DEBUG_PRINT) {
        fmt::print("[DEBUG] T = {}s: Epoch {}/{} is running. [{}]\n",
                   (system_time_ms_) / 1000.0,
                   (system_time_ms_ + cycle_ms_) / cycle_ms_,
                   system_shutdown_time_ms_ / cycle_ms_,
                   progress_phase);
    }

    // 1. Update the vehicles' positions and the orders' statuses. (system_time_ms_ is updated at this step.)
    if (system_time_ms_ >= main_sim_start_time_ms_ && system_time_ms_ < main_sim_end_time_ms_) {
        // Advance the vehicles frame by frame. (if render_video=false, then frame_ms_=cycle_ms_.)
        for (auto ms = 0; ms < cycle_ms_; ms += frame_ms_) {
            UpdVehiclesPositions(frame_ms_);
            if (ms < cycle_ms_ - frame_ms_ &&
                platform_config_.output_config.datalog_config.output_datalog) {
                WriteToDatalog();
            }
        }
    } else {
        // Advance the vehicles by the whole cycle.
        UpdVehiclesPositions(cycle_ms_);
    }
    for (auto &order : orders_) {
        if (order.status != OrderStatus::PENDING) { continue; }
        // Reject the long waited orders.
        if (order.request_time_ms + 150000 <= system_time_ms_ || order.max_pickup_time_ms <= system_time_ms_) {
            order.status = OrderStatus::WALKAWAY;
        }
    }

    // 2. Generate orders.
    const auto new_received_order_ids = GenerateOrders();

    // 3. Assign pending orders to vehicles.
    if (system_time_ms_ > main_sim_start_time_ms_ && system_time_ms_ <= main_sim_end_time_ms_) {
        if (dispatcher_ == DispatcherMethod::GI) {
            AssignOrdersThroughGreedyInsertion(
                    new_received_order_ids, orders_, vehicles_, system_time_ms_, router_func_);
        } else if (dispatcher_ == DispatcherMethod::SBA) {
            AssignOrdersThroughSingleRequestBatchAssign(
                    new_received_order_ids, orders_, vehicles_, system_time_ms_, router_func_);
        } else if (dispatcher_ == DispatcherMethod::OSP) {
            AssignOrdersThroughOptimalSchedulePoolAssign(
                    new_received_order_ids, orders_, vehicles_, system_time_ms_, router_func_);
        }
    } else {
        AssignOrdersThroughSingleRequestBatchAssign(
                new_received_order_ids, orders_, vehicles_, system_time_ms_, router_func_);
    }

    // 4. Reposition idle vehicles to high demand areas.
    if (rebalancer_ == RebalancerMethod::NR) {
        RepositionIdleVehicleThroughNaiveRebalancer(orders_, vehicles_, router_func_);
    }

    // 5. Write the datalog to file.
    if (system_time_ms_ > main_sim_start_time_ms_ && system_time_ms_ <= main_sim_end_time_ms_ &&
        platform_config_.output_config.datalog_config.output_datalog) {
        WriteToDatalog();
    }

    if (DEBUG_PRINT) {
        // 6. Check the statuses of orders, to make sure that no one is assigned to multiple vehicles.
        auto num_of_total_orders = orders_.size();
        auto num_of_complete_orders = 0, num_of_onboard_orders = 0, num_of_picking_orders = 0,
                num_of_pending_orders = 0, num_of_walkaway_orders = 0;
        for (const auto &order : orders_) {
            if (order.status == OrderStatus::COMPLETE) {
                num_of_complete_orders++;
            } else if (order.status == OrderStatus::ONBOARD) {
                num_of_onboard_orders++;
            } else if (order.status == OrderStatus::PICKING) {
                num_of_picking_orders++;
            } else if (order.status == OrderStatus::PENDING) {
                num_of_pending_orders++;
            } else if (order.status == OrderStatus::WALKAWAY) {
                num_of_walkaway_orders++;
            }
        }
        assert(num_of_total_orders == num_of_complete_orders + num_of_onboard_orders + num_of_picking_orders
                                      + num_of_pending_orders + num_of_walkaway_orders);
        auto num_of_onboard_orders_from_vehicle_schedule = 0, num_of_picking_orders_from_vehicle_schedule = 0,
                num_of_dropping_orders_from_vehicle_schedule = 0;
        for (const auto &vehicle : vehicles_) {
            num_of_onboard_orders_from_vehicle_schedule += vehicle.onboard_order_ids.size();
            for (const auto &wp : vehicle.schedule) {
                if (wp.op == WaypointOp::PICKUP) { num_of_picking_orders_from_vehicle_schedule += 1; }
                if (wp.op == WaypointOp::DROPOFF) { num_of_dropping_orders_from_vehicle_schedule += 1; }
            }
        }
        assert(num_of_onboard_orders_from_vehicle_schedule + num_of_picking_orders_from_vehicle_schedule
               == num_of_dropping_orders_from_vehicle_schedule);
        assert(num_of_picking_orders == num_of_picking_orders_from_vehicle_schedule);
        assert(num_of_onboard_orders == num_of_onboard_orders_from_vehicle_schedule);

        fmt::print("        T = {}s: Epoch {}/{} has finished. Total orders received = {}, of which {} complete "
                   "+ {} onboard + {} picking + {} pending + {} walkaway",
                   (system_time_ms_) / 1000.0,
                   (system_time_ms_) / cycle_ms_,
                   system_shutdown_time_ms_ / cycle_ms_,
                   num_of_total_orders, num_of_complete_orders,
                   num_of_onboard_orders, num_of_picking_orders, num_of_pending_orders, num_of_walkaway_orders);
        TIMER_END(t)
        fmt::print("\n");
    }
}

template <typename RouterFunc, typename DemandGeneratorFunc>
void Platform<RouterFunc, DemandGeneratorFunc>::UpdVehiclesPositions(uint64_t time_ms) {

    TIMER_START(t)
    if (DEBUG_PRINT) {
        fmt::print("        -Updating vehicles positions and orders statues by {}s...\n", time_ms / 1000);
    }

    auto num_of_picked_orders = 0;
    auto num_of_dropped_orders = 0;

    // Do it for each of the vehicles independently.
    for (auto &vehicle : vehicles_) {
        auto[new_picked_order_ids, new_dropped_order_ids] =
        UpdVehiclePos(vehicle,
                      orders_,
                      system_time_ms_,
                      time_ms,
                      (system_time_ms_ > main_sim_start_time_ms_ &&
                       system_time_ms_ <= main_sim_end_time_ms_));
        num_of_picked_orders += new_picked_order_ids.size();
        num_of_dropped_orders += new_dropped_order_ids.size();
    }

    // Increment the system time.
    system_time_ms_ += time_ms;

    if (DEBUG_PRINT) {
        int num_of_idle_vehicles = 0;
        int num_of_rebalancing_vehicles = 0;
        for (const auto &vehicle : vehicles_) {
            if (vehicle.status == VehicleStatus::IDLE) { num_of_idle_vehicles++; }
            else if (vehicle.status == VehicleStatus::REBALANCING) { num_of_rebalancing_vehicles++; }
        }
        fmt::print("            +Picked orders: {}, Dropped orders: {}\n",
                   num_of_picked_orders, num_of_dropped_orders);
        fmt::print("            +Idle vehicles: {}/{}, Rebalancing vehicles: {}/{}",
                   num_of_idle_vehicles, vehicles_.size(), num_of_rebalancing_vehicles, vehicles_.size());
        TIMER_END(t)
    }
}

template <typename RouterFunc, typename DemandGeneratorFunc>
std::vector<size_t> Platform<RouterFunc, DemandGeneratorFunc>::GenerateOrders() {
    TIMER_START(t)
    if (DEBUG_PRINT) {
        fmt::print("        -Loading new orders...\n");
    }

    // Get order requests generated during the past cycle.
    auto requests = demand_generator_func_(system_time_ms_);
    int32_t max_pickup_wait_time = platform_config_.mod_system_config.request_config.max_pickup_wait_time_s * 1000;
    auto max_detour = platform_config_.mod_system_config.request_config.max_onboard_detour;

    // Add the new received requests into the order list.
    std::vector<size_t> new_received_order_ids;
    for (auto &request : requests) {
        Order order;
        order.id = orders_.size();
        order.origin = router_func_.getNodePos(request.origin_node_id);
        order.destination = router_func_.getNodePos(request.destination_node_id);
        order.request_time_ms = request.request_time_ms;
        order.request_time_date = request.request_time_date;
        order.shortest_travel_time_ms =
                router_func_(order.origin, order.destination, RoutingType::TIME_ONLY).duration_ms;
        // max_wait = min(max_pickup_wait_time, shortest_travel_time * 0.7),
        // max_total_delay = min(max_pickup_wait_time * 2, MaxWait + ShortestTravelTime * 0.3)
        order.max_pickup_time_ms =
                request.request_time_ms
                + std::min(max_pickup_wait_time, static_cast<int32_t>(order.shortest_travel_time_ms * (2 - max_detour)));
        order.max_dropoff_time_ms =
                request.request_time_ms + order.shortest_travel_time_ms
                + std::min(max_pickup_wait_time * 2,
                         order.max_pickup_time_ms - order.request_time_ms
                         + static_cast<int32_t>(order.shortest_travel_time_ms * (max_detour - 1)));
        new_received_order_ids.push_back(orders_.size());
        assert(order.status == OrderStatus::PENDING);
        orders_.push_back(std::move(order));

//        if (DEBUG_PRINT) {
//            fmt::print("            +Order {} requested at {}s ({}), from {} to {}, Ts = {}s\n",
//                       orders_.back().id,
//                       orders_.back().request_time_ms / 1000,
//                       orders_.back().request_time_date,
//                       orders_.back().origin.node_id,
//                       orders_.back().destination.node_id,
//                       orders_.back().shortest_travel_time_ms / 1000.0 );
//        }

    }

    if (DEBUG_PRINT) {
        fmt::print("            +Orders new received: {}", requests.size());
        TIMER_END(t)
    }

    return new_received_order_ids;
}


template <typename RouterFunc, typename DemandGeneratorFunc>
void Platform<RouterFunc, DemandGeneratorFunc>::WriteToDatalog() {
    TIMER_START(t)
    if (DEBUG_PRINT) {
        fmt::print("        -Writing to datalog ()...");
    }

    YAML::Node node;
    node["system_time_ms"] = system_time_ms_;

    // For each of the vehicles, we write the relevant data in yaml format.
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
            for (const auto &step : waypoint.route.steps) {
                for (const auto &pose : step.poses) {
                    YAML::Node step_node;
                    step_node["lon"] = fmt::format("{:.6f}", pose.lon);
                    step_node["lat"] = fmt::format("{:.6f}", pose.lat);
                    waypoint_node.push_back(std::move(step_node));
                }
            }
            waypoints_node.push_back(std::move(waypoint_node));
        }
        veh_node["waypoints"] = std::move(waypoints_node);
        node["vehicles"].push_back(std::move(veh_node));
    }

    // For each of the orders, we write the relevant data in yaml format.
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
        order_node["status"] = order_status_to_string(order.status);
        order_node["request_time_ms"] = order.request_time_ms;
        order_node["max_pickup_time_ms"] = order.max_pickup_time_ms;
        order_node["pickup_time_ms"] = order.pickup_time_ms;
        order_node["dropoff_time_ms"] = order.dropoff_time_ms;

        node["orders"].push_back(std::move(order_node));
    }

    datalog_ofstream_ << node << std::endl << "---\n";

    if (DEBUG_PRINT) { TIMER_END(t) }
}

template <typename RouterFunc, typename DemandGeneratorFunc>
void Platform<RouterFunc, DemandGeneratorFunc>::CreateReport(std::time_t simulation_start_time_stamp,
                                                             float total_init_time_s,
                                                             float main_sim_runtime_s) {
    // Get the width of the current console window.
    struct winsize size;
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &size);
    int window_width = size.ws_col;
    if (window_width == 0 || window_width > 90) { window_width = 90; }
    std::string dividing_line(window_width, '-');
    fmt::print("{}\n", dividing_line);

    // Get the real world time when the simulation starts and ends.
    auto simulation_start_time_real_world_date = ConvertTimeSecondToDate(simulation_start_time_stamp / 1000);
    auto simulation_end_time_stamp = getTimeStampMs();
    std::string simulation_end_time_real_world_date;
    if (orders_.size() == 0) {
        simulation_end_time_real_world_date = "0000-00-00 00:00:00";
    } else {
        simulation_end_time_real_world_date = ConvertTimeSecondToDate(simulation_end_time_stamp / 1000);
    }
    auto total_sim_runtime_s = (simulation_end_time_stamp - simulation_start_time_stamp) / 1000.0;

    // Convert the running time to format h:m:s.
    int time_consumed_s_1 = total_sim_runtime_s;
    int time_consumed_min_1 = time_consumed_s_1 / 60;
    time_consumed_s_1 %= 60;
    int time_consumed_hour_1 = time_consumed_min_1 / 60;
    time_consumed_min_1 %= 60;
    std::string total_sim_run_time_formatted =
            fmt::format("{}:{:02d}:{:02d}", time_consumed_hour_1, time_consumed_min_1, time_consumed_s_1);
    int time_consumed_s_2 = main_sim_runtime_s;
    int time_consumed_min_2 = time_consumed_s_2 / 60;
    time_consumed_s_2 %= 60;
    int time_consumed_hour_2 = time_consumed_min_2 / 60;
    time_consumed_min_2 %= 60;
    std::string main_sim_run_time_formatted =
            fmt::format("{}:{:02d}:{:02d}", time_consumed_hour_2, time_consumed_min_2, time_consumed_s_2);

    // Get some system configurations.
    auto file_path = platform_config_.data_file_path.path_to_taxi_data;
    auto request_number = file_path.substr(file_path.length() - 9, 5);
    if (request_number.substr(0, 1) == "-") { request_number = request_number.substr(1, 4); }
    auto sim_start_time_date = platform_config_.simulation_config.simulation_start_time;
    auto sim_end_time_date = ConvertTimeSecondToDate(
            ConvertTimeDateToSeconds(sim_start_time_date) + system_shutdown_time_ms_ / 1000);
    auto main_sim_start_date = ConvertTimeSecondToDate(
            ConvertTimeDateToSeconds(sim_start_time_date) + main_sim_start_time_ms_ / 1000);
    auto main_sim_end_date = ConvertTimeSecondToDate(
            ConvertTimeDateToSeconds(sim_start_time_date) + main_sim_end_time_ms_ / 1000);
    auto num_of_epochs = system_shutdown_time_ms_ / cycle_ms_;
    auto num_of_main_epochs = platform_config_.simulation_config.simulation_duration_s / (cycle_ms_ / 1000);
    auto frame_length_s = cycle_ms_ / 1000 / platform_config_.output_config.video_config.frames_per_cycle;
    auto video_frames =  platform_config_.simulation_config.simulation_duration_s / frame_length_s;
    auto video_fps = platform_config_.output_config.video_config.replay_speed / frame_length_s;
    auto video_duration = video_frames / video_fps;

    // Simulation Runtime.
    fmt::print("# Simulation Runtime\n");
    fmt::print("  - Start: {}, End: {}, Time: {}.\n",
               simulation_start_time_real_world_date, simulation_end_time_real_world_date,
               total_sim_run_time_formatted);
    fmt::print("  - Main Simulation: init_time = {:.2f} s, runtime = {}, avg_time = {:.2f} s.\n",
               total_init_time_s, main_sim_run_time_formatted, main_sim_runtime_s / num_of_main_epochs);

    // Report the platform configurations.
    fmt::print("# System Configurations\n");
    fmt::print("  - From {} to {}. (main simulation between {} and {}).\n",
               sim_start_time_date.substr(11,20), sim_end_time_date.substr(11,20),
               main_sim_start_date.substr(11,20), main_sim_end_date.substr(11,20));
    fmt::print("  - Fleet Config: size = {}, capacity = {}. ({} + {} + {} = {} epochs).\n",
               platform_config_.mod_system_config.fleet_config.fleet_size,
               platform_config_.mod_system_config.fleet_config.veh_capacity,
               platform_config_.simulation_config.warmup_duration_s / (cycle_ms_ / 1000),
               num_of_main_epochs,
               platform_config_.simulation_config.winddown_duration_s / (cycle_ms_ / 1000), num_of_epochs);
    fmt::print("  - Order Config: density = {} ({}), max_wait = {} s. (Δt = {} s).\n",
               platform_config_.mod_system_config.request_config.request_density,
               request_number,
               platform_config_.mod_system_config.request_config.max_pickup_wait_time_s,
               cycle_ms_ / 1000);
    fmt::print("  - Dispatch Config: dispatcher = {}, rebalancer = {}.\n",
               platform_config_.mod_system_config.dispatch_config.dispatcher,
               platform_config_.mod_system_config.dispatch_config.rebalancer);
    fmt::print("  - Video Config: {}, frame_length = {} s, fps = {}, duration = {} s.\n",
               platform_config_.output_config.video_config.render_video,
               frame_length_s, video_fps, video_duration);

    if (orders_.size() == 0) {
        fmt::print("{}\n", dividing_line);
        return;
    }

    // Report order status.
    auto order_count = 0;
    auto walkaway_order_count = 0;
    auto complete_order_count = 0;
    auto onboard_order_count = 0;
    auto picking_order_count = 0;
    auto pending_order_count = 0;
    uint64_t total_wait_time_ms = 0;
    uint64_t total_delay_time_ms = 0;
    uint64_t total_order_time_ms = 0;

    for (const auto &order : orders_) {
        if (order.request_time_ms < main_sim_start_time_ms_) { continue; }
        if (order.request_time_ms >= main_sim_end_time_ms_) { break; }
        order_count++;
        if (order.status == OrderStatus::WALKAWAY) {
            walkaway_order_count++;
        } else if (order.status == OrderStatus::COMPLETE) {
            complete_order_count++;
            total_wait_time_ms += order.pickup_time_ms - order.request_time_ms;
            total_delay_time_ms += order.dropoff_time_ms - (order.request_time_ms + order.shortest_travel_time_ms);
            total_order_time_ms += order.shortest_travel_time_ms;
        } else if (order.status == OrderStatus::ONBOARD) {
            onboard_order_count++;
        } else if (order.status == OrderStatus::PICKING) {
            picking_order_count++;
        } else if (order.status == OrderStatus::PENDING) {
            pending_order_count++;
        }
    }

    auto service_order_count = complete_order_count + onboard_order_count;
    assert(service_order_count + picking_order_count + pending_order_count == order_count - walkaway_order_count);
    fmt::print("# Orders ({}/{})\n", order_count - walkaway_order_count, order_count);
    fmt::print("  - complete = {} ({:.2f}%), onboard = {} ({:.2f}%), total_service = {} ({:.2f}%).\n",
               complete_order_count, 100.0 * complete_order_count / order_count,
               onboard_order_count, 100.0 * onboard_order_count / order_count,
               service_order_count, 100.0 * service_order_count / order_count);
    if (picking_order_count + pending_order_count > 0) {
        fmt::print("  - picking = {} ({:.2f}%), pending = {} ({:.2f}%).\n",
                   picking_order_count,100.0 * picking_order_count / order_count,
                   pending_order_count, 100.0 * pending_order_count / order_count);
    }
    if (complete_order_count > 0) {
        fmt::print("  - avg_shortest_travel = {:.2f} s, avg_wait = {:.2f} s, avg_delay = {:.2f} s.\n",
                   total_order_time_ms / 1000.0 / complete_order_count,
                   total_wait_time_ms / 1000.0 / complete_order_count,
                   total_delay_time_ms / 1000.0 / complete_order_count);
    } else {
        fmt::print("  [PLEASE USE LONGER SIMULATION DURATION TO BE ABLE TO COMPLETE ORDERS!]\n");
    }

    // Report vehicle status.
    uint64_t total_dist_traveled_mm = 0;
    uint64_t total_loaded_dist_traveled_mm = 0;
    uint64_t total_empty_dist_traveled_mm = 0;
    uint64_t total_rebl_dist_traveled_mm = 0;
    uint64_t total_time_traveled_ms = 0;
    uint64_t total_loaded_time_traveled_ms = 0;
    uint64_t total_empty_time_traveled_ms = 0;
    uint64_t total_rebl_time_traveled_ms = 0;

    for (const auto &vehicle : vehicles_) {
        total_dist_traveled_mm += vehicle.dist_traveled_mm;
        total_loaded_dist_traveled_mm += vehicle.loaded_dist_traveled_mm;
        total_empty_dist_traveled_mm += vehicle.empty_dist_traveled_mm;
        total_rebl_dist_traveled_mm += vehicle.rebl_dist_traveled_mm;
        total_time_traveled_ms += vehicle.time_traveled_ms;
        total_loaded_time_traveled_ms += vehicle.loaded_time_traveled_ms;
        total_empty_time_traveled_ms += vehicle.empty_time_traveled_ms;
        total_rebl_time_traveled_ms += vehicle.rebl_time_traveled_ms;
    }

    auto avg_dist_traveled_km = total_dist_traveled_mm / 1000000.0 / vehicles_.size();
    auto avg_empty_dist_traveled_km = total_empty_dist_traveled_mm / 1000000.0 / vehicles_.size();
    auto avg_rebl_dist_traveled_km = total_rebl_dist_traveled_mm / 1000000.0 / vehicles_.size();
    auto avg_time_traveled_s = total_time_traveled_ms / 1000.0 / vehicles_.size();
    auto avg_empty_time_traveled_s = total_empty_time_traveled_ms / 1000.0 / vehicles_.size();
    auto avg_rebl_time_traveled_s = total_rebl_time_traveled_ms / 1000.0 / vehicles_.size();
    fmt::print("# Vehicles ({})\n", vehicles_.size());
    fmt::print("  - Service Distance: total_dist = {:.2f} km, avg_dist = {:.2f} km.\n",
               total_dist_traveled_mm / 1000000.0, avg_dist_traveled_km);
    fmt::print("  - Service Duration: avg_time = {:.2f} s ({:.2f}% of the main simulation time).\n",
               avg_time_traveled_s,
               100.0 * avg_time_traveled_s / platform_config_.simulation_config.simulation_duration_s);
    fmt::print("  - Empty Travel: avg_time = {:.2f} s ({:.2f}%), avg_dist = {:.2f} km ({:.2f}%).\n",
               avg_empty_time_traveled_s, 100.0 * avg_empty_time_traveled_s / avg_time_traveled_s,
               avg_empty_dist_traveled_km, 100.0 * avg_empty_dist_traveled_km / avg_dist_traveled_km);
    fmt::print("  - Rebl Travel: avg_time = {:.2f} s ({:.2f}%), avg_dist = {:.2f} km ({:.2f}%).\n",
               avg_rebl_time_traveled_s, 100.0 * avg_rebl_time_traveled_s / avg_time_traveled_s,
               avg_rebl_dist_traveled_km, 100.0 * avg_rebl_dist_traveled_km / avg_dist_traveled_km);
    fmt::print("  - Load: average_load_dist = {:.2f}, average_load_time = {:.2f}.\n",
               total_loaded_dist_traveled_mm * 1.0 / total_dist_traveled_mm,
               total_loaded_time_traveled_ms * 1.0 / total_time_traveled_ms);

    fmt::print("{}\n", dividing_line);
}
