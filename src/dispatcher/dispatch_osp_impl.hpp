//
// Created by Leot on 2021/4/25.
//

#pragma once

#include "dispatch_osp.hpp"

#include <fmt/format.h>
#undef NDEBUG
#include <assert.h>
//#include ""


template <typename RouterFunc>
void AssignOrdersThroughOptimalSchedulePoolAssign(const std::vector<size_t> &new_received_order_ids,
                                                  std::vector<Order> &orders,
                                                  std::vector<Vehicle> &vehicles,
                                                  uint64_t system_time_ms,
                                                  RouterFunc &router_func) {
    TIMER_START(t)

    // 1. Get the list of considered orders, normally including all picking and pending orders. But when the traffic
    // is deterministic, an order that cannot be served at last epoch is theoretically not servable at the current
    // epoch either. Then the list of considered orders is consisting of all picking and new received orders.
    bool consider_previous_unassigned_orders = true;
    std::vector<size_t> considered_order_ids;
    for (auto &order: orders) {
        if (order.status == OrderStatus::PICKING) { considered_order_ids.push_back(order.id); }
        if (consider_previous_unassigned_orders) {
            if (order.status == OrderStatus::PENDING) { considered_order_ids.push_back(order.id); }
        }
    }
    if (!consider_previous_unassigned_orders) {
        considered_order_ids.insert(considered_order_ids.end(),
                                    new_received_order_ids.begin(), new_received_order_ids.end());
    }

    if (DEBUG_PRINT) {
        fmt::print("        -Assigning {} orders to vehicles through OSP...\n", considered_order_ids.size());
    }

    // 2. Compute all possible vehicle trip pairs, each indicating the orders in the trip can be served by the vehicle.
    auto feasible_vehicle_trip_pairs = ComputeFeasibleVehicleTripPairs(considered_order_ids, orders, vehicles,
                                                                         system_time_ms, router_func);

    // 3. Compute the assignment policy, indicating which vehicle to pick which trip.
    std::sort(feasible_vehicle_trip_pairs.begin(), feasible_vehicle_trip_pairs.end(), SortVehicleTripPairs);
    auto selected_vehicle_trip_pair_indices = IlpAssignment(feasible_vehicle_trip_pairs, considered_order_ids,
                                                            orders, vehicles);
//    auto selected_vehicle_trip_pair_indices = GreedyAssignment(feasible_vehicle_trip_pairs);

    // 4. Update vehicles' schedules and assigned orders' statuses
    for (auto order_id : considered_order_ids) { orders[order_id].status = OrderStatus::PENDING; }
    for (auto idx : selected_vehicle_trip_pair_indices) {
        auto &vt_pair = feasible_vehicle_trip_pairs[idx];
        for (auto order_id : vt_pair.trip_ids) { orders[order_id].status = OrderStatus::PICKING; }
        auto &vehicle = vehicles[vt_pair.vehicle_id];
        auto &schedule = vt_pair.feasible_schedules[vt_pair.best_schedule_idx];
        UpdVehicleScheduleAndBuildRoute(vehicle, schedule, router_func);
    }

//    fmt::print("(veh_id, trip_ids, cost_s)\n");
//    for (auto i = 0; i < feasible_vehicle_trip_pairs.size(); i++) {
//        auto &vt_pair = feasible_vehicle_trip_pairs[i];
//        fmt::print("  -({}, {}, {})", vt_pair.vehicle_id, vt_pair.trip_ids,
//                   vt_pair.best_schedule_cost_ms / 1000.0);
//        if (std::find(selected_vehicle_trip_pair_indices.begin(), selected_vehicle_trip_pair_indices.end(),
//                      i) != selected_vehicle_trip_pair_indices.end()) { fmt::print(" {}", i); }
//        fmt::print("\n");
//    }
//    fmt::print("selected_vehicle_order_pair_indices {}\n", selected_vehicle_trip_pair_indices);

    // 5. Update schedule of vehicles, the assigned (picking) orders of which are reassigned to other vehicles.
    auto num_vehicles = vehicles.size();
    try {
        for (auto i = 0; i < num_vehicles; i++) {
            fmt::print("[DEBUG] 1\n");
            auto &vehicle = vehicles[i];
            if (vehicle.schedule_is_updated_at_current_epoch) { continue; }
            if (vehicle.status == VehicleStatus::WORKING) {
                if (vehicle.schedule.size() == vehicle.load) { continue; }
                fmt::print("[DEBUG] 2\n");
                fmt::print("onboard rids {}\n", vehicle.onboard_order_ids);
                PrintSchedule(vehicle, vehicle.schedule);
                std::vector<Waypoint> basic_schedule;
                auto vehicle_schedule = vehicle.schedule;
                for (const auto &wp : vehicle_schedule) {
                    if (std::find(vehicle.onboard_order_ids.begin(), vehicle.onboard_order_ids.end(), wp.order_id)
                        != vehicle.onboard_order_ids.end()) { basic_schedule.push_back(wp); }
                }
                fmt::print("[DEBUG] 5\n");
                PrintSchedule(vehicle, basic_schedule);
                UpdVehicleScheduleAndBuildRoute(vehicle, basic_schedule, router_func);
                fmt::print("[DEBUG] 6\n");
            }
        }
    }
    catch (std::exception &e) {
        std::cout << " a standard exception was caught, with message '"
                  << e.what() << "'\n";
    }


    if (DEBUG_PRINT) {
        int num_of_assigned_orders = 0;
        for (auto order_id : considered_order_ids) {
            if (orders[order_id].status == OrderStatus::PICKING) {
                num_of_assigned_orders++;
            }
        }
        fmt::print("            +Assigned orders: {}", num_of_assigned_orders);
        TIMER_END(t)
    }
}

template <typename RouterFunc>
std::vector<SchedulingResult> ComputeFeasibleVehicleTripPairs(const std::vector<size_t> &considered_order_ids,
                                                              const std::vector<Order> &orders,
                                                              const std::vector<Vehicle> &vehicles,
                                                              uint64_t system_time_ms,
                                                              RouterFunc &router_func) {
    TIMER_START(t)
    if (DEBUG_PRINT) {
        fmt::print("                +Computing feasible vehicle trip pairs...\n");
    }
    std::vector<SchedulingResult> feasible_vehicle_trip_pairs;
    for (const auto &vehicle : vehicles) {
        auto feasible_trips_for_this_vehicle = ComputeFeasibleTripsForOneVehicle(considered_order_ids, orders, vehicle,
                                                                                 system_time_ms, router_func);
        feasible_vehicle_trip_pairs.insert(feasible_vehicle_trip_pairs.end(),
                                           feasible_trips_for_this_vehicle.begin(),
                                           feasible_trips_for_this_vehicle.end());
    }

    if (DEBUG_PRINT) {
        TIMER_END(t)
    }
    return feasible_vehicle_trip_pairs;
}

template <typename RouterFunc>
std::vector<SchedulingResult> ComputeFeasibleTripsForOneVehicle(const std::vector<size_t> &considered_order_ids,
                                                                const std::vector<Order> &orders,
                                                                const Vehicle &vehicle,
                                                                uint64_t system_time_ms,
                                                                RouterFunc &router_func) {

//    if (DEBUG_PRINT) {
//        fmt::print("                    +Computing feasible vehicle trip pairs for Vehicle #{}...\n",
//                   vehicle.id);
//    }

    std::vector<SchedulingResult> feasible_trips_for_this_vehicle;

    // Get the basic schedules of the vehicle.
    auto basic_schedules = GetBasicSchedulesOfVehicle(orders, vehicle, system_time_ms, router_func);

    // Compute trips of size 1.
    std::vector<SchedulingResult> feasible_trips_of_size_1 = ComputeSize1TripsForOneVehicle(considered_order_ids,
                                                                                            orders,
                                                                                            vehicle,
                                                                                            basic_schedules,
                                                                                            system_time_ms,
                                                                                            router_func);
    feasible_trips_for_this_vehicle.insert(feasible_trips_for_this_vehicle.end(),
                                           feasible_trips_of_size_1.begin(), feasible_trips_of_size_1.end());
    // Compute trips of size k (k > 1).
    std::vector<SchedulingResult> feasible_trips_of_size_k_minus_1 = feasible_trips_of_size_1;
    while(feasible_trips_of_size_k_minus_1.size() != 0) {
        auto feasible_trips_of_size_k = ComputeSizeKTripsForOneVehicle(considered_order_ids,
                                                                       feasible_trips_of_size_k_minus_1,
                                                                       orders, vehicle, system_time_ms, router_func);
        feasible_trips_for_this_vehicle.insert(feasible_trips_for_this_vehicle.end(),
                                               feasible_trips_of_size_k.begin(), feasible_trips_of_size_k.end());
        feasible_trips_of_size_k_minus_1 = feasible_trips_of_size_k;
    }

    return feasible_trips_for_this_vehicle;
}

template <typename RouterFunc>
std::vector<SchedulingResult> ComputeSize1TripsForOneVehicle(const std::vector<size_t> &considered_order_ids,
                                                             const std::vector<Order> &orders,
                                                             const Vehicle &vehicle,
                                                             const std::vector<std::vector<Waypoint>> &basic_schedules,
                                                             uint64_t system_time_ms,
                                                             RouterFunc &router_func) {
//    if (DEBUG_PRINT) {
//        fmt::print("                        +Computing size 1 trip for Vehicle #{}...",
//                   vehicle.id);
//    }
    std::vector<SchedulingResult> feasible_trips_of_size_1;
    for (auto order_id : considered_order_ids) {
        const auto &order = orders[order_id];
        if (!PassQuickCheck(order, vehicle, system_time_ms, router_func)) { continue; }
        auto scheduling_result_this_pair = ComputeScheduleOfInsertingOrderToVehicle(
                order, orders, vehicle, basic_schedules, system_time_ms, router_func);
        if (scheduling_result_this_pair.success) {
            std::vector<size_t> trip;
            trip.push_back(order_id);
            scheduling_result_this_pair.trip_ids = trip;
            feasible_trips_of_size_1.push_back(std::move(scheduling_result_this_pair));
        }
    }

//    if (DEBUG_PRINT) {
//        fmt::print("  ({} trips)\n", feasible_trips_of_size_1.size());
//    }
    return feasible_trips_of_size_1;
}

template <typename RouterFunc>
std::vector<SchedulingResult> ComputeSizeKTripsForOneVehicle(
        const std::vector<size_t> &considered_order_ids,
        const std::vector<SchedulingResult> &feasible_trips_of_size_k_minus_1,
        const std::vector<Order> &orders,
        const Vehicle &vehicle,
        uint64_t system_time_ms,
        RouterFunc &router_func) {

    std::vector<SchedulingResult> feasible_trips_of_size_k;
    auto k = feasible_trips_of_size_k_minus_1[0].trip_ids.size() + 1;
//    if (DEBUG_PRINT) {
//        fmt::print("                        +Computing size {} trip for Vehicle #{}...",
//                   k, vehicle.id);
//    }
    std::vector<std::vector<size_t>> searched_trip_ids_of_size_k;
    std::vector<std::vector<size_t>> feasible_trip_ids_of_size_k_minus_1;
    for (const auto &vt_pair : feasible_trips_of_size_k_minus_1) {
        feasible_trip_ids_of_size_k_minus_1.push_back(vt_pair.trip_ids);
    }

    for (auto i = 0; i < feasible_trips_of_size_k_minus_1.size() - 1; i++) {
        std::vector<size_t> trip1_ids = feasible_trips_of_size_k_minus_1[i].trip_ids;
        for (auto j = i + 1; j < feasible_trips_of_size_k_minus_1.size(); j++) {
            // Trip 2 will be extended to a size k trip, so it is denoted by trip2_k.
            std::vector<size_t> trip2_k_ids = feasible_trips_of_size_k_minus_1[j].trip_ids;
            assert(trip1_ids != trip2_k_ids);
            trip2_k_ids.insert(trip2_k_ids.end(), trip1_ids.begin(), trip1_ids.end());
            std::sort(trip2_k_ids.begin(), trip2_k_ids.end());
            trip2_k_ids.erase(std::unique(trip2_k_ids.begin(), trip2_k_ids.end()),trip2_k_ids.end());
            if (k > 2) {
                // Check if the new trip size is not k.
                if (trip2_k_ids.size() != k) { continue; }
                // Check if the trip has been already computed.
                if (std::find(searched_trip_ids_of_size_k.begin(), searched_trip_ids_of_size_k.end(), trip2_k_ids)
                    != searched_trip_ids_of_size_k.end()) { continue; }
                // Check if any sub-trip is not feasible.
                bool flag_at_least_one_subtrip_is_not_feasible = false;
                for (auto idx = 0; idx < k; idx++) {
                    auto sub_trip_ids = trip2_k_ids;
                    sub_trip_ids.erase(sub_trip_ids.begin() + idx);
                    if (std::find(feasible_trip_ids_of_size_k_minus_1.begin(),
                                  feasible_trip_ids_of_size_k_minus_1.end(),
                                  sub_trip_ids) == feasible_trip_ids_of_size_k_minus_1.end()) {
                        flag_at_least_one_subtrip_is_not_feasible = true;
                        break;
                    }
                }
                if (flag_at_least_one_subtrip_is_not_feasible) { continue; }
            }
            // The schedules of the new trip is computed as inserting an order into vehicle's schedules of serving trip1.
            const auto &sub_schedules = feasible_trips_of_size_k_minus_1[i].feasible_schedules;
            std::vector<size_t> insertion_order_ids;
            std::set_difference(trip2_k_ids.begin(), trip2_k_ids.end(), trip1_ids.begin(), trip1_ids.end(),
                                std::back_inserter(insertion_order_ids));
            assert(insertion_order_ids.size() == 1);
            const auto &order = orders[insertion_order_ids[0]];
            auto scheduling_result_this_pair = ComputeScheduleOfInsertingOrderToVehicle(
                    order, orders, vehicle, sub_schedules, system_time_ms, router_func);
            if (scheduling_result_this_pair.success) {
                scheduling_result_this_pair.trip_ids = trip2_k_ids;
                feasible_trips_of_size_k.push_back(std::move(scheduling_result_this_pair));
                searched_trip_ids_of_size_k.push_back(trip2_k_ids);
                assert(std::includes(considered_order_ids.begin(), considered_order_ids.end(),
                                     trip2_k_ids.begin(), trip2_k_ids.end()) &&
                       "trip2_k_ids should be a subset of considered_order_ids !");
            }
        }
    }

//    if (DEBUG_PRINT) {
//        fmt::print("  ({} trips)\n", feasible_trips_of_size_k.size());
//    }
    return feasible_trips_of_size_k;
}

template <typename RouterFunc>
std::vector<std::vector<Waypoint>> GetBasicSchedulesOfVehicle(const std::vector<Order> &orders,
                                                              const Vehicle &vehicle,
                                                              uint64_t system_time_ms,
                                                              RouterFunc &router_func) {
    std::vector<std::vector<Waypoint>> basic_schedules;
    // If the vehicle is rebalancing, just return its current full schedule to ensure its rebalancing task.
    // If the vehicle is idle, then the basic schedule is an empty schedule.
    if (vehicle.status == VehicleStatus::REBALANCING || vehicle.status == VehicleStatus::IDLE) {
        basic_schedules.push_back(vehicle.schedule);
        return basic_schedules;
    }

    // If the vehicle is walking, return the sub-schedule only including the drop off tasks.
    std::vector<Waypoint> basic_schedule;
    for (const auto &wp : vehicle.schedule) {
        if (std::find(vehicle.onboard_order_ids.begin(), vehicle.onboard_order_ids.end(), wp.order_id)
        != vehicle.onboard_order_ids.end()) { basic_schedule.push_back(wp); }
    }
    assert(basic_schedule.size() == vehicle.load);
    basic_schedules.push_back(basic_schedule);

    // Consider permutations of basic_schedule to make sure we search all possible schedules later.
    std::vector<int> wp_indices(basic_schedule.size());
    std::iota(begin(wp_indices), end(wp_indices), 0);
    while (std::next_permutation(wp_indices.begin(), wp_indices.end())) {
        // Build new basic schedule.
        std::vector<Waypoint> new_basic_schedule;
        auto pre_pos = vehicle.pos;
        for (auto wp_idx : wp_indices) {
            auto wp = basic_schedule[wp_idx];
            auto route = router_func(pre_pos, wp.pos, RoutingType::TIME_ONLY);
            wp.route = std::move(route);
            new_basic_schedule.push_back(std::move(wp));
            pre_pos = wp.pos;
        }
        // Terms "0, 0, orders[0]" here are useless, they are served as default values for the following function.
        auto [feasible_this_schedule, violation_type] = ValidateSchedule(
                new_basic_schedule, 0, 0, orders[0], orders, vehicle, system_time_ms, router_func);
        if (feasible_this_schedule) { basic_schedules.push_back(new_basic_schedule); }
    }

    assert(basic_schedules.size() > 0);
    return basic_schedules;
}