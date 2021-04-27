//
// Created by Leot on 2021/4/22.
//

#pragma once

#include "dispatch_sba.hpp"

#include <fmt/format.h>
#undef NDEBUG
#include <assert.h>

template <typename RouterFunc>
void AssignOrdersThroughSingleRequestBatchAssign(const std::vector<size_t> &new_received_order_ids,
                                                 std::vector<Order> &orders,
                                                 std::vector<Vehicle> &vehicles,
                                                 uint64_t system_time_ms,
                                                 RouterFunc &router_func) {

    TIMER_START(t)
    if (DEBUG_PRINT) {
        fmt::print("        -Assigning {} orders to vehicles through SBA...\n", new_received_order_ids.size());
    }

    // 1. Compute all possible vehicle order pairs, each indicating that the order can be served by the vehicle.
    auto feasible_vehicle_order_pairs = ComputeFeasibleVehicleOrderPairs(new_received_order_ids, orders, vehicles,
                                                                         system_time_ms, router_func);

    // 2. Compute the assignment policy, indicating which vehicle to pick which order.
    std::sort(feasible_vehicle_order_pairs.begin(), feasible_vehicle_order_pairs.end(), SortVehicleTripPairs);
//    auto selected_vehicle_order_pair_indices = GreedyAssignment(feasible_vehicle_order_pairs);
    auto selected_vehicle_order_pair_indices = IlpAssignment(feasible_vehicle_order_pairs, new_received_order_ids,
                                                             orders, vehicles);

//    fmt::print("(veh_id, trip_ids, cost_s)\n");
//    for (const SchedulingResult &vo_pair : feasible_vehicle_order_pairs) {
//        Order &order = orders[vo_pair.trip_ids[0]];
//        fmt::print("  -({}, {}, {})\n", vo_pair.vehicle_id, vo_pair.trip_ids,
//                   vo_pair.best_schedule_cost_ms / 1000.0);
//    }
//    fmt::print("selected_vehicle_order_pair_indices {}\n", selected_vehicle_order_pair_indices);

    std::vector<size_t> rids_assigned;

    // 3. Update vehicles' schedules and assigned orders' statuses
    for (auto idx : selected_vehicle_order_pair_indices) {
        auto &vo_pair = feasible_vehicle_order_pairs[idx];
        if (vo_pair.trip_ids.size() == 0) { continue; }  // empty assign, no change to the vehicle's schedule
        orders[vo_pair.trip_ids[0]].status = OrderStatus::PICKING;
        rids_assigned.push_back(vo_pair.trip_ids[0]);
        auto &vehicle = vehicles[vo_pair.vehicle_id];
        auto &schedule = vo_pair.feasible_schedules[vo_pair.best_schedule_idx];
        UpdVehicleScheduleAndBuildRoute(vehicle, schedule, router_func);

//        if (DEBUG_PRINT) {
//            fmt::print("            +Assigned Order #{} to Vehicle #{}, with a schedule has {} waypoints.\n",
//                       vo_pair.trip_ids[0], vehicle.id, vehicle.schedule.size());
//        }
    }

    if (DEBUG_PRINT) {
        int num_of_assigned_orders = 0;
        for (auto order_id : new_received_order_ids) {
            if (orders[order_id].status == OrderStatus::PICKING) {
                num_of_assigned_orders++;
            }
        }
        fmt::print("            +Assigned orders: {}", num_of_assigned_orders);
        TIMER_END(t)
    }
}

template <typename RouterFunc>
std::vector<SchedulingResult> ComputeFeasibleVehicleOrderPairs(const std::vector<size_t> &new_received_order_ids,
                                                               const std::vector<Order> &orders,
                                                               const std::vector<Vehicle> &vehicles,
                                                               uint64_t system_time_ms,
                                                               RouterFunc &router_func) {
    TIMER_START(t)
    if (DEBUG_PRINT) {
        fmt::print("                +Computing feasible vehicle order pairs...");
    }
    std::vector<SchedulingResult> feasible_vehicle_order_pairs;

    // Compute feasible orders for each vehicle
    for (const auto &vehicle: vehicles) {
        std::vector<std::vector<Waypoint>> basic_schedules;
        basic_schedules.push_back(vehicle.schedule);
        auto feasible_vehicle_order_pairs_for_this_vehicle = ComputeSize1TripsForOneVehicle(new_received_order_ids,
                                                                                            orders,
                                                                                            vehicle,
                                                                                            basic_schedules,
                                                                                            system_time_ms,
                                                                                            router_func);
        feasible_vehicle_order_pairs.insert(feasible_vehicle_order_pairs.end(),
                                            feasible_vehicle_order_pairs_for_this_vehicle.begin(),
                                            feasible_vehicle_order_pairs_for_this_vehicle.end());
    }

    if (DEBUG_PRINT) {
        TIMER_END(t)
    }
    return feasible_vehicle_order_pairs;
}