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

    // 2. Score the candidate vehicle_order_pairs.
    ScoreVtPairsWithNumOfOrdersAndIncreasedDelay(feasible_vehicle_order_pairs, orders, vehicles, system_time_ms);

    // 3. Compute the assignment policy based on the scores, indicating which vehicle to pick which order.
    auto selected_vehicle_order_pair_indices = IlpAssignment(feasible_vehicle_order_pairs,
                                                             new_received_order_ids, orders, vehicles);
//    auto selected_vehicle_order_pair_indices = GreedyAssignment(feasible_vehicle_order_pairs);

    // 4. Update the assigned vehicles' schedules and the assigned orders' statuses.
    UpdScheduleForVehiclesInSelectedVtPairs(feasible_vehicle_order_pairs, selected_vehicle_order_pair_indices,
                                            orders, vehicles, router_func);

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
        fmt::print("                *Computing feasible vehicle order pairs...");
    }
    std::vector<SchedulingResult> feasible_vehicle_order_pairs;

    // 1. Compute the feasible orders for each vehicle.
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

    // 2. Add the basic schedule of each vehicle, which denotes the "empty assign" option in ILP.
    for (const auto &vehicle: vehicles) {
        SchedulingResult basic_vo_pair;
        basic_vo_pair.success = true;
        basic_vo_pair.vehicle_id = vehicle.id;
        basic_vo_pair.feasible_schedules.push_back(vehicle.schedule);
        basic_vo_pair.best_schedule_idx = 0;
        basic_vo_pair.best_schedule_cost_ms = ComputeScheduleCost(vehicle.schedule, orders, vehicle, system_time_ms);
        feasible_vehicle_order_pairs.push_back(std::move(basic_vo_pair));
    }

    if (DEBUG_PRINT) {
        TIMER_END(t)
    }
    return feasible_vehicle_order_pairs;
}