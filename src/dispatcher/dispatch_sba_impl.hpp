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

    // compute all possible vehicle order pairs, each indicating that the order can be served by the vehicle
    auto feasible_vehicle_order_pairs = BuildRvGraph(new_received_order_ids, orders, vehicles,
                                                     system_time_ms, router_func);
    std::sort(feasible_vehicle_order_pairs.begin(), feasible_vehicle_order_pairs.end(), SortVehicleTripPairsForGreedy);

//    fmt::print("(veh_id, trip_ids, cost_ms)\n");
//    for (const SchedulingResult &vo_pair : feasible_vehicle_order_pairs) {
//        Order &order = orders[vo_pair.trip_ids[0]];
//        fmt::print("  -({}, {}, {})\n", vo_pair.vehicle_id, vo_pair.trip_ids, vo_pair.best_schedule_cost_ms);
//    }

    // compute the assignment policy, indicating which vehicle to pick which order
    auto selected_vehicle_trip_pair_indices = GreedyAssignment(feasible_vehicle_order_pairs);
//    fmt::print("selected_vehicle_trip_pair_indices {}\n", selected_vehicle_trip_pair_indices);

    // update vehicles' schedules and assigned orders' statuses
    for (auto idx : selected_vehicle_trip_pair_indices) {
        auto &vo_pair = feasible_vehicle_order_pairs[idx];
        orders[vo_pair.trip_ids[0]].status = OrderStatus::PICKING;
        auto &vehicle = vehicles[vo_pair.vehicle_id];
        auto &schedule = vo_pair.feasible_schedules[vo_pair.best_schedule_idx];
        UpdaVehicleScheduleAndBuildRoute(vehicle, schedule, router_func);

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

//    exit(0);

}

template <typename RouterFunc>
std::vector<SchedulingResult> BuildRvGraph(const std::vector<size_t> &new_received_order_ids,
                                           const std::vector<Order> &orders,
                                           const std::vector<Vehicle> &vehicles,
                                           uint64_t system_time_ms,
                                           RouterFunc &router_func) {
    TIMER_START(t)
    if (DEBUG_PRINT) {
        fmt::print("                +Computing feasible vehicle order pairs...");
    }
    std::vector<SchedulingResult> feasible_vehicle_order_pairs;
    for (auto order_id : new_received_order_ids) {
        const auto &order = orders[order_id];
        assert(order.status == OrderStatus::PENDING);
        std::vector<size_t> trip;
        trip.push_back(order_id);
        for (const auto &vehicle: vehicles) {
            if (!PassQuickCheck(order, vehicle, system_time_ms, router_func)) { continue; }
            std::vector<std::vector<Waypoint>> sub_schedules;
            sub_schedules.push_back(vehicle.schedule);
            auto scheduling_result_this_pair = ComputeScheduleOfInsertingOrderToVehicle(
                    order, orders, vehicle, sub_schedules, system_time_ms, router_func);
            if (scheduling_result_this_pair.success) {
                scheduling_result_this_pair.trip_ids = trip;
                feasible_vehicle_order_pairs.push_back(std::move(scheduling_result_this_pair));
            }
        }
    }
    if (DEBUG_PRINT) {
        TIMER_END(t)
    }
    return feasible_vehicle_order_pairs;
}