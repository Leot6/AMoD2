//
// Created by Leot on 2021/4/16.
//

#pragma once

#include "dispatch_gi.hpp"

#include <fmt/format.h>
#undef NDEBUG
#include <assert.h>

template <typename RouterFunc>
void AssignOrdersThroughGreedyInsertion(const std::vector<size_t> &new_received_order_ids,
                                        std::vector<Order> &orders,
                                        std::vector<Vehicle> &vehicles,
                                        uint64_t system_time_ms,
                                        RouterFunc &router_func) {
    TIMER_START(t)
    if (DEBUG_PRINT) {
        fmt::print("        -Assigning {} orders to vehicles through GI...\n",
                   new_received_order_ids.size());
    }

    // Assigning new_received_orders in the first-in-first-out manner.
    for (auto order_id : new_received_order_ids) {
        auto &order = orders[order_id];
        HeuristicInsertionOfOneOrder(order, orders, vehicles, system_time_ms, router_func);
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
void HeuristicInsertionOfOneOrder(Order &order,
                                const std::vector<Order> &orders,
                                std::vector<Vehicle> &vehicles,
                                uint64_t system_time_ms,
                                RouterFunc &router_func) {

    SchedulingResult scheduling_result;
    // 1. Iterate through all vehicles and find the one with the least cost.
    for (const auto &vehicle : vehicles) {
        if (!PassQuickCheck(order, vehicle, system_time_ms, router_func)) { continue; }
        std::vector<std::vector<Waypoint>> basic_schedules;
        basic_schedules.push_back(vehicle.schedule);
        auto result_this_vehicle = ComputeScheduleOfInsertingOrderToVehicle(
                order, orders, vehicle, basic_schedules, system_time_ms, router_func);
        if (!result_this_vehicle.success) { continue; }
        // Compute the score as minus the increased schedule cost. The smaller the cost, the higher the score.
        result_this_vehicle.score = ComputeScheduleCost(vehicle.schedule, orders, vehicle, system_time_ms)
                                    - result_this_vehicle.best_schedule_cost_ms;
        assert(result_this_vehicle.score <= 0);
        if (result_this_vehicle.score > scheduling_result.score) {
            scheduling_result = std::move(result_this_vehicle);
        }
    }

    // 2. Insert the order to the best vehicle and update the vehicle's schedule.
    if (scheduling_result.success) {
        order.status = OrderStatus::PICKING;
        auto &best_vehicle = vehicles[scheduling_result.vehicle_id];
        auto &best_schedule = scheduling_result.feasible_schedules[scheduling_result.best_schedule_idx];
        UpdVehicleScheduleAndBuildRoute(best_vehicle, best_schedule, router_func);

//        if (DEBUG_PRINT) {
//            fmt::print("            +Assigned Order #{} to Vehicle #{}, with a schedule has {} waypoints.\n",
//                       order.id, best_vehicle.id, best_vehicle.schedule.size());
//        }

    }
}
