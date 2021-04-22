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
#ifdef DEBUG_INFO_GLOBAL
    TIMER_START(t)
    if (DEBUG_PRINT) {
        fmt::print("        -Assigning orders to vehicles through GI...\n");
    }
#endif

    for (auto order_id : new_received_order_ids) {
        auto &order = orders[order_id];
        HeuristicInsertionOfOneOrder(order, orders, vehicles, system_time_ms, router_func);
    }

#ifdef DEBUG_INFO_GLOBAL
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
#endif
}


template <typename RouterFunc>
void HeuristicInsertionOfOneOrder(Order &order,
                                const std::vector<Order> &orders,
                                std::vector<Vehicle> &vehicles,
                                uint64_t system_time_ms,
                                RouterFunc &router_func) {

    SchedulingResult scheduling_result;
    // Iterate through all vehicles and find the one with least cost.
    for (const auto &vehicle : vehicles) {
//        if (vehicle.status == VehicleStatus::REBALANCING) { continue; }
        // filter out the vehicle that can not serve the order even when it is idle.
        if (router_func(vehicle.pos, order.origin, RoutingType::TIME_ONLY).duration_ms +
            vehicle.step_to_pos.duration_ms + system_time_ms > order.max_pickup_time_ms) { continue; }
        std::vector<std::vector<Waypoint>> sub_schedules = {vehicle.schedule};
        auto result_this_vehicle = ComputeScheduleOfInsertingOrderToVehicle(
                order, orders, vehicle, sub_schedules, system_time_ms, router_func);
        if (result_this_vehicle.best_schedule_cost_ms < scheduling_result.best_schedule_cost_ms) {
            scheduling_result = std::move(result_this_vehicle);
        }
    }

    // Insert the order to the best vehicle and update the vehicle's schedule.
    if (scheduling_result.success) {
        order.status = OrderStatus::PICKING;
        auto &best_vehicle = vehicles[scheduling_result.vehicle_id];
        auto &best_schedule = scheduling_result.feasible_schedules[scheduling_result.best_schedule_idx];
        UpdaVehicleScheduleAndBuildRoute(best_vehicle, best_schedule, router_func);

//#ifdef DEBUG_INFO_GLOBAL
//        fmt::print("            +Assigned Order #{} to Vehicle #{}, with a schedule has {} waypoints.\n",
//                   order.id,
//                   best_vehicle.id,
//                   best_vehicle.schedule.size());
//#endif
    }
}
