//
// Created by Leot on 2021/4/20.
//

#pragma once

#include "rebalancing_npo.hpp"

#include <fmt/format.h>
#undef NDEBUG
#include <assert.h>

template <typename RouterFunc>
void RepositionIdleVehiclesToNearestPendingOrders(const std::vector<Order> &orders,
                                                std::vector<Vehicle> &vehicles,
                                                RouterFunc &router_func) {
    TIMER_START(t)

    // 1. Get a list of the unassigned orders.
    std::vector<size_t> pending_order_ids;
    for (const auto &order : orders) {
        if (order.status == OrderStatus::PENDING) { pending_order_ids.push_back(order.id); }
    }

    if (DEBUG_PRINT) {
        int num_of_idle_vehicles = 0;
        for (const auto &vehicle : vehicles) {
            if (vehicle.status == VehicleStatus::IDLE) { num_of_idle_vehicles++; }
        }
        fmt::print("        -Repositioning {} idle vehicles to {} locations through NPO...\n",
                   num_of_idle_vehicles, pending_order_ids.size());
    }

    // 2. Compute all rebalancing candidates.
    std::vector<std::pair<size_t, std::vector<Waypoint>>> rebalancing_candidates;
    for (const auto &vehicle : vehicles) {
        if (vehicle.status != VehicleStatus::IDLE) { continue; }
        for (auto order_id : pending_order_ids) {
//    for (auto order_id : pending_order_ids) {
//        for (const auto &vehicle : vehicles) {
//            if (vehicle.status != VehicleStatus::IDLE) { continue; }
            auto rebalancing_route = router_func(vehicle.pos, orders[order_id].origin, RoutingType::TIME_ONLY);
            std::vector<Waypoint> rebalancing_schedule =
                    {Waypoint{orders[order_id].origin, WaypointOp::REPOSITION,
                              orders[order_id].id, std::move(rebalancing_route)}};
            rebalancing_candidates.push_back({vehicle.id, rebalancing_schedule});
        }
    }

    // 3. Select suitable rebalancing candidates. Greedily from the one with the shortest travel time.
    std::sort(rebalancing_candidates.begin(), rebalancing_candidates.end(),
              [](std::pair<size_t, std::vector<Waypoint>> a, std::pair<size_t, std::vector<Waypoint>> b) {
        return a.second[0].route.duration_ms < b.second[0].route.duration_ms;
    });
    std::vector<size_t> selected_vehicle_ids;
    std::vector<size_t> selected_order_ids;
    for (auto &rebalancing_task : rebalancing_candidates) {
        // Check if the vehicle has been selected to do a rebalancing task.
        if (std::find(selected_vehicle_ids.begin(), selected_vehicle_ids.end(), rebalancing_task.first)
            != selected_vehicle_ids.end()) { continue; }
        // Check if the visiting point in the current rebalancing task has been visited.
        if (std::find(selected_order_ids.begin(), selected_order_ids.end(), rebalancing_task.second[0].order_id)
            != selected_order_ids.end()) { continue; }
        selected_vehicle_ids.push_back(rebalancing_task.first);
        selected_order_ids.push_back(rebalancing_task.second[0].order_id);
        // 4. Push the rebalancing task to the assigned vehicle.
        auto &rebalancing_vehicle = vehicles[rebalancing_task.first];
        auto &rebalancing_schedule = rebalancing_task.second;
        UpdVehicleScheduleAndBuildRoute(rebalancing_vehicle, rebalancing_schedule, router_func);

//        if (DEBUG_PRINT) {
//            fmt::print("            +Reposition Vehicle #{} to Pos {}, where Order #{} is located.\n",
//                       rebalancing_vehicle.id, rebalancing_schedule[0].pos.node_id, rebalancing_schedule[0].order_id);
//        }

    }

    if (DEBUG_PRINT) {
        fmt::print("            +Rebalancing vehicles: {}", selected_vehicle_ids.size());
        TIMER_END(t)
    }
}
