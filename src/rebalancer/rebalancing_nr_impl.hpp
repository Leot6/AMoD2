//
// Created by Leot on 2021/4/20.
//

#pragma once

#include "rebalancing_nr.hpp"

#include <fmt/format.h>
#undef NDEBUG
#include <assert.h>

template <typename RouterFunc>
void RepositionIdleVehicleThroughNaiveRebalancer(const std::vector<size_t> &new_received_order_ids,
                                                 const std::vector<Order> &orders,
                                                 std::vector<Vehicle> &vehicles,
                                                 RouterFunc &router_func) {
#ifdef DEBUG_INFO_GLOBAL
    TIMER_START(t)
#endif

    // Get a list of the unassigned orders.
    std::vector<size_t> unassigned_new_received_order_ids;
    for (const auto &order_id : new_received_order_ids) {
        if (orders[order_id].status == OrderStatus::PENDING) { unassigned_new_received_order_ids.push_back(order_id); }
    }

#ifdef DEBUG_INFO_GLOBAL
    if (DEBUG_PRINT) {
        int num_of_idle_vehicles = 0;
        for (const auto &vehicle : vehicles) {
            if (vehicle.status == VehicleStatus::IDLE) { num_of_idle_vehicles++; }
        }

        fmt::print("        -Repositioning {} idle vehicles to {} positions through NR...\n",
                   num_of_idle_vehicles, unassigned_new_received_order_ids.size());
    }
#endif

    // Compute all rebalancing candidates.
    std::vector<std::pair<size_t, std::vector<Waypoint>>> rebalancing_candidates;
    for (auto order_id : unassigned_new_received_order_ids) {
        for (const auto &vehicle : vehicles) {
            if (vehicle.status != VehicleStatus::IDLE) { continue; }
            auto rebalancing_route = router_func(vehicle.pos, orders[order_id].origin, RoutingType::TIME_ONLY);
            std::vector<Waypoint> rebalancing_schedule = {Waypoint{orders[order_id].origin, WaypointOp::REPOSITION,
                                                                  orders[order_id].id, std::move(rebalancing_route)}};
            rebalancing_candidates.push_back({vehicle.id, rebalancing_schedule});
        }
    }


    // Select suitable rebalancing candidates. Greedily from the one with the shortest travel time.
    int num_of_new_rebalancing_vehicles = 0;
    std::sort(rebalancing_candidates.begin(), rebalancing_candidates.end(),
              [](std::pair<size_t, std::vector<Waypoint>> a, std::pair<size_t, std::vector<Waypoint>> b) {
        return a.second[0].route.duration_ms < b.second[0].route.duration_ms;
    });

    std::vector<size_t> selected_vehicle_ids;
    std::vector<size_t> selected_order_ids;
    for (auto &rebalancing_task : rebalancing_candidates) {
        // check if the vehicle has been selected to do a rebalancing task
        if (std::find(selected_vehicle_ids.begin(), selected_vehicle_ids.end(), rebalancing_task.first)
            != selected_vehicle_ids.end()) { continue; }
        // check if the visiting point in the current rebalancing task has been visited
        if (std::find(selected_order_ids.begin(), selected_order_ids.end(), rebalancing_task.second[0].order_id)
            != selected_order_ids.end()) { continue; }
        selected_vehicle_ids.push_back(rebalancing_task.first);
        selected_order_ids.push_back(rebalancing_task.second[0].order_id);
        num_of_new_rebalancing_vehicles++;
        auto &rebalancing_vehicle = vehicles[rebalancing_task.first];
        auto &rebalancing_schedule = rebalancing_task.second;
        UpdaVehicleScheduleAndBuildRoute(rebalancing_vehicle, rebalancing_schedule, router_func);

//#ifdef DEBUG_INFO_GLOBAL
//        fmt::print("            +Reposition Vehicle #{} to Pos {}, where Order #{} is located.\n",
//                   rebalancing_vehicle.id, rebalancing_schedule[0].pos.node_id, rebalancing_schedule[0].order_id);
//#endif

    }

#ifdef DEBUG_INFO_GLOBAL
    if (DEBUG_PRINT) {
        fmt::print("            +Rebalancing vehicles: {}", num_of_new_rebalancing_vehicles);
        TIMER_END(t)
    }
#endif
}
