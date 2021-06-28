//
// Created by Leot on 2021/4/20.
//

#pragma once

#include "rebalancing_rvs.hpp"

#include <fmt/format.h>
#undef NDEBUG
#include <assert.h>

template <typename RouterFunc>
void RepositionIdleVehicleToRandomVehicleStation(std::vector<Vehicle> &vehicles, RouterFunc &router_func) {
    TIMER_START(t)
    int num_of_stations = router_func.getNumOfVehicleStations();
    if (DEBUG_PRINT) {
        int num_of_idle_vehicles = 0;
        for (const auto &vehicle : vehicles) {
            if (vehicle.status == VehicleStatus::IDLE) { num_of_idle_vehicles++; }
        }
        fmt::print("        -Repositioning {} idle vehicles to {} stations through RVS...\n",
                   num_of_idle_vehicles, num_of_stations);
    }

    int num_of_rebalancing_vehicles = 0;
    for (auto &vehicle : vehicles) {
        if (vehicle.status != VehicleStatus::IDLE) { continue; }
        int rebalancing_station_idx = (rand() % num_of_stations);
        auto rebalancing_pos = router_func.getNodePos(router_func.getVehicleStationId(rebalancing_station_idx));
        if (vehicle.pos.node_id == rebalancing_pos.node_id) { continue; }
        auto rebalancing_route = router_func(vehicle.pos, rebalancing_pos, RoutingType::TIME_ONLY);
        std::vector<Waypoint> rebalancing_schedule =
                {Waypoint{rebalancing_pos, WaypointOp::REPOSITION, 0, std::move(rebalancing_route)}};
        UpdVehicleScheduleAndBuildRoute(vehicle, rebalancing_schedule, router_func);
        num_of_rebalancing_vehicles++;
    }

    if (DEBUG_PRINT) {
        fmt::print("            +Rebalancing vehicles: {}", num_of_rebalancing_vehicles);
        TIMER_END(t)
    }
}
