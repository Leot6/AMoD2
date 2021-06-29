//
// Created by Leot on 2021/4/23.
//

#include "scheduling.hpp"

uint64_t ComputeScheduleCost(const std::vector<Waypoint> &schedule,
                             const std::vector<Order> &orders,
                             const Vehicle &vehicle,
                             uint64_t system_time_ms) {
    auto accumulated_time_ms = vehicle.step_to_pos.duration_ms;
    auto cost_pickup_delay_ms = 0;
    auto cost_total_delay_ms = 0;

    for (const auto &wp : schedule) {
        accumulated_time_ms += wp.route.duration_ms;
    }

    for (const auto &wp : schedule) {
        accumulated_time_ms += wp.route.duration_ms;
        if (wp.op == WaypointOp::PICKUP) {
            cost_pickup_delay_ms += system_time_ms + accumulated_time_ms - orders[wp.order_id].request_time_ms;
            assert(system_time_ms + accumulated_time_ms - orders[wp.order_id].request_time_ms >= 0);
        }
        if (wp.op == WaypointOp::DROPOFF) {
            cost_total_delay_ms += system_time_ms + accumulated_time_ms -
                               (orders[wp.order_id].request_time_ms + orders[wp.order_id].shortest_travel_time_ms);
            assert(system_time_ms + accumulated_time_ms -
                   (orders[wp.order_id].request_time_ms + orders[wp.order_id].shortest_travel_time_ms) >= 0);
        }
    }
    return cost_total_delay_ms;
}