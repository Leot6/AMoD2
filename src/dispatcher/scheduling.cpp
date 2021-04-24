//
// Created by Leot on 2021/4/23.
//

#include "scheduling.hpp"

uint64_t ComputeScheduleCost(const std::vector<Waypoint> &schedule,
                                    const std::vector<Order> &orders,
                                    const Vehicle &vehicle,
                                    uint64_t system_time_ms) {
    auto cost_pickup_ms = 0;
    auto cost_dropoff_ms = 0;
    auto accumulated_time_ms = vehicle.step_to_pos.duration_ms;

    for (const auto &wp : schedule) {
        accumulated_time_ms += wp.route.duration_ms;
    }
    return accumulated_time_ms;

//    for (const auto &wp : schedule) {
//        accumulated_time_ms += wp.route.duration_ms;
//        if (wp.op == WaypointOp::PICKUP) {
//            cost_pickup_ms += system_time_ms + accumulated_time_ms - orders[wp.order_id].request_time_ms;
//            assert(system_time_ms + accumulated_time_ms - orders[wp.order_id].request_time_ms >= 0);
//        }
//        if (wp.op == WaypointOp::DROPOFF) {
//            cost_dropoff_ms += system_time_ms + accumulated_time_ms -
//                               (orders[wp.order_id].request_time_ms + orders[wp.order_id].shortest_travel_time_ms);
//            assert(system_time_ms + accumulated_time_ms -
//                   (orders[wp.order_id].request_time_ms + orders[wp.order_id].shortest_travel_time_ms) >= 0);
//        }
//    }
//    auto cost_ms = cost_pickup_ms + cost_dropoff_ms;
//    return cost_ms;
}