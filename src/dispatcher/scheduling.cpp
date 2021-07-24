//
// Created by Leot on 2021/4/23.
//

#include "scheduling.hpp"

uint64_t ComputeScheduleCost(const std::vector<Waypoint> &schedule,
                             const std::vector<Order> &orders,
                             const Vehicle &vehicle,
                             uint64_t system_time_ms) {
    if (schedule.empty()) { return 0; }

    auto accumulated_time_ms = vehicle.step_to_pos.duration_ms;
    auto cost_pickup_delay_ms = 0;
    auto cost_total_delay_ms = 0;

    // Set the initial value of accumulated_time_ms as 0, when computing the cost of the vehicle's current working
        // schedule. (Because when the schedule was updated to the vehicle, the vehicle's step_to_pos has been added to
        // the build route. If "accumulated_time_ms = vehicle.step_to_pos.duration_ms", the vehicle's step_to_pos will
        // be counted twice. The difference between a new generated schedule and a vehicle's working schedule is
        // whether the vector term "steps" of route is empty.)
    auto &first_route = schedule[0].route;
    if (!first_route.steps.empty() && accumulated_time_ms != 0) {
        accumulated_time_ms = 0;
        auto &first_step = first_route.steps[0];
        assert(first_step.poses[0].node_id == vehicle.pos.node_id);
        assert(first_step.poses[0].node_id == first_step.poses[1].node_id);
        assert(first_step.duration_ms == vehicle.step_to_pos.duration_ms);
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