//
// Created by Leot on 2021/4/23.
//

#include "scheduling.hpp"

uint32_t ComputeScheduleCost(const std::vector<Waypoint> &schedule,
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

void ScoreVtPairWithIncreasedDelay(SchedulingResult &vehicle_trip_pair,
                                   const std::vector<Order> &orders,
                                   const std::vector<Vehicle> &vehicles,
                                   uint64_t system_time_ms,
                                   bool is_reoptimization) {
    auto &vehicle = vehicles[vehicle_trip_pair.vehicle_id];
    vehicle_trip_pair.score = ComputeScheduleCost(vehicle.schedule, orders, vehicle, system_time_ms)
                              - vehicle_trip_pair.best_schedule_cost_ms;
    if (!is_reoptimization) {
        assert(vehicle_trip_pair.score <= 0);
    } else {
        if (vehicle_trip_pair.trip_ids.empty()) {
            int deviation_due_to_data_structure = 5;
            assert(vehicle_trip_pair.score + deviation_due_to_data_structure * 10 >= 0);
        }
    }
}

void ScoreVtPairsWithNumOfOrdersAndIncreasedDelay(std::vector<SchedulingResult> &vehicle_trip_pairs,
                                                 const std::vector<Order> &orders,
                                                 const std::vector<Vehicle> &vehicles,
                                                 uint64_t system_time_ms,
                                                 bool is_reoptimization) {
    // 1. Score the vt_pairs with the increased delay cause by inserting new orders.
    for (auto &vt_pair : vehicle_trip_pairs) {
        ScoreVtPairWithIncreasedDelay(vt_pair, orders, vehicles, system_time_ms, is_reoptimization);
    }

    // 2. Get the coefficients for NumOfOrders and IncreasedDelay.
    int32_t max_score_abs = 1;
    for (const auto &vt_pair :  vehicle_trip_pairs) {
        if (abs(vt_pair.score) > max_score_abs) { max_score_abs = abs(vt_pair.score); }
    }
    int num_length = 1;
    while ( max_score_abs /= 10 ) { num_length++; }
    int reward_for_serving_an_order = pow(10, num_length);

    // 3. Re-score the vt_pairs with NumOfOrders and IncreasedDelay.
    for (auto &vt_pair : vehicle_trip_pairs) {
        vt_pair.score = reward_for_serving_an_order * vt_pair.trip_ids.size() + vt_pair.score / 1e3;
    }

}
