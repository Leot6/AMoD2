/// \author Jian Wen
/// \date 2021/02/08

#include "vehicle.hpp"

#include <fmt/format.h>

#undef NDEBUG
#include <assert.h>

void TruncateStepByTime(Step &step, uint64_t time_ms) {
//    fmt::print("Input step, t={}, d={}, time_ms={}\n", step.duration_ms, step.distance_mm, time_ms);
//    fmt::print("step.poses.size({}), poses[0]({} {} {}), poses[1]({} {} {})\n", step.poses.size(),
//               step.poses[0].node_id, step.poses[0].lon, step.poses[0].lat,
//               step.poses[1].node_id, step.poses[1].lon, step.poses[1].lat);

    assert(step.poses.size() == 2 &&
           "Input step in truncate_step_by_time() should have 2 poses!");
    assert(step.distance_mm > 0 &&
           "Input step's distance in truncate_step_by_time() must be positive!");
    assert(step.duration_ms > 0 &&
           "Input step's duration in truncate_step_by_time() must be positive!");
    assert(time_ms >= 0 && "Time in truncate_step_by_time() must be non negative!");
    assert(time_ms < step.duration_ms && "Ratio in truncate_step_by_time() must be within [0, 1)!");

    // Early return.
    if (time_ms == 0) {
        return;
    }

    auto ratio = static_cast<double>(time_ms) / step.duration_ms;
    // When the vehicle is travelling on the link from point A (step.poses[0]) to point B (step.poses[1]),
    // it can be treated as it were at point B to do route planning, considering the time left to arrive point B.
    step.poses[0].node_id = step.poses[1].node_id;
    step.poses[0].lon = step.poses[0].lon + ratio * (step.poses[1].lon - step.poses[0].lon);
    step.poses[0].lat = step.poses[0].lat + ratio * (step.poses[1].lat - step.poses[0].lat);
    step.distance_mm *= (1 - ratio);
    step.duration_ms -= time_ms;  // we do not use "*= (1 - ratio)" to avoid bug cases, e.g. "11119 / 11120 = 1.0"

    assert(step.poses.size() == 2 &&
           "Output step in truncate_step_by_time() should have 2 poses!");
    // normally distance_mm should be larger than 0,
    // but sometimes the distance_mm could be less than 1 and converted to 0, e.g. 370 * (1-4990/5000) = 0.74 = 0 (int)
    assert(step.distance_mm >= 0 &&
           "Output step's distance in truncate_step_by_time() must be positive!");
    assert(step.duration_ms > 0 &&
           "Output step's duration in truncate_step_by_time() must be positive!");
}

void TruncateRouteByTime(Route &route, uint64_t time_ms) {
    assert(route.steps.size() >= 2 &&
           "Input route in truncate_route_by_time() must have at least 2 steps!");
    assert(route.distance_mm > 0 &&
           "Input route's distance in truncate_route_by_time() must be positive!");
    assert(route.duration_ms > 0 &&
           "Input route's duration in truncate_route_by_time() must be positive!");
    assert(time_ms >= 0 && "Time in truncate_route_by_time() must be non negative!");
    assert(time_ms < route.duration_ms && "Time in truncate_route_by_time() must be less than route's duration!");

    // Early return.
    if (time_ms == 0) {
        return;
    }

    for (auto i = 0; i < route.steps.size(); i++) {
        auto &step = route.steps[i];

        // If we can finish this step within the time, remove the entire step.
        if (step.duration_ms <= time_ms) {
            time_ms -= step.duration_ms;
            continue;
        }
        // If we can not finish this step, truncate the step.
        TruncateStepByTime(step, time_ms);
        route.steps.erase(route.steps.begin(), route.steps.begin() + i);

        break;
    }

    // Recalculate the total duration and distance.
    route.distance_mm = 0;
    route.duration_ms = 0;
    for (const auto &step : route.steps) {
        route.distance_mm += step.distance_mm;
        route.duration_ms += step.duration_ms;
    }

    assert(route.steps.size() >= 2 &&
           "Output route in truncate_route_by_time() must have at least 2 steps!");
    // normally distance_mm should be larger than 0,
    // but sometimes the distance_mm could be less than 1 and converted to 0, e.g. 370 * (1-4990/5000) = 0.74 = 0 (int)
    assert(route.distance_mm >= 0 &&
           "Output route's distance in truncate_route_by_time() must be positive!");
    assert(route.duration_ms > 0 &&
           "Output route's duration in truncate_route_by_time() must be positive!");
}

std::pair<std::vector<size_t>, std::vector<size_t>> UpdVehiclePos(Vehicle &vehicle,
                                                                  std::vector<Order> &orders,
                                                                  uint64_t system_time_ms,
                                                                  uint64_t time_ms,
                                                                  bool update_vehicle_statistics) {
    std::vector<size_t> new_picked_order_ids = {};
    std::vector<size_t> new_dropped_order_ids = {};

    // Early return.
    if (time_ms == 0) {
        return {new_picked_order_ids, new_dropped_order_ids};
    }

    // Move the vehicle's pos by step_to_pos, if it is not empty while the vehicle's schedule is empty.
    // (This case is raised when the vehicle's assigned orders are reassigned to other vehicles and it becomes idle.)
    if (vehicle.status == VehicleStatus::IDLE) {
        if (vehicle.step_to_pos.duration_ms <= time_ms) {
            if (update_vehicle_statistics) {
                vehicle.dist_traveled_mm += vehicle.step_to_pos.distance_mm;
                vehicle.time_traveled_ms += vehicle.step_to_pos.duration_ms;
                vehicle.empty_dist_traveled_mm += vehicle.step_to_pos.distance_mm;
                vehicle.empty_time_traveled_ms += vehicle.step_to_pos.duration_ms;
            }
            Step empty_step;
            vehicle.step_to_pos = empty_step;
        }
        // If we can not finish this step_to_pos, truncate the step.
        if (vehicle.step_to_pos.duration_ms > time_ms) {
            const auto original_distance_mm = vehicle.step_to_pos.distance_mm;
            TruncateStepByTime(vehicle.step_to_pos, time_ms);
            const auto dist_traveled_mm = original_distance_mm - vehicle.step_to_pos.distance_mm;
            if (update_vehicle_statistics) {
                vehicle.dist_traveled_mm += dist_traveled_mm;
                vehicle.time_traveled_ms += time_ms;
                vehicle.empty_dist_traveled_mm += dist_traveled_mm;
                vehicle.empty_time_traveled_ms += time_ms;
            }
        }
        return {new_picked_order_ids, new_dropped_order_ids};
    }

    // Clear vehicle's step_to_pos to be prepared for the case when vehicle's pos is at a waypoint node.
    Step empty_step;
    vehicle.step_to_pos = empty_step;

    // Move the vehicle's pos by the schedule.
    for (auto i = 0; i < vehicle.schedule.size(); i++) {
        auto &wp = vehicle.schedule[i];

        // If we can finish this waypoint within the time.
        if (wp.route.duration_ms <= time_ms) {
            system_time_ms += wp.route.duration_ms;
            time_ms -= wp.route.duration_ms;

            vehicle.pos = wp.pos;

            if (update_vehicle_statistics) {
                vehicle.dist_traveled_mm += wp.route.distance_mm;
                vehicle.loaded_dist_traveled_mm += wp.route.distance_mm * vehicle.load;
                vehicle.time_traveled_ms += wp.route.duration_ms;
                vehicle.loaded_time_traveled_ms += wp.route.duration_ms * vehicle.load;
                if (vehicle.status == VehicleStatus::WORKING && vehicle.load == 0) {
                    vehicle.empty_dist_traveled_mm += wp.route.distance_mm;
                    vehicle.empty_time_traveled_ms += wp.route.duration_ms;
                }
                if (vehicle.status == VehicleStatus::REBALANCING) {
                    vehicle.rebl_dist_traveled_mm += wp.route.distance_mm;
                    vehicle.rebl_time_traveled_ms += wp.route.duration_ms;
                }
            }

            if (wp.op == WaypointOp::PICKUP) {
                assert(vehicle.load < vehicle.capacity &&
                       "Vehicle's load should be less than its capacity before a pickup!");
                assert(orders[wp.order_id].status == OrderStatus::PICKING);
                orders[wp.order_id].pickup_time_ms = system_time_ms;
                orders[wp.order_id].status = OrderStatus::ONBOARD;
                vehicle.load++;
                vehicle.onboard_order_ids.push_back(wp.order_id);
                new_picked_order_ids.push_back(wp.order_id);

//                if (DEBUG_PRINT){
//                    fmt::print("            +Vehicle #{} picked up Order #{} at {}s\n",
//                               vehicle.id, wp.order_id, system_time_ms / 1000.0);
//                }

            } else if (wp.op == WaypointOp::DROPOFF) {
                assert(vehicle.load > 0 && "Vehicle's load should not be zero before a dropoff!");
                assert(orders[wp.order_id].status == OrderStatus::ONBOARD);
                orders[wp.order_id].dropoff_time_ms = system_time_ms;
                orders[wp.order_id].status = OrderStatus::COMPLETE;
                vehicle.load--;
                for (auto i = 0; i < vehicle.onboard_order_ids.size(); i++) {
                    if (vehicle.onboard_order_ids[i] == wp.order_id) {
                        vehicle.onboard_order_ids.erase(vehicle.onboard_order_ids.begin() + i);
                        break;
                    }
                }
                new_dropped_order_ids.push_back(wp.order_id);

//                if (DEBUG_PRINT){
//                    fmt::print("            +Vehicle #{} dropped off Order #{} at {}s\n",
//                               vehicle.id, wp.order_id, system_time_ms / 1000.0);
//                }

            }
            assert(vehicle.load == vehicle.onboard_order_ids.size());
            continue;
        }

        // If we can not finish this waypoint, truncate the route.
        const auto original_distance_mm = wp.route.distance_mm;
        const auto original_duration_ms = wp.route.duration_ms;

        TruncateRouteByTime(wp.route, time_ms);
        vehicle.pos = wp.route.steps.front().poses.front();

        if (update_vehicle_statistics) {
            const auto dist_traveled_mm = original_distance_mm - wp.route.distance_mm;
            const auto time_traveled_ms = original_duration_ms - wp.route.duration_ms;
            vehicle.dist_traveled_mm += dist_traveled_mm;
            vehicle.loaded_dist_traveled_mm += dist_traveled_mm * vehicle.load;
            vehicle.time_traveled_ms += time_traveled_ms;
            vehicle.loaded_time_traveled_ms += time_traveled_ms * vehicle.load;
            if (vehicle.status == VehicleStatus::WORKING && vehicle.load == 0) {
                vehicle.empty_dist_traveled_mm += dist_traveled_mm;
                vehicle.empty_time_traveled_ms += time_traveled_ms;
            }
            if (vehicle.status == VehicleStatus::REBALANCING) {
                vehicle.rebl_dist_traveled_mm += dist_traveled_mm;
                vehicle.rebl_time_traveled_ms += time_traveled_ms;
            }
        }

        vehicle.schedule.erase(vehicle.schedule.begin(), vehicle.schedule.begin() + i);

        // If the vehicle is currently on a link, we store its unfinished step to step_to_pos.
        auto first_step_of_route = vehicle.schedule[0].route.steps[0];
        if (first_step_of_route.poses[0].node_id == first_step_of_route.poses[1].node_id) {
            vehicle.step_to_pos = first_step_of_route;
            assert (first_step_of_route.duration_ms != 0);
            assert (vehicle.pos.node_id == vehicle.step_to_pos.poses[0].node_id);
        }
        return {new_picked_order_ids, new_dropped_order_ids};
    }

    // We've finished the whole schedule.
    vehicle.schedule.clear();
    vehicle.status = VehicleStatus::IDLE;
    return {new_picked_order_ids, new_dropped_order_ids};
}
