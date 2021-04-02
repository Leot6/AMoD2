/// \author Jian Wen
/// \date 2021/02/08

#include "vehicle.hpp"

#include <fmt/format.h>

void truncate_step_by_time(Step &step, uint64_t time_ms) {
    assert(step.poses.size() >= 2 &&
           "Input step in truncate_step_by_time() must have at least 2 poses!");
    assert(step.distance_mm > 0 &&
           "Input step's distance in truncate_step_by_time() must be positive!");
    assert(step.duration_ms > 0 &&
           "Input step's duration in truncate_step_by_time() must be positive!");
    assert(time_ms < step.duration_ms && "Ratio in truncate_step_by_time() must be within [0, 1)!");

    // Early return.
    if (time_ms == step.duration_ms) {
        return;
    }

    auto ratio = static_cast<double>(time_ms) / step.duration_ms;

    // Get the total distance of the step. We use Mahhantan distance for simplicity.
    auto total_dist = 0.0;
    for (auto i = 0; i < step.poses.size() - 1; i++) {
        total_dist += abs(step.poses[i].lat - step.poses[i + 1].lat) +
                      abs(step.poses[i].lon - step.poses[i + 1].lon);
    }

    // Compute the distance to be truncated.
    const auto truncated_dist = total_dist * ratio;

    // Iterate through the poses for the target distance.
    auto accumulated_dist = 0.0;
    for (auto i = 0; i < step.poses.size() - 1; i++) {
        auto dist = abs(step.poses[i].lat - step.poses[i + 1].lat) +
                    abs(step.poses[i].lon - step.poses[i + 1].lon);

        if (accumulated_dist + dist > truncated_dist) {
            auto subratio = (truncated_dist - accumulated_dist) / dist;

            assert(subratio >= 0 && subratio < 1 &&
                   "Ratio in truncate_step_by_time() must be within [0, 1)!");

            step.poses[i].lon =
                step.poses[i].lon + subratio * (step.poses[i + 1].lon - step.poses[i].lon);
            step.poses[i].lat =
                step.poses[i].lat + subratio * (step.poses[i + 1].lat - step.poses[i].lat);

            step.poses.erase(step.poses.begin(), step.poses.begin() + i);

            break;
        }

        accumulated_dist += dist;
    }

    step.distance_mm *= (1 - ratio);
    step.duration_ms *= (1 - ratio);

    assert(step.poses.size() >= 2 &&
           "Output step in truncate_step_by_time() must have at least 2 poses!");
    assert(step.distance_mm > 0 &&
           "Output step's distance in truncate_step_by_time() must be positive!");
    assert(step.duration_ms > 0 &&
           "Output step's duration in truncate_step_by_time() must be positive!");
}

void truncate_leg_by_time(Leg &leg, uint64_t time_ms) {
    assert(leg.steps.size() >= 1 &&
           "Input leg in truncate_leg_by_time() must have at least 1 step!");
    assert(leg.distance_mm > 0 &&
           "Input leg's distance in truncate_leg_by_time() must be positive!");
    assert(leg.duration_ms > 0 &&
           "Input leg's duration in truncate_leg_by_time() must be positive!");
    assert(time_ms >= 0 && "Time in truncate_leg_by_time() must be non negative!");
    assert(time_ms < leg.duration_ms &&
           "Time in truncate_leg_by_time() must be less than leg's duration!");

    // Early return.
    if (time_ms == 0) {
        return;
    }

    for (auto i = 0; i < leg.steps.size(); i++) {
        auto &step = leg.steps[i];

        // If we can finish this step within the time, remove the entire step.
        if (step.duration_ms <= time_ms) {
            time_ms -= step.duration_ms;
            continue;
        }

        truncate_step_by_time(step, time_ms);
        leg.steps.erase(leg.steps.begin(), leg.steps.begin() + i);

        break;
    }

    // Recalculate the total duration and distance.
    leg.distance_mm = 0;
    leg.duration_ms = 0;
    for (const auto &step : leg.steps) {
        leg.distance_mm += step.distance_mm;
        leg.duration_ms += step.duration_ms;
    }

    assert(leg.steps.size() >= 1 &&
           "Output leg in truncate_leg_by_time() must have at least 1 step!");
    assert(leg.distance_mm > 0 &&
           "Output leg's distance in truncate_leg_by_time() must be positive!");
    assert(leg.duration_ms > 0 &&
           "Output leg's duration in truncate_leg_by_time() must be positive!");
}

void truncate_route_by_time(Route &route, uint64_t time_ms) {
    assert(route.legs.size() >= 1 &&
           "Input route in truncate_route_by_time() must have at least 1 leg!");
    assert(route.distance_mm > 0 &&
           "Input route's distance in truncate_route_by_time() must be positive!");
    assert(route.duration_ms > 0 &&
           "Input route's duration in truncate_route_by_time() must be positive!");
    assert(time_ms >= 0 && "Time in truncate_route_by_time() must be non negative!");
    assert(time_ms < route.duration_ms &&
           "Time in truncate_route_by_time() must be less than route's duration!");

    // Early return.
    if (time_ms == 0) {
        return;
    }

    for (auto i = 0; i < route.legs.size(); i++) {
        auto &leg = route.legs[i];

        // If we can finish this step within the time, remove the entire step.
        if (leg.duration_ms <= time_ms) {
            time_ms -= leg.duration_ms;
            continue;
        }

        truncate_leg_by_time(leg, time_ms);
        route.legs.erase(route.legs.begin(), route.legs.begin() + i);

        break;
    }

    // Recalculate the total duration and distance.
    route.distance_mm = 0;
    route.duration_ms = 0;
    for (const auto &leg : route.legs) {
        route.distance_mm += leg.distance_mm;
        route.duration_ms += leg.duration_ms;
    }

    assert(route.legs.size() >= 1 &&
           "Output route in truncate_route_by_time() must have at least 1 step!");
    assert(route.distance_mm > 0 &&
           "Output route's distance in truncate_route_by_time() must be positive!");
    assert(route.duration_ms > 0 &&
           "Output route's duration in truncate_route_by_time() must be positive!");
}

void advance_vehicle(Vehicle &vehicle,
                     std::vector<Order> &orders,
                     uint64_t system_time_ms,
                     uint64_t time_ms,
                     bool update_vehicle_stats) {
    // Early return.
    if (time_ms == 0) {
        return;
    }

    for (auto i = 0; i < vehicle.schedule.size(); i++) {
        auto &wp = vehicle.schedule[i];

        // If we can finish this waypoint within the time.
        if (wp.route.duration_ms <= time_ms) {
            system_time_ms += wp.route.duration_ms;
            time_ms -= wp.route.duration_ms;

            vehicle.pos = wp.pos;

            if (update_vehicle_stats) {
                vehicle.dist_traveled_mm += wp.route.distance_mm;
                vehicle.loaded_dist_traveled_mm += wp.route.distance_mm * vehicle.load;
            }

            if (wp.op == WaypointOp::PICKUP) {
                assert(vehicle.load < vehicle.capacity &&
                       "Vehicle's load should never exceed its capacity!");

                orders[wp.order_id].pickup_time_ms = system_time_ms;
                orders[wp.order_id].status = OrderStatus::PICKED_UP;
                vehicle.load++;

                fmt::print("[DEBUG] T = {}s: Vehicle #{} picked up Order #{}\n",
                           system_time_ms / 1000.0,
                           vehicle.id,
                           wp.order_id);
            } else if (wp.op == WaypointOp::DROPOFF) {
                assert(vehicle.load > 0 && "Vehicle's load should not be zero before a dropoff!");

                orders[wp.order_id].dropoff_time_ms = system_time_ms;
                orders[wp.order_id].status = OrderStatus::DROPPED_OFF;
                vehicle.load--;

                fmt::print("[DEBUG] T = {}s: Vehicle #{} droped off Order #{}\n",
                           system_time_ms / 1000.0,
                           vehicle.id,
                           wp.order_id);
            }

            continue;
        }

        // If we can not finish this waypoint, truncate the route.
        const auto original_distance_mm = wp.route.distance_mm;

        truncate_route_by_time(wp.route, time_ms);
        vehicle.pos = wp.route.legs.front().steps.front().poses.front();

        if (update_vehicle_stats) {
            const auto dist_traveled_mm = original_distance_mm - wp.route.distance_mm;
            vehicle.dist_traveled_mm += dist_traveled_mm;
            vehicle.loaded_dist_traveled_mm += dist_traveled_mm * vehicle.load;
        }

        vehicle.schedule.erase(vehicle.schedule.begin(), vehicle.schedule.begin() + i);

        return;
    }

    // We've finished the whole schedule.
    vehicle.schedule.clear();
    return;
}
