/// \author Jian Wen
/// \date 2021/02/08

#include "vehicle.hpp"

#include <fmt/format.h>

#undef NDEBUG

#include <assert.h>

void truncate_step_by_time(Step &step, uint64_t time_ms) {
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
    assert(time_ms < step.duration_ms && "Ratio in truncate_step_by_time() must be within [0, 1)!");

    // Early return.
    if (time_ms == step.duration_ms) {
        return;
    }

    auto ratio = static_cast<double>(time_ms) / step.duration_ms;
    // When the vehicle is travelling on the link from step.poses[0] to step.poses[1],
    // it can be treated as it were at point B to do route planning, considering the time left to arrive step.poses[1]
    step.poses[0].node_id = step.poses[1].node_id;
    step.poses[0].lon = step.poses[0].lon + ratio * (step.poses[1].lon - step.poses[0].lon);
    step.poses[0].lat = step.poses[0].lat + ratio * (step.poses[1].lat - step.poses[0].lat);
    step.distance_mm *= (1 - ratio);
    step.duration_ms -= time_ms;  // we do not use "*= (1 - ratio)" to avoid bug cases, e.g. "11119 / 11120 = 1.0"

//    fmt::print("Output step, t={}, d={}, time_ms={}\n", step.duration_ms, step.distance_mm, time_ms);
//    fmt::print("step.poses.size({}), poses[0]({} {} {}), poses[1]({} {} {})\n", step.poses.size(),
//               step.poses[0].node_id, step.poses[0].lon, step.poses[0].lat,
//               step.poses[1].node_id, step.poses[1].lon, step.poses[1].lat);

    assert(step.poses.size() == 2 &&
           "Output step in truncate_step_by_time() should have 2 poses!");
    assert(step.distance_mm > 0 &&
           "Output step's distance in truncate_step_by_time() must be positive!");
    assert(step.duration_ms > 0 &&
           "Output step's duration in truncate_step_by_time() must be positive!");
}

void truncate_leg_by_time(Leg &leg, uint64_t time_ms) {
    assert(leg.steps.size() >= 2 &&
           "Input leg in truncate_leg_by_time() must have at least 2 step!");
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

//    fmt::print("leg.dist {}, dura {}, time_ms{}\n", leg.distance_mm, leg.duration_ms, time_ms);
//    fmt::print("leg.steps.size {}\n", leg.steps.size());
//    auto & steps1 = leg.steps;
//    for (int i = 0; i< steps1.size(); i++) {
//        fmt::print("[DEBUG] printing step {} ({} poses), t = {}s, d = {}m\n",
//                   i+1, steps1[i].poses.size(), (float)steps1[i].duration_ms/1000, (float)steps1[i].distance_mm/1000);
//        auto & poses = steps1[i].poses;
//        assert (poses.size() == 2);
//        fmt::print("      printing pos {} ({}, {}) \n", poses[0].node_id, poses[0].lon, poses[0].lat);
//        fmt::print("      printing pos {} ({}, {}) \n", poses[1].node_id, poses[1].lon, poses[1].lat);
//    }

    for (auto i = 0; i < leg.steps.size(); i++) {
        auto &step = leg.steps[i];
//        fmt::print("1({}) step.dist {}, dura {}, time_ms{}\n", i, step.distance_mm, step.duration_ms, time_ms);
        // If we can finish this step within the time, remove the entire step.
        if (step.duration_ms <= time_ms) {
            time_ms -= step.duration_ms;
            continue;
        }
        // If we can not finish this step, truncate the step.
//        fmt::print("2({}) step.dist {}, dura {}, time_ms{}\n",i, step.distance_mm, step.duration_ms, time_ms);
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

    assert(leg.steps.size() >= 2 &&
           "Output leg in truncate_leg_by_time() must have at least 2 step!");
    assert(leg.distance_mm > 0 &&
           "Output leg's distance in truncate_leg_by_time() must be positive!");
    assert(leg.duration_ms > 0 &&
           "Output leg's duration in truncate_leg_by_time() must be positive!");
}

void truncate_route_by_time(Route &route, uint64_t time_ms) {
//    fmt::print("Input route, t={}, d={}, time_ms={}\n", route.duration_ms, route.distance_mm, time_ms);
//    auto & leg = route.legs[0];
//    fmt::print("leg.dist {}, dura {}, time_ms{}\n", leg.distance_mm, leg.duration_ms, time_ms);
//    fmt::print("leg.steps.size {}\n", leg.steps.size());
//    auto &steps1 = leg.steps;
//    for (int i = 0; i < steps1.size(); i++) {
//        fmt::print("[DEBUG] printing step {} ({} poses), t = {}s, d = {}m\n",
//                   i + 1, steps1[i].poses.size(), (float) steps1[i].duration_ms / 1000,
//                   (float) steps1[i].distance_mm / 1000);
//        auto &poses = steps1[i].poses;
//        assert (poses.size() == 2);
//        fmt::print("      printing pos {} ({}, {}) \n", poses[0].node_id, poses[0].lon, poses[0].lat);
//        fmt::print("      printing pos {} ({}, {}) \n", poses[1].node_id, poses[1].lon, poses[1].lat);
//    }

    assert(route.legs.size() == 1 &&
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

        // If we can finish this leg within the time, remove the entire leg.
        if (leg.duration_ms <= time_ms) {
            time_ms -= leg.duration_ms;
            continue;
        }
        // If we can not finish this leg, truncate the leg.
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

    assert(route.legs.size() == 1 &&
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

    Step empty_step;
    vehicle.step_to_pos = empty_step;

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

        auto first_step_of_route = vehicle.schedule[0].route.legs[0].steps[0];
        if (first_step_of_route.poses[0].node_id == first_step_of_route.poses[1].node_id) {
            assert (first_step_of_route.duration_ms != 0);
            vehicle.step_to_pos = first_step_of_route;
        }

        return;
    }

    // We've finished the whole schedule.
    vehicle.schedule.clear();
    return;
}
