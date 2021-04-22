//
// Created by Leot on 2021/4/12.
//

#pragma once

#include "scheduling.hpp"

#include <fmt/format.h>

#undef NDEBUG
#include <assert.h>

template<typename RouterFunc>
std::vector<Waypoint> GenerateScheduleFromSubSchedule(const Order &order,
                                                      const Vehicle &vehicle,
                                                      const std::vector<Waypoint> &sub_schedule,
                                                      size_t pickup_idx,
                                                      size_t dropoff_idx,
                                                      RouterFunc &router_func) {
    std::vector<Waypoint> new_schedule;

    auto pos = vehicle.pos;
    int idx = 0;
    while (true) {
        if (idx == pickup_idx) {
            auto route = router_func(pos, order.origin, RoutingType::TIME_ONLY);
            pos = order.origin;
            new_schedule.emplace_back(
                    Waypoint{pos, WaypointOp::PICKUP, order.id, std::move(route)});
        }

        if (idx == dropoff_idx) {
            auto route = router_func(pos, order.destination, RoutingType::TIME_ONLY);
            pos = order.destination;
            new_schedule.emplace_back(
                    Waypoint{pos, WaypointOp::DROPOFF, order.id, std::move(route)});
        }

        if (idx >= sub_schedule.size()) {
            assert (!new_schedule.empty());
            return new_schedule;
        }

        auto route = router_func(pos, vehicle.schedule[idx].pos, RoutingType::TIME_ONLY);
        pos = sub_schedule[idx].pos;
        new_schedule.emplace_back(Waypoint{pos,
                                           vehicle.schedule[idx].op,
                                           vehicle.schedule[idx].order_id,
                                           std::move(route)});

        idx++;
    }

    assert(false && "Logical error! We should never reach this line of code!");
}


template<typename RouterFunc>
SchedulingResult ComputeScheduleOfInsertingOrderToVehicle(const Order &order,
                                                          const std::vector<Order> &orders,
                                                          const Vehicle &vehicle,
                                                          const std::vector<std::vector<Waypoint>> &sub_schedules,
                                                          uint64_t system_time_ms,
                                                          RouterFunc &router_func) {
    SchedulingResult scheduling_result;
    scheduling_result.vehicle_id = vehicle.id;
    int violation_type;
    for (const auto &sub_schedule: sub_schedules) {
        const auto num_wps = sub_schedule.size();
        // insert the order's pickup point
        for (int pickup_idx = 0; pickup_idx <= num_wps; pickup_idx++) {
            // insert the order's dropoff point
            for (int dropoff_idx = pickup_idx; dropoff_idx <= num_wps; dropoff_idx++) {
                auto new_schedule = GenerateScheduleFromSubSchedule(
                        order, vehicle, sub_schedule, pickup_idx, dropoff_idx, router_func);
                auto [feasible_this_schedule, violation_type] = ValidateSchedule(
                        new_schedule, pickup_idx, dropoff_idx, order, orders, vehicle, system_time_ms, router_func);
                if (feasible_this_schedule) {
                    auto new_schedule_cost_ms = ComputeScheduleCost(new_schedule, orders, vehicle, system_time_ms);
                    if (new_schedule_cost_ms < scheduling_result.best_schedule_cost_ms) {
                        scheduling_result.best_schedule_idx = scheduling_result.feasible_schedules.size();
                        scheduling_result.best_schedule_cost_ms = new_schedule_cost_ms;
                    }
                    scheduling_result.success = true;
                    scheduling_result.feasible_schedules.push_back(std::move(new_schedule));

//                    // for gi debug
//                    assert(scheduling_result.best_schedule_idx == 0 && scheduling_result.feasible_schedules.size() == 1);
//                    return scheduling_result;
                }
                if (violation_type > 0) { break; }
            }
            if (violation_type == 2) { break; }
        }
    }
    return scheduling_result;
}


template <typename RouterFunc>
void UpdaVehicleScheduleAndBuildRoute(Vehicle &vehicle, std::vector<Waypoint> &schedule, RouterFunc &router_func) {
    // if a rebalancing vehicle is assigned a trip while ensuring it visits the reposition waypoint,
    // its rebalancing task is cancelled
    if (vehicle.status == VehicleStatus::REBALANCING && schedule.size() > 1) {
        assert(vehicle.schedule.size() == 1);
        for (auto i = 0; i < schedule.size(); i++) {
            if (schedule[i].op == WaypointOp::REPOSITION) {
                schedule.erase(schedule.begin() + i);
                break;
            }
        }
        assert(schedule.size() % 2 == 0);
    }

    // update vehicle's schedule
    vehicle.schedule = schedule;
    auto pos = vehicle.pos;
    for (auto i = 0; i < vehicle.schedule.size(); i++) {
        auto &wp = vehicle.schedule[i];
        auto route = router_func(pos, wp.pos, RoutingType::FULL_ROUTE);
        wp.route = std::move(route);
        pos = wp.pos;
    }

    // update vehicle's status
    if (vehicle.schedule.size() > 0) {
        if (vehicle.schedule[0].op == WaypointOp::PICKUP || vehicle.schedule[0].op == WaypointOp::DROPOFF) {
            vehicle.status = VehicleStatus::WORKING;
        } else if (vehicle.schedule[0].op == WaypointOp::REPOSITION) {
            vehicle.status = VehicleStatus::REBALANCING;
            assert(vehicle.schedule.size() == 1);
        }
    } else if (vehicle.schedule.size() == 0) {
        vehicle.status = VehicleStatus::IDLE;
    }

    // add vehicle's pre-route, when vehicle is currently on the road link instead of a waypoint node
    if (vehicle.step_to_pos.duration_ms > 0) {
        auto &route = vehicle.schedule[0].route;
        route.duration_ms += vehicle.step_to_pos.duration_ms;
        route.distance_mm += vehicle.step_to_pos.distance_mm;
        route.steps.insert(route.steps.begin(), vehicle.step_to_pos);
        assert(route.steps[0].poses[0].node_id == route.steps[0].poses[1].node_id);
    }
}

template <typename RouterFunc>
std::pair<bool, int> ValidateSchedule(const std::vector<Waypoint> &schedule,
                                      size_t pickup_idx,
                                      size_t dropoff_idx,
                                      const Order &order,
                                      const std::vector<Order> &orders,
                                      const Vehicle &vehicle,
                                      uint64_t system_time_ms,
                                      RouterFunc &router_func) {
    auto load = vehicle.load;
    auto accumulated_time_ms = system_time_ms + vehicle.step_to_pos.duration_ms;
    auto idx = 0;
    for (const auto &wp : schedule) {
        // The planned pickup/dropoff time should be no larger than the max allowed pickup/dropoff time.
        accumulated_time_ms += wp.route.duration_ms;
        if (idx >= pickup_idx) {  // the points ahead of the pickup of the inserted order do not need check
            if (wp.op == WaypointOp::PICKUP && accumulated_time_ms > orders[wp.order_id].max_pickup_time_ms) {
                // (wp.order_id == order.id) means the max pickup constraint of the inserted order is violated,
                // since later pickup brings longer wait, we can break the insertion of this order.
                if (wp.order_id == order.id) { return {false, 2}; }
                // (idx <= dropoff_idx) mean the the violation is caused by the pick-up of the inserted order,
                // since the violation happens before the dropoff of the order, we can try a new pickup insertion
                if (idx <= dropoff_idx) { return {false, 1}; }
                return {false, 0};
            } else if (wp.op == WaypointOp::DROPOFF && accumulated_time_ms > orders[wp.order_id].max_dropoff_time_ms) {
                // (wp.order_id == order.id) means the max dropoff constraint of the inserted order is violated,
                // since later dropoff brings longer delay, we do not need to check later dropoff idx.
                if (idx <= dropoff_idx || wp.order_id == order.id) { return {false, 1}; }
                return {false, 0};
            } else if (wp.op == WaypointOp::REPOSITION) {
                auto time_to_reposition_point_ms = router_func(vehicle.pos, wp.pos, RoutingType::TIME_ONLY).duration_ms
                                                   + vehicle.step_to_pos.duration_ms;
                if (accumulated_time_ms > time_to_reposition_point_ms * 1.1) { return {false, 0}; }
            }
        }

        // The load should not exceed the vehicle capacity.
        if (wp.op == WaypointOp::PICKUP) {
            load++;
            if (load > vehicle.capacity) {
                return {false, 0};
            }
        } else if (wp.op == WaypointOp::DROPOFF) {
            load--;
        }

        idx++;
    }
    assert (load == 0);
    return {true, -1};
}

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