//
// Created by Leot on 2021/4/12.
//

#pragma once

#include "scheduling.hpp"

#include <fmt/format.h>
#undef NDEBUG
#include <assert.h>

SchedulingResult ComputeScheduleOfInsertingOrderToVehicle(const Order &order,
                                                          const std::vector<Order> &orders,
                                                          const Vehicle &vehicle,
                                                          const std::vector<std::vector<Waypoint>> &sub_schedules,
                                                          uint64_t system_time_ms,
                                                          RouterFunc &router_func) {
    SchedulingResult scheduling_result;
    scheduling_result.vehicle_id = vehicle.id;
    for (const auto &sub_schedule: sub_schedules) {
        const auto num_wps = sub_schedule.size();
        // insert the order's pickup point
        for (int pickup_idx = 0; pickup_idx <= num_wps; pickup_idx++ ) {
            // insert the order's dropoff point
            for (int dropoff_idx = pickup_idx; dropoff_idx <= num_wps; dropoff_idx++) {
                auto new_schedule = GenerateScheduleFromSubschedule(
                        order, vehicle, sub_schedule, pickup_idx, dropoff_idx, router_func);
                auto [feasible_this_schedule, violation_type] = ValidateSchedule(
                        new_schedule, pickup_idx, dropoff_idx, order, orders, vehicle, system_time_ms);
                if (feasible_this_schedule) {
                    new_schedule_cost_ms = ComputeScheduleCost(inserting_result.schedule, orders, vehicle);
                    if (new_schedule_cost_ms < scheduling_result.best_schedule_cost_ms) {
                        scheduling_result.best_schedule_idx = scheduling_result.feasible_schedules.size();
                        scheduling_result.best_schedule_cost_ms = new_schedule_cost_ms;
                    }
                    scheduling_result.success = true;
                    scheduling_result.feasible_schedules.push_back(std::move(new_schedule));
                }
                if (violation_type > 0) { break; }
            }
            if (violation_type == 2) { break; }
        }
    }
    return scheduling_result;
}

template <typename RouterFunc>
std::vector<Waypoint> GenerateScheduleFromSubschedule(const Order &order,
                                                      const Vehicle &vehicle,
                                                      const std::vector<Waypoint> &sub_schedule,
                                                      size_t pickup_idx,
                                                      size_t dropoff_idx,
                                                      RouterFunc &router_func) {
    std::vector<Waypoint> new_schedule;

    auto pos = vehicle.pos;
    auto idx = 0;
    while (true) {
        if (idx == pickup_idx) {
            auto route_response = router_func(pos, order.origin, RoutingType::TIME_ONLY);
            if (route_response.status != RoutingStatus::OK) { return {}; }
            pos = order.origin;
            new_schedule.emplace_back(
                    Waypoint{pos, WaypointOp::PICKUP, order.id, std::move(route_response.route)});
        }

        if (idx == dropoff_idx) {
            auto route_response = router_func(pos, order.destination, RoutingType::TIME_ONLY);
            if (route_response.status != RoutingStatus::OK) { return {}; }
            pos = order.destination;
            new_schedule.emplace_back(
                    Waypoint{pos, WaypointOp::DROPOFF, order.id, std::move(route_response.route)});
        }

        if (idx >= sub_schedule.size()) {
            assert (!new_schedule.empty())
            return new_schedule;
        }

        auto route_response = router_func(pos, vehicle.schedule[index].pos, RoutingType::TIME_ONLY);
        if (route_response.status != RoutingStatus::OK) { return {}; }
        pos = sub_schedule[idx].pos;
        new_schedule.emplace_back(Waypoint{pos,
                                  vehicle.schedule[idx].op,
                                  vehicle.schedule[idx].order_id,
                                  std::move(route_response.route)});

        idx++;
    }

    assert(false && "Logical error! We should never reach this line of code!");
}

template <typename RouterFunc>
std::pair<bool, int> ValidateSchedule(const std::vector<Waypoint> &schedule,
                                      size_t pickup_idx,
                                      size_t dropoff_idx,
                                      const Order &order
                                      const std::vector<Order> &orders,
                                      const Vehicle &vehicle,
                                      uint64_t system_time_ms) {
    auto load = vehicle.load;
    auto accumulated_time_ms = system_time_ms + vehicle.step_to_pos.duration_ms;
    auto idx = 0;
    for (const auto &wp : schedule) {
        // The planned pickup/dropoff time should be no larger than the max allowed pickup time.
        accumulated_time_ms += wp.route.duration_ms;
        if (idx >= pickup_idx) {  // the points ahead of the pickup of the inserted order do not need check
            if (wp.op == WaypointOp::PICKUP && accumulated_time_ms > orders[wp.order_id].max_pickup_time_ms) {
                // (wp.order_id == order.id) means the max pickup constraint of the inserted order is violated,
                // since later pick-up brings longer wait, we can break the insertion of this order.
                if (wp.order_id == order.id) { return {false, 2}; }
                // (idx <= dropoff_idx) mean the the violation is caused by the pick-up of the inserted order,
                // since the violation happens before the drop-off of the order
                if (idx <= dropoff_idx) { return {false, 1}; }
                return {false, 0};
            } else if (wp.op == WaypointOp::DROPOFF && accumulated_time_ms > orders[wp.order_id].max_pickup_time_ms) {
                // (wp.order_id == order.id) means the max dropoff constraint of the inserted order is violated,
                // since later dropoff brings longer delay, we do not need to check later dropoff idx.
                if (idx <= dropoff_idx || wp.order_id == order.id) { return {false, 1}; }
                return {false, 0}
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

    return {True, -1};
}

uint64_t ComputeScheduleCost(const std::vector<Waypoint> &schedule,
                             const std::vector<Order> &orders,
                             const Vehicle &vehicle) {

}