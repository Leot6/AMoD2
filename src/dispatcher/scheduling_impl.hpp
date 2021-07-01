//
// Created by Leot on 2021/4/12.
//

#pragma once

#include "scheduling.hpp"

#include <fmt/format.h>

#undef NDEBUG
#include <assert.h>


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
        // Insert the order's pickup point.
        for (int pickup_idx = 0; pickup_idx <= num_wps; pickup_idx++) {
            // Insert the order's drop-off point.
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
                }
                if (violation_type > 0) { break; }
            }
            if (violation_type == 2) { break; }
        }
    }
    return scheduling_result;
}

template<typename RouterFunc>
std::vector<Waypoint> GenerateScheduleFromSubSchedule(const Order &order,
                                                      const Vehicle &vehicle,
                                                      const std::vector<Waypoint> &sub_schedule,
                                                      size_t pickup_idx,
                                                      size_t dropoff_idx,
                                                      RouterFunc &router_func) {
    std::vector<Waypoint> new_schedule;
    auto pre_pos = vehicle.pos;
    int idx = 0;
    while (true) {
        if (idx == pickup_idx) {
            auto route = router_func(pre_pos, order.origin, RoutingType::TIME_ONLY);
            new_schedule.emplace_back(
                    Waypoint{order.origin, WaypointOp::PICKUP, order.id, std::move(route)});
            pre_pos = order.origin;
        }
        if (idx == dropoff_idx) {
            auto route = router_func(pre_pos, order.destination, RoutingType::TIME_ONLY);
            new_schedule.emplace_back(
                    Waypoint{order.destination, WaypointOp::DROPOFF, order.id, std::move(route)});
            pre_pos = order.destination;
        }
        if (idx >= sub_schedule.size()) {
            assert (!new_schedule.empty());
            return new_schedule;
        }
        auto route = router_func(pre_pos, sub_schedule[idx].pos, RoutingType::TIME_ONLY);
        new_schedule.emplace_back(Waypoint{sub_schedule[idx].pos,
                                           sub_schedule[idx].op,
                                           sub_schedule[idx].order_id,
                                           std::move(route)});
        pre_pos = sub_schedule[idx].pos;

        idx++;
    }
    assert(false && "Logical error! We should never reach this line of code!");
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
        // The planned pickup/drop-off time should be no larger than the max allowed pickup/drop-off time.
        accumulated_time_ms += wp.route.duration_ms;
        if (idx >= pickup_idx) {  // the points ahead of the pickup of the inserted order do not need check.
            if (wp.op == WaypointOp::PICKUP && accumulated_time_ms > orders[wp.order_id].max_pickup_time_ms) {
                // (wp.order_id == order.id) means that the max pickup constraint of the inserted order is violated,
                // since later pickup brings longer wait, we can break the insertion of this order.
                if (wp.order_id == order.id) { return {false, 2}; }
                // (idx <= dropoff_idx) means that the the violation is caused by the pick-up of the inserted order,
                // since the violation happens before the drop-off of the order, we can try a new pickup insertion.
                if (idx <= dropoff_idx) { return {false, 1}; }
                return {false, 0};
            } else if (wp.op == WaypointOp::DROPOFF && accumulated_time_ms > orders[wp.order_id].max_dropoff_time_ms) {
                // (wp.order_id == order.id) means that the max drop-off constraint of the inserted order is violated,
                // since later drop-off brings longer delay, we do not need to check later drop-off idx.
                if (idx <= dropoff_idx || wp.order_id == order.id) { return {false, 1}; }
                return {false, 0};
            } else if (wp.op == WaypointOp::REPOSITION) {
                auto direct_time_to_reposition_point_ms =
                        router_func(vehicle.pos, wp.pos, RoutingType::TIME_ONLY).duration_ms
                        + vehicle.step_to_pos.duration_ms;
                // The following line is to make sure that a rebalancing vehicle can only be assigned orders when
                // ensuring its visit to the reposition waypoint with a small detour.
                // So that it is a valid rebalancing to the reposition waypoint.
                if (accumulated_time_ms > direct_time_to_reposition_point_ms * 2) { return {false, 0}; }
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

template <typename RouterFunc>
bool PassQuickCheck(const Order &order, const Vehicle &vehicle, uint64_t system_time_ms, RouterFunc &router_func) {
    // The vehicle can not serve the order even when it is idle.
    if (router_func(vehicle.pos, order.origin, RoutingType::TIME_ONLY).duration_ms +
        vehicle.step_to_pos.duration_ms + system_time_ms > order.max_pickup_time_ms) {
        return false;
    } else {
        return true;
    }
}

template <typename RouterFunc>
void UpdVehicleScheduleAndBuildRoute(Vehicle &vehicle, std::vector<Waypoint> &schedule, RouterFunc &router_func) {
    // If a rebalancing vehicle is assigned a trip while ensuring its visit to the reposition waypoint,
    // its rebalancing task can be cancelled.
    if (vehicle.status == VehicleStatus::REBALANCING && schedule.size() > 1) {
        assert(vehicle.schedule.size() == 1);
        auto removed_wp_idx = 0;
        for (auto i = 0; i < schedule.size(); i++) {
            if (schedule[i].op == WaypointOp::REPOSITION) {
                schedule.erase(schedule.begin() + i);
                removed_wp_idx = i;
                break;
            }
        }
        assert(schedule.size() % 2 == 0);
    }

    // Update vehicle's schedule with detailed route.
    vehicle.schedule = schedule;
    auto pre_pos = vehicle.pos;
    for (auto i = 0; i < vehicle.schedule.size(); i++) {
        auto &wp = vehicle.schedule[i];
        auto route = router_func(pre_pos, wp.pos, RoutingType::FULL_ROUTE);
        wp.route = std::move(route);
        pre_pos = wp.pos;
    }

    // Update vehicle's status.
    vehicle.schedule_has_been_updated_at_current_epoch = true;
    if (vehicle.schedule.size() > 0) {
        if (vehicle.schedule[0].op == WaypointOp::PICKUP || vehicle.schedule[0].op == WaypointOp::DROPOFF) {
            vehicle.status = VehicleStatus::WORKING;
        } else if (vehicle.schedule[0].op == WaypointOp::REPOSITION) {
            vehicle.status = VehicleStatus::REBALANCING;
            assert(vehicle.schedule.size() == 1);
        } else {
            assert(false && "Logical error! We should never reach this line of code!");
        }
    } else if (vehicle.schedule.size() == 0) {
        vehicle.status = VehicleStatus::IDLE;
        return;
    }

    // Add vehicle's pre-route, when vehicle is currently on the road link instead of a waypoint node.
    if (vehicle.step_to_pos.duration_ms > 0) {
        auto &route = vehicle.schedule[0].route;
        route.duration_ms += vehicle.step_to_pos.duration_ms;
        route.distance_mm += vehicle.step_to_pos.distance_mm;
        route.steps.insert(route.steps.begin(), vehicle.step_to_pos);
        assert(route.steps[0].poses[0].node_id == route.steps[0].poses[1].node_id);
    }
}

template <typename RouterFunc>
void UpdScheduleForVehiclesInSelectedVtPairs(std::vector<SchedulingResult> &vehicle_trip_pairs,
                                             const std::vector<size_t> &selected_vehicle_trip_pair_indices,
                                             std::vector<Order> &orders,
                                             std::vector<Vehicle> &vehicles,
                                             RouterFunc &router_func) {
    TIMER_START(t)
    if (DEBUG_PRINT) {
        fmt::print("                *Executing assignment with {} pairs...",
                   selected_vehicle_trip_pair_indices.size());
    }

    for (auto idx : selected_vehicle_trip_pair_indices) {
        auto &vt_pair = vehicle_trip_pairs[idx];
//        if (vt_pair.trip_ids.size() == 0) { continue; }  // "stay same" assign, no change to the vehicle's schedule
        for (auto order_id : vt_pair.trip_ids) { orders[order_id].status = OrderStatus::PICKING; }
        auto &vehicle = vehicles[vt_pair.vehicle_id];
        auto &schedule = vt_pair.feasible_schedules[vt_pair.best_schedule_idx];
        UpdVehicleScheduleAndBuildRoute(vehicle, schedule, router_func);

//        if (DEBUG_PRINT) {
//            fmt::print("            +Assigned Trip #{} to Vehicle #{}, with a schedule has {} waypoints.\n",
//                       vt_pair.trip_ids, vehicle.id, vehicle.schedule.size());
//        }
    }

    if (DEBUG_PRINT) { TIMER_END(t) }
}
