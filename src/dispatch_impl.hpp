/// \author Jian Wen
/// \date 2021/02/02

#pragma once

#include "dispatch.hpp"

#include <fmt/format.h>

#include <numeric>

template <typename RouterFunc>
void assign_orders_through_insertion_heuristics(const std::vector<size_t> &pending_order_ids,
                                               std::vector<Order> &orders,
                                               std::vector<Vehicle> &vehicles,
                                               uint64_t system_time_ms,
                                               RouterFunc &router_func) {
    fmt::print("[DEBUG] Assigning orders to vehicles through insertion heuristics.\n");

    // For each order, we assign it to the best vehicle.
    for (auto order_id : pending_order_ids) {
        auto &order = orders[order_id];
        assign_order_through_insertion_heuristics(
            order, orders, vehicles, system_time_ms, router_func);
    }

    return;
}

template <typename RouterFunc>
void assign_order_through_insertion_heuristics(Order &order,
                                              const std::vector<Order> &orders,
                                              std::vector<Vehicle> &vehicles,
                                              uint64_t system_time_ms,
                                              RouterFunc &router_func) {
    InsertionResult res;

    // Iterate through all vehicles and find the one with least additional cost.
    for (const auto &vehicle : vehicles) {
        auto res_this_vehicle = compute_cost_of_inserting_order_to_vehicle(
            order, orders, vehicle, system_time_ms, router_func);

        if (res_this_vehicle.success && res_this_vehicle.cost_ms < res.cost_ms) {
            res = std::move(res_this_vehicle);
        }
    }

    // If none of the vehicles can serve the order, return false.
    if (!res.success) {
        order.status = OrderStatus::WALKAWAY;
        fmt::print("[DEBUG] Failed to assign Order #{}.\n", order.id);

        return;
    }

    // Insert the order to the best vehicle.
    auto &best_vehicle = vehicles[res.vehicle_id];
    insert_order_to_vehicle(order, best_vehicle, res.pickup_index, res.dropoff_index, router_func);

    fmt::print("[DEBUG] Assigned Order #{} to Vehicle #{}, the schedule of which consists of {} waypoints.\n",
               order.id,
               best_vehicle.id,
               best_vehicle.schedule.size());

    return;
}

uint64_t get_cost_of_schedule(const std::vector<Waypoint> &schedule) {
    auto cost_ms = 0;
    auto accumulated_time_ms = 0;

    for (const auto &wp : schedule) {
        accumulated_time_ms += wp.route.duration_ms;

        if (wp.op == WaypointOp::DROPOFF) {
            cost_ms += accumulated_time_ms;
        }
    }

    return cost_ms;
}

bool validate_schedule(const std::vector<Waypoint> &schedule,
                        const std::vector<Order> &orders,
                        const Vehicle &vehicle,
                        uint64_t system_time_ms) {
    auto accumulated_time_ms = system_time_ms;
    auto load = vehicle.load;

    for (const auto &wp : schedule) {
        accumulated_time_ms += wp.route.duration_ms;

        // The planned pickup time should be no larger than the max allowed pickup time.
        if (wp.op == WaypointOp::PICKUP &&
            accumulated_time_ms > orders[wp.order_id].max_pickup_time_ms) {
            return false;
        }

        if (wp.op == WaypointOp::PICKUP) {
            load++;

            // The load should not exceed the vehicle capacity.
            if (load > vehicle.capacity) {
                return false;
            }
        } else if (wp.op == WaypointOp::DROPOFF) {
            load--;
        }
    }

    return true;
}

template <typename RouterFunc>
std::pair<bool, uint64_t> get_pickup_time(Pos pos,
                                          const std::vector<Waypoint> &schedule,
                                          Pos pickup_pos,
                                          size_t pickup_index,
                                          uint64_t system_time_ms,
                                          RouterFunc &router_func) {
    auto pickup_time_ms = system_time_ms;

    auto index = 0;
    while (index < pickup_index) {
        pickup_time_ms += schedule[index].route.duration_ms;
        pos = schedule[index++].pos;
    }

    auto route_response = router_func(pos, pickup_pos, RoutingType::TIME_ONLY);

    if (route_response.status != RoutingStatus::OK) {
        return {false, 0};
    }

    return {true, pickup_time_ms + route_response.route.duration_ms};
}

template <typename RouterFunc>
InsertionResult compute_cost_of_inserting_order_to_vehicle(const Order &order,
                                                          const std::vector<Order> &orders,
                                                          const Vehicle &vehicle,
                                                          uint64_t system_time_ms,
                                                          RouterFunc &router_func) {
    InsertionResult ret;

    // Compute the current cost of serving the schedule.
    const auto current_cost_ms = get_cost_of_schedule(vehicle.schedule);

    const auto num_wps = vehicle.schedule.size();

    // The pickup and dropoff can be inserted into any position of the current waypoint list.
    for (auto pickup_index = 0; pickup_index <= num_wps; pickup_index++) {
        // If we can not pick up the order before the max wait time time, stop iterating.
        auto [success_pickup, pickup_time_ms] = get_pickup_time(
            vehicle.pos, vehicle.schedule, order.origin, pickup_index, system_time_ms, router_func);

        if (!success_pickup || pickup_time_ms > order.max_pickup_time_ms) {
            break;
        }

        for (auto dropoff_index = pickup_index; dropoff_index <= num_wps; dropoff_index++) {
            auto [success_this_insert, cost_ms_this_insert] =
                compute_cost_of_inserting_order_to_vehicle_given_pickup_and_dropoff_indices(
                    order, orders, vehicle, pickup_index, dropoff_index, system_time_ms, router_func);

            if (success_this_insert && cost_ms_this_insert - current_cost_ms < ret.cost_ms) {
                ret.success = true;
                ret.vehicle_id = vehicle.id;
                ret.cost_ms = cost_ms_this_insert - current_cost_ms;
                ret.pickup_index = pickup_index;
                ret.dropoff_index = dropoff_index;
            }
        }
    }

    return ret;
}

template <typename RouterFunc>
std::pair<bool, uint64_t>
compute_cost_of_inserting_order_to_vehicle_given_pickup_and_dropoff_indices(
    const Order &order,
    const std::vector<Order> &orders,
    const Vehicle &vehicle,
    size_t pickup_index,
    size_t dropoff_index,
    uint64_t system_time_ms,
    RouterFunc &router_func) {
    auto new_schedule = generate_schedule(
        order, vehicle, pickup_index, dropoff_index, RoutingType::TIME_ONLY, router_func);

    if (new_schedule.empty()) {
        return {false, 0.0};
    }

    if (!validate_schedule(new_schedule, orders, vehicle, system_time_ms)) {
        return {false, 0.0};
    }

    return {true, get_cost_of_schedule(new_schedule)};
}

template <typename RouterFunc>
void insert_order_to_vehicle(Order &order,
                            Vehicle &vehicle,
                            size_t pickup_index,
                            size_t dropoff_index,
                            RouterFunc &router_func) {
    auto new_schedule = generate_schedule(
        order, vehicle, pickup_index, dropoff_index, RoutingType::FULL_ROUTE, router_func);

    assert(!new_schedule.empty() && "The generated new schedule should be never empty!");

    order.status = OrderStatus::DISPATCHED;
    vehicle.schedule = std::move(new_schedule);

    return;
}

template <typename RouterFunc>
std::vector<Waypoint> generate_schedule(const Order &order,
                                         const Vehicle &vehicle,
                                         size_t pickup_index,
                                         size_t dropoff_index,
                                         RoutingType routing_type,
                                         RouterFunc &router_func) {
    std::vector<Waypoint> ret;

    auto pos = vehicle.pos;
    auto index = 0;
    while (true) {
        if (index == pickup_index) {
            auto route_response = router_func(pos, order.origin, routing_type);

            if (route_response.status != RoutingStatus::OK) {
                return {};
            }

            pos = order.origin;
            ret.emplace_back(
                Waypoint{pos, WaypointOp::PICKUP, order.id, std::move(route_response.route)});
        }

        if (index == dropoff_index) {
            auto route_response = router_func(pos, order.destination, routing_type);

            if (route_response.status != RoutingStatus::OK) {
                return {};
            }

            pos = order.destination;
            ret.emplace_back(
                Waypoint{pos, WaypointOp::DROPOFF, order.id, std::move(route_response.route)});
        }

        if (index >= vehicle.schedule.size()) {
            return ret;
        }

        auto route_response = router_func(pos, vehicle.schedule[index].pos, routing_type);

        if (route_response.status != RoutingStatus::OK) {
            return {};
        }

        pos = vehicle.schedule[index].pos;
        ret.emplace_back(Waypoint{pos,
                                  vehicle.schedule[index].op,
                                  vehicle.schedule[index].order_id,
                                  std::move(route_response.route)});

        index++;
    }

    assert(false && "Logical error! We should never reach this line of code!");
}
