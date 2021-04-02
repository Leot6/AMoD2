/// \author Jian Wen
/// \date 2021/02/02

#pragma once

#include "types.hpp"
#include <cstddef>

/// \brief Assign the pending orders to the vehicles using Insertion Heuristics.
/// \param pending_order_ids A vector holding indices to the pending orders.
/// \param orders A vector of all orders.
/// \param vehicles A vector of all vehicles.
/// \param system_time_ms The current system time.
/// \tparam router_func The router func that finds path between two poses.
template <typename RouterFunc>
void assign_orders_through_insertion_heuristics(const std::vector<size_t> &pending_order_ids,
                                               std::vector<Order> &orders,
                                               std::vector<Vehicle> &vehicles,
                                               uint64_t system_time_ms,
                                               RouterFunc &router_func);

/// \brief Assign one single order to the vehicles using using Insertion Heuristics.
/// \param order The order to be inserted.
/// \param orders A vector of all orders.
/// \param vehicles A vector of all vehicles.
/// \param system_time_ms The current system time.
/// \tparam router_func The router func that finds path between two poses.
template <typename RouterFunc>
void assign_order_through_insertion_heuristics(Order &order,
                                              const std::vector<Order> &orders,
                                              std::vector<Vehicle> &vehicles,
                                              uint64_t system_time_ms,
                                              RouterFunc &router_func);

/// \brief Compute the cost (time in millisecond) of serving the current schedule.
/// \details The cost of serving the schedule is defined as the total time taken to drop off each
/// of the orders based on the current ordering.
uint64_t get_cost_of_schedule(const std::vector<Waypoint> &schedule);

/// \brief Validate schedule by checking all constraints. Returns true if valid.
bool validate_schedule(const std::vector<Waypoint> &schedule,
                        const std::vector<Order> &orders,
                        const Vehicle &vehicle,
                        uint64_t system_time_ms);

/// \brief Compute the time that the trip is picked up knowing pickup index.
/// \param pos The current vehicle pose.
/// \param schedule The schedule that are orignially planned.
/// \param pickup_pos The pose for the pickup.
/// \param pickup_index The index in the waypoint list where we pick up.
/// \param system_time_ms The current system time.
/// \tparam router_func The router func that finds path between two poses.
/// \return A pair. True if the order can be inserted, together with the pick up time. False
/// otherwise.
template <typename RouterFunc>
std::pair<bool, uint64_t> get_pickup_time(Pos pos,
                                          const std::vector<Waypoint> &schedule,
                                          Pos pickup_pos,
                                          size_t pickup_index,
                                          uint64_t system_time_ms,
                                          RouterFunc &router_func);

/// \brief The return type of the following function.
/// \details If the order could not be inserted based on the current vehicle status, result is false.
/// Otherwise, result is true. The cost_ms is the additional cost in milliseconds required to serve
/// this order, The following indices point to where to insert the pickup and dropoff.
struct InsertionResult {
    bool success = false;
    size_t vehicle_id;
    uint64_t cost_ms = std::numeric_limits<uint64_t>::max();
    size_t pickup_index;
    size_t dropoff_index;
};

/// \brief Compute the additional cost (time in millisecond) if a vehicle is to serve a order.
/// \see get_cost_of_schedule has the detialed definition of cost.
/// \param order The order to be inserted.
/// \param orders A vector of all orders.
/// \param vehicle The vehicle that serves the order.
/// \param system_time_ms The current system time.
/// \tparam router_func The router func that finds path between two poses.
template <typename RouterFunc>
InsertionResult compute_cost_of_inserting_order_to_vehicle(const Order &order,
                                                          const std::vector<Order> &orders,
                                                          const Vehicle &vehicle,
                                                          uint64_t system_time_ms,
                                                          RouterFunc &router_func);

/// \brief Compute the additional cost knowing pickup and dropoff indices.
/// \param order The order to be inserted.
/// \param orders A vector of all orders.
/// \param vehicle The vehicle that serves the order.
/// \param pickup_index The index in the waypoint list where we pick up.
/// \param dropoff_index The index in the waypoint list where we drop off.
/// \param system_time_ms The current system time.
/// \tparam router_func The router func that finds path between two poses.
/// \return A pair. True if the order can be inserted, together with the additional cost in seconds.
/// False otherwise.
template <typename RouterFunc>
std::pair<bool, uint64_t>
compute_cost_of_inserting_order_to_vehicle_given_pickup_and_dropoff_indices(
    const Order &order,
    const std::vector<Order> &orders,
    const Vehicle &vehicle,
    size_t pickup_index,
    size_t dropoff_index,
    uint64_t system_time_ms,
    RouterFunc &router_func);

/// \brief Insert the order to the vehicle given known pickup and dropoff indices.
/// \param order The order to be inserted.
/// \param vehicle The vehicle that serves the order.
/// \param pickup_index The index in the waypoint list where we pick up.
/// \param dropoff_index The index in the waypoint list where we drop off.
/// \tparam router_func The router func that finds path between two poses.
template <typename RouterFunc>
void insert_order_to_vehicle(Order &order,
                            Vehicle &vehicle,
                            size_t pickup_index,
                            size_t dropoff_index,
                            RouterFunc &router_func);

/// \brief Generate a schedule (consisting of a vector of waypoints) given known pickup and dropoff indices.
/// \param order The order to be inserted.
/// \param vehicle The vehicle that serves the order.
/// \param pickup_index The index in the waypoint list where we pick up.
/// \param dropoff_index The index in the waypoint list where we drop off.
/// \param routing_type The type of the route.
/// \tparam router_func The router func that finds path between two poses.
template <typename RouterFunc>
std::vector<Waypoint> generate_schedule(const Order &order,
                                         const Vehicle &vehicle,
                                         size_t pickup_index,
                                         size_t dropoff_index,
                                         RoutingType routing_type,
                                         RouterFunc &router_func);

// Implementation is put in a separate file for clarity and maintainability.
#include "dispatch_impl.hpp"
