//
// Created by Leot on 2021/4/12.
//

#pragma once

#include "../simulator/types.hpp"
#include "../utility/utility_functions.hpp"

/// \brief The return type of the following function.
/// \details If the order could not be inserted based on the current vehicle status, result is false.
struct SchedulingResult {
    bool success = false;
    size_t vehicle_id;
    std::vector<std::vector<Waypoint>> feasible_schedules;
    size_t best_schedule_idx;
    uint64_t best_schedule_cost_ms = std::numeric_limits<uint64_t>::max();
};
/// \brief Compute all feasible schedules for a vehicle to serve a order.
/// (schedules of a ride-sharing trip T of size k are computed based on schedules of its subtrip of size k-1)
/// \see get_cost_of_schedule has the detialed definition of cost.
/// \param order The order to be inserted.
/// \param orders A vector of all orders.
/// \param vehicle The vehicle that serves the order.
/// \param sub_schedules A vector of feasible schedules of a subtrip.
/// \param system_time_ms The current system time.
/// \tparam router_func The router func that finds path between two poses.
template <typename RouterFunc>
SchedulingResult ComputeScheduleOfInsertingOrderToVehicle(const Order &order,
                                                          const std::vector<Order> &orders,
                                                          const Vehicle &vehicle,
                                                          const std::vector<std::vector<Waypoint>> &sub_schedules,
                                                          uint64_t system_time_ms,
                                                          RouterFunc &router_func);


/// \brief Generate a schedule (consisting of a vector of waypoints) given known pickup and dropoff indices.
/// \param order The order to be inserted.
/// \param vehicle The vehicle that serves the order.
/// \param sub_schedule A feasible schedule of the vehicle.
/// \param pickup_index The index in the waypoint list where we pick up.
/// \param dropoff_index The index in the waypoint list where we drop off.
/// \tparam router_func The router func that finds path between two poses.
template <typename RouterFunc>
std::vector<Waypoint> GenerateScheduleFromSubSchedule(const Order &order,
                                                      const Vehicle &vehicle,
                                                      const std::vector<Waypoint> &sub_schedule,
                                                      size_t pickup_index,
                                                      size_t dropoff_index,
                                                      RouterFunc &router_func);


/// \brief Validate schedule by checking all constraints.
/// Returns true if valid. Otherwise, false, together with the violation type.
template <typename RouterFunc>
std::pair<bool, int> ValidateSchedule(const std::vector<Waypoint> &schedule,
                                      size_t pickup_idx,
                                      size_t dropoff_idx,
                                      const Order &order
                                      const std::vector<Order> &orders,
                                      const Vehicle &vehicle,
                                      uint64_t system_time_ms);


/// \brief Compute the cost (time in millisecond) of serving the current schedule.
/// \details The cost of serving the schedule is defined as the sum of each order's waiting time and travel delay.
uint64_t ComputeScheduleCost(const std::vector<Waypoint> &schedule,
                             const std::vector<Order> &orders,
                             const Vehicle &vehicle,
                             uint64_t system_time_ms);
// Implementation is put in a separate file for clarity and maintainability.
#include "scheduling_impl.hpp"
