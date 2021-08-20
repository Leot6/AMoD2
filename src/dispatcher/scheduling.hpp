//
// Created by Leot on 2021/4/12.
//

#pragma once

#include "utility/utility_functions.hpp"

/// \brief The return type of the following function.
/// \details If the order could not be inserted based on the current vehicle status, result is false.
struct SchedulingResult {
    bool success = false;
    std::vector<size_t> trip_ids;  // denoting orders that can be served by a single vehicle through ride-sharing
    size_t vehicle_id;
    std::vector<std::vector<Waypoint>> feasible_schedules;
    size_t best_schedule_idx;
    int32_t best_schedule_cost_ms = std::numeric_limits<int32_t>::max();
    int32_t score = -std::numeric_limits<int32_t>::max();
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
                                      const Order &order,
                                      const std::vector<Order> &orders,
                                      const Vehicle &vehicle,
                                      uint64_t system_time_ms,
                                      RouterFunc &router_func);

/// \brief Quick check if a order is obviously cannot served by a vehicle.
template <typename RouterFunc>
bool PassQuickCheck(const Order &order, const Vehicle &vehicle, uint64_t system_time_ms, RouterFunc &router_func);

/// \brief Build the detailed routes for all vehicles in the selected vehicle-trip pairs.
template <typename RouterFunc>
void UpdScheduleForVehiclesInSelectedVtPairs(std::vector<SchedulingResult> &vehicle_trip_pairs,
                                             const std::vector<size_t> &selected_vehicle_trip_pair_indices,
                                             std::vector<Order> &orders,
                                             std::vector<Vehicle> &vehicles,
                                             RouterFunc &router_func);

/// \brief Build the detailed route for a vehicle based on its assigned schedule, and update the vehicle's status.
template <typename RouterFunc>
void UpdVehicleScheduleAndBuildRoute(Vehicle &vehicle, std::vector<Waypoint> &schedule, RouterFunc &router_func);

/// \brief Compute the cost (time in millisecond) of serving the current schedule.
/// \details The cost of serving the schedule is defined as the sum of each order's total travel delay.
uint32_t ComputeScheduleCost(const std::vector<Waypoint> &schedule,
                             const std::vector<Order> &orders,
                             const Vehicle &vehicle,
                             uint64_t system_time_ms);

/// \brief Compute the scores of candidate vehicle_trip_pair.
/// \details The score is defined as minus the increased delay. The shorter the delay, the higher the score.
void ScoreVtPairWithIncreasedDelay(SchedulingResult &vehicle_trip_pair,
                                   const std::vector<Order> &orders,
                                   const std::vector<Vehicle> &vehicles,
                                   uint64_t system_time_ms,
                                   bool is_reoptimization = false);

/// \brief Compute the scores of all candidate vehicle_trip_pairs.
/// \details The score is defined as the reward of serving orders minus the increased delay.
void ScoreVtPairsWithNumOfOrdersAndIncreasedDelay(std::vector<SchedulingResult> &vehicle_trip_pairs,
                                                 const std::vector<Order> &orders,
                                                 const std::vector<Vehicle> &vehicles,
                                                 uint64_t system_time_ms,
                                                 bool is_reoptimization = false);

// Implementation is put in a separate file for clarity and maintainability.
#include "scheduling_impl.hpp"

