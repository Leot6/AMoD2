//
// Created by Leot on 2021/4/25.
//

#pragma once

#include "ilp_assign.hpp"


/// \brief Assign the new received orders to the vehicles using multi-request Batch Assignment.
/// \details Optimal Schedule Pool assignment (OSP): takes all picking and pending orders for a batch period and
/// assigns them together in a multi-to-one match manner, where multiple orders (denoted by trip) are assigned to a
/// single vehicle and trips are allowed to be reassigned to different vehicles for better system performance. OSP is
/// an improved version of Request Trip Vehicle (RTV) assignment, it computes all possible vehicle-trip pairs along
/// with the optimal schedule of each pair. The computation of the optimal schedule ensures that no pair is mistakenly
/// ignored. Based on this complete feasible solution space (called optimal schedule pool, each optimal schedule
/// representing a vehicle-trip pair), the optimal assignment policy could be found by an ILP solver.
/// \param new_received_order_ids A vector holding indices to the new received orders in the current epoch.
/// \param orders A vector of all orders.
/// \param vehicles A vector of all vehicles.
/// \param system_time_ms The current system time.
/// \tparam router_func The router func that finds path between two poses.
template <typename RouterFunc>
void AssignOrdersThroughOptimalSchedulePoolAssign(const std::vector<size_t> &new_received_order_ids,
                                                 std::vector<Order> &orders,
                                                 std::vector<Vehicle> &vehicles,
                                                 uint64_t system_time_ms,
                                                 RouterFunc &router_func);

/// \brief Compute all possible vehicle-trip pairs and return the result as a vector.
/// \details Each element in the vector indicates a feasible assignment (insertion) of trip to vehicle.
/// A trip is a group of orders that can be assigned to the same vehicle.
/// \param considered_order_ids A vector holding indices to the orders considered by OSP in the current epoch.
/// \param orders A vector of all orders.
/// \param vehicles A vector of all vehicles.
/// \param system_time_ms The current system time.
/// \tparam router_func The router func that finds path between two poses.
template <typename RouterFunc>
std::vector<SchedulingResult> ComputeFeasibleVehicleTripPairs(const std::vector<size_t> &considered_order_ids,
                                                              const std::vector<Order> &orders,
                                                              const std::vector<Vehicle> &vehicles,
                                                              uint64_t system_time_ms,
                                                              RouterFunc &router_func,
                                                              int cutoff_time_for_a_size_k_trip_search_per_vehicle_ms);

/// \brief Compute all possible trips for the given vehicle, along with the optimal schedule for each trip.
/// \details All possible trips are computed incrementally for increasing ride-sharing trip sizes.
/// \param considered_order_ids A vector holding indices to the orders considered by OSP in the current epoch.
/// \param orders A vector of all orders.
/// \param vehicle The vehicle to be computed.
/// \param system_time_ms The current system time.
/// \tparam router_func The router func that finds path between two poses.
template <typename RouterFunc>
std::vector<SchedulingResult> ComputeFeasibleTripsForOneVehicle(const std::vector<size_t> &considered_order_ids,
                                                                const std::vector<Order> &orders,
                                                                const Vehicle &vehicle,
                                                                uint64_t system_time_ms,
                                                                RouterFunc &router_func,
                                                                int cutoff_time_for_a_size_k_trip_search_ms);

/// \brief Compute all possible size 1 trips for the given vehicle.
/// \details Each element in the vector indicates a feasible assignment (insertion) of order to vehicle.
template <typename RouterFunc>
std::vector<SchedulingResult> ComputeSize1TripsForOneVehicle(const std::vector<size_t> &considered_order_ids,
                                                             const std::vector<Order> &orders,
                                                             const Vehicle &vehicle,
                                                             const std::vector<std::vector<Waypoint>> &basic_schedules,
                                                             uint64_t system_time_ms,
                                                             RouterFunc &router_func);

/// \brief Compute all possible size k (k>1) trips for the given vehicle.
/// \details Each element in the vector indicates a feasible assignment (insertion) of trip to vehicle.
template <typename RouterFunc>
std::vector<SchedulingResult> ComputeSizeKTripsForOneVehicle(
        const std::vector<size_t> &considered_order_ids,
        const std::vector<SchedulingResult> &feasible_trips_of_size_k_minus_1,
        const std::vector<Order> &orders,
        const Vehicle &vehicle,
        uint64_t system_time_ms,
        RouterFunc &router_func,
        int cutoff_time_for_search_ms);

/// \brief Get the basic schedules of the given vehicle, each of which only includes waypoints
/// of dropping off onboard orders.
template <typename RouterFunc>
std::vector<std::vector<Waypoint>> ComputeBasicSchedulesOfVehicle(const std::vector<Order> &orders,
                                                                  const Vehicle &vehicle,
                                                                  uint64_t system_time_ms,
                                                                  RouterFunc &router_func);


// Implementation is put in a separate file for clarity and maintainability.
#include "dispatch_osp_impl.hpp"