//
// Created by Leot on 2021/4/22.
//

#pragma once

#include "scheduling.hpp"

/// \brief A function using an ILP solver (Gurobi) to compute the optimal assignment.
/// It returns the indices of selected vehicle_trip_pairs.
std::vector<size_t> IlpAssignment(const std::vector<SchedulingResult> &vehicle_trip_pairs,
                                  const std::vector<size_t> &considered_order_ids,
                                  const std::vector<Order> &orders,
                                  const std::vector<Vehicle> &vehicles,
                                  bool ensure_assigning_orders_that_are_picking = true);

/// \brief A function greedily computes the assignment, in decreasing size of the trip and increasing cost.
/// It returns the indices of selected vehicle_trip_pairs.
std::vector<size_t> GreedyAssignment(const std::vector<SchedulingResult> &vehicle_trip_pairs);

/// \brief A function used to sort the vehicle_trip_pairs, decrease as the trip size and increase as the cost.
bool SortVehicleTripPairs(const SchedulingResult &a, const SchedulingResult &b);




