//
// Created by Leot on 2021/4/22.
//

#pragma once

#include "scheduling.hpp"

/// \brief A function using an ILP solver (Gurobi) to compute the optimal assignment.
/// It returns the indices of selected vehicle_trip_pairs.
std::vector<size_t> IlpAssignment(const std::vector<SchedulingResult> &vehicle_trip_pairs);

/// \brief A function greedily computes the assignment, in decreasing size of the trip and increasing cost.
/// It returns the indices of selected vehicle_trip_pairs.
std::vector<size_t> GreedyAssignment(const std::vector<SchedulingResult> &vehicle_trip_pairs);

/// \brief A function used to sort the vehicle_trip_pairs inputted into the above ILP assign solver.
bool SortVehicleTripPairsForILP(const SchedulingResult &a, const SchedulingResult &b);

/// \brief A function used to sort the vehicle_trip_pairs inputted into the above greedy assign solver.
bool SortVehicleTripPairsForGreedy(const SchedulingResult &a, const SchedulingResult &b);



