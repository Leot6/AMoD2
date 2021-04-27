//
// Created by Leot on 2021/4/22.
//

#pragma once

#include "ilp_assign.hpp"
#include "dispatch_osp.hpp"

/// \brief Assign the new received orders to the vehicles using single-request batch assignment.
/// \details Single-request Batch Assignment (SBA): takes the new received orders for a batch period and assigns
/// them together in a one-to-one match manner, where at most one new order is assigned to a single vehicle.
/// \param new_received_order_ids A vector holding indices to the new received orders in the current epoch.
/// \param orders A vector of all orders.
/// \param vehicles A vector of all vehicles.
/// \param system_time_ms The current system time.
/// \tparam router_func The router func that finds path between two poses.
template <typename RouterFunc>
void AssignOrdersThroughSingleRequestBatchAssign(const std::vector<size_t> &new_received_order_ids,
                                                 std::vector<Order> &orders,
                                                 std::vector<Vehicle> &vehicles,
                                                 uint64_t system_time_ms,
                                                 RouterFunc &router_func);

/// \brief Compute all possible vehicle-order pairs and return the result as a vector.
/// \details Each element in the vector indicates a feasible assignment (insertion) of order to vehicle.
/// \param new_received_order_ids A vector holding indices to the new received orders in the current epoch.
/// \param orders A vector of all orders.
/// \param vehicles A vector of all vehicles.
/// \param system_time_ms The current system time.
/// \tparam router_func The router func that finds path between two poses.
template <typename RouterFunc>
std::vector<SchedulingResult> ComputeFeasibleVehicleOrderPairs(const std::vector<size_t> &new_received_order_ids,
                                                               const std::vector<Order> &orders,
                                                               const std::vector<Vehicle> &vehicles,
                                                               uint64_t system_time_ms,
                                                               RouterFunc &router_func);

// Implementation is put in a separate file for clarity and maintainability.
#include "dispatch_sba_impl.hpp"