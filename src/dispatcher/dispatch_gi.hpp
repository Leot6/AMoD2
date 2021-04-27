//
// Created by Leot on 2021/4/16.
//

#pragma once

#include "scheduling.hpp"
#include <cstddef>

/// \brief Assign the new received orders to the vehicles using Greedy Insertion Heuristics.
/// \details greedy insertion (GI): insert orders to vehicles in first-in-first-out manner
/// \param new_received_order_ids A vector holding indices to the new received orders in the current epoch.
/// \param orders A vector of all orders.
/// \param vehicles A vector of all vehicles.
/// \param system_time_ms The current system time.
/// \tparam router_func The router func that finds path between two poses.
template <typename RouterFunc>
void AssignOrdersThroughGreedyInsertion(const std::vector<size_t> &new_received_order_ids,
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
void HeuristicInsertionOfOneOrder(Order &order,
                        const std::vector<Order> &orders,
                        std::vector<Vehicle> &vehicles,
                        uint64_t system_time_ms,
                        RouterFunc &router_func);

// Implementation is put in a separate file for clarity and maintainability.
#include "dispatch_gi_impl.hpp"
