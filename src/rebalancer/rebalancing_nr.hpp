//
// Created by Leot on 2021/4/20.
//

#pragma once

#include "dispatcher/scheduling.hpp"


/// \brief Reposition the idel vehicles using a naive rebalancer.
/// \details naive rebalancer (NR): dispatch idle vehicle to the locations of unassigned requests, under the
/// assumption that it is likely that more requests occur in the same area where all requests cannot be satisfied
/// \param orders A vector of all orders.
/// \param vehicles A vector of all vehicles.
/// \tparam router_func The router func that finds path between two poses.
template <typename RouterFunc>
void RepositionIdleVehicleThroughNaiveRebalancer(const std::vector<Order> &orders,
                                                 std::vector<Vehicle> &vehicles,
                                                 RouterFunc &router_func);

// Implementation is put in a separate file for clarity and maintainability.
#include "rebalancing_nr_impl.hpp"
