//
// Created by Leot on 2021/4/20.
//

#pragma once

#include "dispatcher/scheduling.hpp"


/// \brief Reposition the idel vehicles to the nearest pending orders, which have no vehicles around.
/// \details Nearest Pending Order (NPO): dispatch idle vehicle to the locations of unassigned requests, under the
/// assumption that it is likely that more requests occur in the same area where all requests cannot be satisfied
/// \param orders A vector of all orders.
/// \param vehicles A vector of all vehicles.
/// \tparam router_func The router func that finds path between two poses.
template <typename RouterFunc>
void RepositionIdleVehicleToNearestPendingOrder(const std::vector<Order> &orders,
                                                std::vector<Vehicle> &vehicles,
                                                RouterFunc &router_func);

// Implementation is put in a separate file for clarity and maintainability.
#include "rebalancing_npo_impl.hpp"
