//
// Created by Leot on 2021/6/27.
//

#pragma once

#include "dispatcher/scheduling.hpp"


/// \brief Reposition the idel vehicles randomly to vehicle stations.
/// \details Random Vehicle Station (RVS): randomly dispatch idle vehicle to one of the vehicle stations.
/// \param vehicles A vector of all vehicles.
/// \tparam router_func The router func that finds path between two poses.
template <typename RouterFunc>
void RepositionIdleVehiclesToRandomVehicleStations(std::vector<Vehicle> &vehicles, RouterFunc &router_func);

// Implementation is put in a separate file for clarity and maintainability.
#include "rebalancing_rvs_impl.hpp"
