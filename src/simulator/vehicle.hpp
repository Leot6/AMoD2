/// \author Jian Wen
/// \date 2021/02/08

#pragma once
#include "types.hpp"
#include "config.hpp"

#undef NDEBUG
#include <assert.h>

/// \brief Trucate Step so that the first x milliseconds worth of route is completed.
void TruncateStepByTime(Step &step, uint64_t time_ms);

/// \brief Trucate Route so that the first x milliseconds worth of route is completed.
void TruncateRouteByTime(Route &route, uint64_t time_ms);

/// \brief Update the vehicle position by x milliseconds .
/// \param vehicle the vehicle that contains a schedule to be processed.
/// \param orders the reference to the orders.
/// \param system_time_ms the current system time in milliseconds.
/// \param time_ms the time in seconds that we need to advance the system.
/// \param update_vehicle_stats true if we update the vehicle statistics.
std::pair<std::vector<size_t>, std::vector<size_t>> UpdVehiclePos(Vehicle &vehicle,
                     std::vector<Order> &orders,
                     uint64_t system_time_ms,
                     uint64_t time_ms,
                     bool update_vehicle_stats = true);