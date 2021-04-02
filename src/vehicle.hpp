/// \author Jian Wen
/// \date 2021/02/08

#pragma once

#include "types.hpp"

/// \brief Trucate Step so that the first x milliseconds worth of route is completed.
void truncate_step_by_time(Step &step, uint64_t time_ms);

/// \brief Trucate Leg so that the first x milliseconds worth of route is completed.
void truncate_leg_by_time(Leg &leg, uint64_t time_ms);

/// \brief Trucate Route so that the first x milliseconds worth of route is completed.
void truncate_route_by_time(Route &route, uint64_t time_ms);

/// \brief Advance the vehicle by x milliseconds .
/// \param vehicle the vehicle that contains a schedule to be processed.
/// \param orders the reference to the orders.
/// \param system_time_ms the current system time in milliseconds.
/// \param time_ms the time in seconds that we need to advance the system.
/// \param update_vehicle_stats true if we update the vehicle statistics including distance
/// traveled and loaded distance traveled.
void advance_vehicle(Vehicle &vehicle,
                     std::vector<Order> &orders,
                     uint64_t system_time_ms,
                     uint64_t time_ms,
                     bool update_vehicle_stats = true);
