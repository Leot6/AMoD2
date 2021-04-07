/// \author Jian Wen
/// \date 2021/02/08

#pragma once
#include "types.hpp"

#undef NDEBUG
#include <assert.h>

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
/// \param update_vehicle_stats true if we update the vehicle statistics.
void advance_vehicle(Vehicle &vehicle,
                     std::vector<Order> &orders,
                     uint64_t system_time_ms,
                     uint64_t time_ms,
                     bool update_vehicle_stats = true);

/// \brief Build the detailed route for a vehicle based on its assigned schedule.
template <typename RouterFunc>
void build_route_for_a_vehicle_schedule(Vehicle &vehicle, RouterFunc &router_func) {
    auto pos = vehicle.pos;
    for (auto i = 0; i < vehicle.schedule.size(); i++) {
        auto &wp = vehicle.schedule[i];
        auto route_response = router_func(pos, wp.pos, RoutingType::FULL_ROUTE);

        // check the accuracy of routing
        int deviation_due_to_data_structure = 5;
        assert(abs(wp.route.duration_ms - route_response.route.duration_ms) <= deviation_due_to_data_structure);
        assert(abs(wp.route.distance_mm - route_response.route.distance_mm) <= deviation_due_to_data_structure);
        wp.route = std::move(route_response.route);
        pos = wp.pos;
    }

    if (vehicle.step_to_pos.duration_ms > 0) {
        auto &first_leg_of_the_schedule = vehicle.schedule[0].route.legs[0];
        first_leg_of_the_schedule.duration_ms += vehicle.step_to_pos.duration_ms;
        first_leg_of_the_schedule.distance_mm += vehicle.step_to_pos.distance_mm;
        first_leg_of_the_schedule.steps.insert(first_leg_of_the_schedule.steps.begin(), vehicle.step_to_pos);
        vehicle.schedule[0].route.duration_ms = first_leg_of_the_schedule.duration_ms;
        vehicle.schedule[0].route.distance_mm = first_leg_of_the_schedule.distance_mm;


    }
}