//
// Created by Leot on 2021/4/22.
//

#include "ilp_assign.hpp"

std::vector<size_t> GreedyAssignment(const std::vector<SchedulingResult> &vehicle_trip_pairs) {
    TIMER_START(t)
    if (DEBUG_PRINT) {
        fmt::print("                +Greedy assignment...");
    }
    std::vector<size_t> selected_vehicle_trip_pair_indices;
    std::vector<size_t> selected_vehicle_ids;
    std::vector<size_t> selected_order_ids;
    for (auto i = 0; i < vehicle_trip_pairs.size(); i++) {
        auto &vt_pair = vehicle_trip_pairs[i];
        // check if the vehicle has been selected
        if (std::find(selected_vehicle_ids.begin(), selected_vehicle_ids.end(), vt_pair.vehicle_id)
            != selected_vehicle_ids.end()) { continue; }
        // check if any order in the trip has been selected
        bool flag_at_least_one_order_has_been_selected = false;
        for (auto order_id : vt_pair.trip_ids) {
            if (std::find(selected_order_ids.begin(), selected_order_ids.end(), order_id) != selected_order_ids.end()) {
                flag_at_least_one_order_has_been_selected = true;
                break;
            }
        }
        if (flag_at_least_one_order_has_been_selected) { continue; }
        selected_vehicle_ids.push_back(vt_pair.vehicle_id);
        selected_order_ids.insert(selected_order_ids.end(), vt_pair.trip_ids.begin(), vt_pair.trip_ids.end());
        selected_vehicle_trip_pair_indices.push_back(i);
    }
    if (DEBUG_PRINT) {
        TIMER_END(t)
    }
    return selected_vehicle_trip_pair_indices;
}


bool SortVehicleTripPairs(const SchedulingResult &a, const SchedulingResult &b) {
    if (a.vehicle_id != b.vehicle_id) {
        return (a.vehicle_id < b.vehicle_id);
    } else {
        if (a.trip_ids.size() != b.trip_ids.size()){
            return (a.trip_ids.size() < b.trip_ids.size());
        } else {
            return (a.best_schedule_cost_ms < b.best_schedule_cost_ms);
        }
    }
}


bool SortVehicleTripPairsForGreedy(const SchedulingResult &a, const SchedulingResult &b) {
    if (a.trip_ids.size() != b.trip_ids.size()){
        return (a.trip_ids.size() < b.trip_ids.size());
    } else {
        return (a.best_schedule_cost_ms < b.best_schedule_cost_ms);
    }
}