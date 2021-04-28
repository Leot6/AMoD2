//
// Created by Leot on 2021/4/22.
//

#include "ilp_assign.hpp"
#include "gurobi_c++.h"

#undef NDEBUG
#include <assert.h>

std::vector<size_t> IlpAssignment(const std::vector<SchedulingResult> &vehicle_trip_pairs,
                                  const std::vector<size_t> &considered_order_ids,
                                  const std::vector<Order> &orders,
                                  const std::vector<Vehicle> &vehicles,
                                  bool ensure_assigning_orders_that_are_picking) {
    TIMER_START(t)
    if (DEBUG_PRINT) {
        fmt::print("                +ILP assignment with {} pairs...", vehicle_trip_pairs.size());
    }
    std::vector<size_t> selected_vehicle_trip_pair_indices;
    if (vehicle_trip_pairs.size() == 0) { return selected_vehicle_trip_pair_indices; }

    // Get the coefficients for vehicle_trip_pair cost and order ignore penalty.
    uint32_t max_cost_ms = 1;
    for (const auto &vt_pair :  vehicle_trip_pairs) {
        if (vt_pair.best_schedule_cost_ms > max_cost_ms) { max_cost_ms = vt_pair.best_schedule_cost_ms; }
    }
    int num_length = 1;
    while ( max_cost_ms /= 10 ) { num_length++; }
    auto coe_vt_pair = 1.0 / pow(10, num_length);
    auto ignore_order_penalty = pow(10, 2);
    auto ignore_order_penalty_high = pow(10, 6);

    try {
        // Create an environment.
        GRBEnv env = GRBEnv(true);
        env.set("LogToConsole", "0");
        env.start();

        // Create an empty model.
        GRBModel model = GRBModel(env);

        // Create variables (lower_bound, upper_bounds, objective_coefficient (zero here and set later), variable_type)
        std::vector<GRBVar> var_vt_pair; // var_vt_pair[i] = 1 indicates selecting the i_th vehicle_trip_pair.
        for (auto i = 0; i < vehicle_trip_pairs.size(); i++) {
            var_vt_pair.push_back(model.addVar(0.0, 1.0, 0.0, GRB_BINARY,
                                          fmt::format("var_vt_pair_{}", i)));
        }
        std::vector<GRBVar> var_order;  // var_order[j] = 0 indicates assigning the i_th order in the list.
        for (auto j = 0; j < considered_order_ids.size(); j++) {
            var_order.push_back(model.addVar(0.0, 1.0, 0.0, GRB_BINARY,
                                             fmt::format("var_order_{}", j)));
        }

        // Set objective: minimize Σ var_vt_pair[i] * cost(vt_pair) + Σ var_order[j] * penalty_ignore.
        GRBLinExpr obj = 0.0;
        for (auto i = 0; i < vehicle_trip_pairs.size(); i++) {
            obj += var_vt_pair[i] * vehicle_trip_pairs[i].best_schedule_cost_ms * coe_vt_pair;
        }
        for (auto j = 0; j < considered_order_ids.size(); j++) {
            if (ensure_assigning_orders_that_are_picking &&
                orders[considered_order_ids[j]].status == OrderStatus::PICKING) {
                    obj += var_order[j] * 1.0 * ignore_order_penalty_high;
            } else {
                obj += var_order[j] * 1.0 * ignore_order_penalty;
            }
        }
        model.setObjective(obj, GRB_MINIMIZE);

        // Add constraints.
        for (const auto &vehicle : vehicles) {  // Σ var_vt_pair[i] * Θ_vt(v) <= 1, ∀ v ∈ V (Θ_vt(v) = 1 if v is in vt).
            GRBLinExpr con_this_vehicle = 0.0;
            for (auto i = 0; i < vehicle_trip_pairs.size(); i++) {
                if (vehicle_trip_pairs[i].vehicle_id == vehicle.id) {
                    con_this_vehicle += var_vt_pair[i];
                }
            }
            model.addConstr(con_this_vehicle <= 1);
        }
        for (auto j = 0; j < considered_order_ids.size(); j++) { // Σ var_vt_pair[i] * Θ_vt(order) + var_order[j] = 1.
            GRBLinExpr con_this_order = 0.0;
            const auto &order = orders[considered_order_ids[j]];
            for (auto i = 0; i < vehicle_trip_pairs.size(); i++) {
                const auto &trip_ids_this_vt = vehicle_trip_pairs[i].trip_ids;
                if (std::find(trip_ids_this_vt.begin(), trip_ids_this_vt.end(), order.id) != trip_ids_this_vt.end()) {
                    con_this_order += var_vt_pair[i];
                }
            }
            con_this_order += var_order[j];
            model.addConstr(con_this_order == 1);
        }

        // Optimize model.
        model.optimize();

        for (auto i = 0; i < vehicle_trip_pairs.size(); i++) {
            if (var_vt_pair[i].get(GRB_DoubleAttr_X) == 1){ selected_vehicle_trip_pair_indices.push_back(i); }
        }

        // Check the results of orders
        if (ensure_assigning_orders_that_are_picking) {
            for (auto j = 0; j < considered_order_ids.size(); j++) {
                if (orders[considered_order_ids[j]].status == OrderStatus::PICKING) {
                    assert(var_order[j].get(GRB_DoubleAttr_X) == 0
                           && "Order that was picking is not assigned at this epoch!");
                }
            }
        }

    } catch(GRBException e) {
        std::cout << "Error code = " << e.getErrorCode() << std::endl;
        std::cout << e.getMessage() << std::endl;
    } catch(...) {
        std::cout << "Exception during optimization" << std::endl;
    }



    if (DEBUG_PRINT) {
        TIMER_END(t)
    }
//    exit(0);
    return selected_vehicle_trip_pair_indices;
}

std::vector<size_t> GreedyAssignment(const std::vector<SchedulingResult> &vehicle_trip_pairs) {
    TIMER_START(t)
    if (DEBUG_PRINT) {
        fmt::print("                +Greedy assignment with {} pairs...", vehicle_trip_pairs.size());
    }
    std::vector<size_t> selected_vehicle_trip_pair_indices;
    if (vehicle_trip_pairs.size() == 0) { return selected_vehicle_trip_pair_indices; }
    std::vector<size_t> selected_vehicle_ids;
    std::vector<size_t> selected_order_ids;
    for (auto i = 0; i < vehicle_trip_pairs.size(); i++) {
        auto &vt_pair = vehicle_trip_pairs[i];
        // Check if the vehicle has been selected
        if (std::find(selected_vehicle_ids.begin(), selected_vehicle_ids.end(), vt_pair.vehicle_id)
            != selected_vehicle_ids.end()) { continue; }
        // Check if any order in the trip has been selected
        bool flag_at_least_one_order_has_been_selected = false;
        for (auto order_id : vt_pair.trip_ids) {
            if (std::find(selected_order_ids.begin(), selected_order_ids.end(), order_id) != selected_order_ids.end()) {
                flag_at_least_one_order_has_been_selected = true;
                break;
            }
        }
        if (flag_at_least_one_order_has_been_selected) { continue; }
        // The current vehicle_trip_pair is selected.
        selected_vehicle_ids.push_back(vt_pair.vehicle_id);
        selected_order_ids.insert(selected_order_ids.end(), vt_pair.trip_ids.begin(), vt_pair.trip_ids.end());
        selected_vehicle_trip_pair_indices.push_back(i);
    }
    if (DEBUG_PRINT) {
        TIMER_END(t)
    }
    return selected_vehicle_trip_pair_indices;
}

bool SortForIlp(const SchedulingResult &a, const SchedulingResult &b) {
    if (a.vehicle_id != b.vehicle_id) {
        return (a.vehicle_id < b.vehicle_id);
    } else {
        if (a.trip_ids.size() != b.trip_ids.size()){
            return (a.trip_ids.size() > b.trip_ids.size());
        } else {
            return (a.best_schedule_cost_ms < b.best_schedule_cost_ms);
        }
    }
}

bool SortVehicleTripPairs(const SchedulingResult &a, const SchedulingResult &b) {
    if (a.trip_ids.size() != b.trip_ids.size()){
        return (a.trip_ids.size() > b.trip_ids.size());
    } else {
        return (a.best_schedule_cost_ms < b.best_schedule_cost_ms);
    }
}