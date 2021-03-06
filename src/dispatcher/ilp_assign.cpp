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
        fmt::print("                *ILP assignment with {} pairs...", vehicle_trip_pairs.size());
    }
    std::vector<size_t> selected_vehicle_trip_pair_indices;
    if (vehicle_trip_pairs.size() == 0) { return selected_vehicle_trip_pair_indices; }

    try {
        // 1. Create an environment.
        GRBEnv env = GRBEnv(true);
        env.set("LogToConsole", "0");
        env.start();

        // 2. Create an empty model.
        GRBModel model = GRBModel(env);

        // 3. Create variables
        //     (lower_bound, upper_bounds, objective_coefficient (zero here and set later), variable_type)
        std::vector<GRBVar> var_vt_pair; // var_vt_pair[i] = 1 indicates selecting the i_th vehicle_trip_pair.
        for (auto i = 0; i < vehicle_trip_pairs.size(); i++) {
            var_vt_pair.push_back(model.addVar(0.0, 1.0, 0.0, GRB_BINARY));
        }
        std::vector<GRBVar> var_order;  // var_order[j] = 0 indicates assigning the i_th order in the list.
        for (auto j = 0; j < considered_order_ids.size(); j++) {
            var_order.push_back(model.addVar(0.0, 1.0, 0.0, GRB_BINARY));
        }

        // 4. Set objective: maximize Σ var_vt_pair[i] * score(vt_pair).
        GRBLinExpr obj = 0.0;
        for (auto i = 0; i < vehicle_trip_pairs.size(); i++) {
            obj += var_vt_pair[i] * (vehicle_trip_pairs[i].score);
        }
        model.setObjective(obj, GRB_MAXIMIZE);

        // 5. Add constraints.
        // Add constraint 1: each vehicle (v) can only be assigned at most one schedule (trip).
        //     Σ var_vt_pair[i] * Θ_vt(v) = 1, ∀ v ∈ V. (Θ_vt(v) = 1 if v is in vt).
        for (const auto &vehicle : vehicles) {
            GRBLinExpr con_this_vehicle = 0.0;
            for (auto i = 0; i < vehicle_trip_pairs.size(); i++) {
                if (vehicle_trip_pairs[i].vehicle_id == vehicle.id) {
                    con_this_vehicle += var_vt_pair[i];
                }
            }
            model.addConstr(con_this_vehicle == 1);
        }
        // Add constraint 2: each order/request (r) can only be assigned to at most one vehicle.
        //     Σ var_vt_pair[i] * Θ_vt(r) + var_order[j] = 1, ∀ r ∈ R. (Θ_vt(order) = 1 if r is in vt).
        for (auto j = 0; j < considered_order_ids.size(); j++) {
            const auto &order = orders[considered_order_ids[j]];
            GRBLinExpr con_this_order = 0.0;
            for (auto i = 0; i < vehicle_trip_pairs.size(); i++) {
                const auto &trip_ids_this_vt = vehicle_trip_pairs[i].trip_ids;
                if (std::find(trip_ids_this_vt.begin(), trip_ids_this_vt.end(), order.id) != trip_ids_this_vt.end()) {
                    con_this_order += var_vt_pair[i];
                }
            }
            con_this_order += var_order[j];
            model.addConstr(con_this_order == 1);
        }
        // Add constraint 3: no currently picking order is ignored.
        //     var_order[j] = 0, if OrderStatus==PICKING, ∀ r ∈ R.
        if (ensure_assigning_orders_that_are_picking) {
            for (auto j = 0; j < considered_order_ids.size(); j++) {
                if (orders[considered_order_ids[j]].status == OrderStatus::PICKING) {
                    model.addConstr(var_order[j] == 0);
                }
            }
        }

        // 6. Optimize model.
        model.optimize();

        // 7. Get the result.
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

//        fmt::print("\n[GUROBI] Objective:{}\n", model.get(GRB_DoubleAttr_ObjVal));

    } catch(GRBException e) {
        fmt::print("\n[GUROBI] Error code = {} ({}).\n", e.getErrorCode(), e.getMessage());
    } catch(...) {
        fmt::print("[GUROBI] Exception during optimization\n");
    }
    if (DEBUG_PRINT) { TIMER_END(t) }
    return selected_vehicle_trip_pair_indices;
}

std::vector<size_t> GreedyAssignment(std::vector<SchedulingResult> &vehicle_trip_pairs) {
    TIMER_START(t)
    if (DEBUG_PRINT) {
        fmt::print("                *Greedy assignment with {} pairs...", vehicle_trip_pairs.size());
    }
    std::vector<size_t> selected_vehicle_trip_pair_indices;
    if (vehicle_trip_pairs.size() == 0) { return selected_vehicle_trip_pair_indices; }
    std::sort(vehicle_trip_pairs.begin(), vehicle_trip_pairs.end(), SortVehicleTripPairs);
    std::vector<size_t> selected_vehicle_ids;
    std::vector<size_t> selected_order_ids;
    for (auto i = 0; i < vehicle_trip_pairs.size(); i++) {
        auto &vt_pair = vehicle_trip_pairs[i];
        // Check if the vehicle has been selected.
        if (std::find(selected_vehicle_ids.begin(), selected_vehicle_ids.end(), vt_pair.vehicle_id)
            != selected_vehicle_ids.end()) { continue; }
        // Check if any order in the trip has been selected.
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

bool SortVehicleTripPairs(const SchedulingResult &a, const SchedulingResult &b) {
    return (a.score > b.score);
}
