//
// Created by Leot on 2021/3/30.
//
#undef NDEBUG
#include <assert.h>
#include "simulator/config.hpp"
#include "simulator/types.hpp"
#include "simulator/demand_generator.hpp"
#include "simulator/router.hpp"

#include <libc.h>
#include "gurobi_c++.h"

#include <numeric>

using namespace std;

int test(std::vector<int> & list, int delete_item) {
    for (auto i = 0; i < list.size(); i++) {
        if (list[i] == delete_item) {
            list.erase(list.begin() + i);
            break;
        }
    }
    return 1;
}

int main(int argc, const char *argv[]) {
    // get the root directory
    const int MAXPATH=250;
    char buffer[MAXPATH];
    getcwd(buffer, MAXPATH);
    std::string build_file_directory = buffer;
    auto root_directory = build_file_directory.substr(0, build_file_directory.find("AMoD2") + 5);
    auto path_to_config_file = root_directory + "/config/platform_demo.yml";
    auto platform_config = load_platform_config(path_to_config_file, root_directory);

    try {

        // Create an environment
        GRBEnv env = GRBEnv(true);
        env.set("LogToConsole", "0");
        env.start();

        // Create an empty model
        GRBModel model = GRBModel(env);


        // Create variables
        // (lower_bound, upper_bounds, objective_coefficient (zero here and set later), variable_type, name)
        std::vector<GRBVar> v_t;
        for (auto i = 0; i < 3; i++) {
            v_t.push_back(model.addVar(0.0, 1.0, 0.0, GRB_BINARY, fmt::format("vt_{}", i)));
        }

        // Set objective: maximize x + y + 2 z
        std::vector<int> obj_c = {1, 1, 2};  // objective_coefficients
        GRBLinExpr obj = 0.0;
        for (auto i = 0; i < obj_c.size(); i++) {
            obj += obj_c[i] * v_t[i];
        }
        model.setObjective(obj, GRB_MAXIMIZE);

        // Add constraint for vehicles
        std::vector<GRBLinExpr> c_v;
        c_v.push_back(v_t[0] + 2 * v_t[1] + 3 * v_t[2]);
        c_v.push_back(v_t[0] + v_t[1]);
        model.addConstr(c_v[0] <= 4);
        model.addConstr(c_v[1] >= 1);


        // Optimize model
        model.optimize();
        for (const auto & var : v_t) {
            fmt::print("{} {}\n", var.get(GRB_StringAttr_VarName), var.get(GRB_DoubleAttr_X));
        }
        fmt::print("Obj: {}\n", model.get(GRB_DoubleAttr_ObjVal));


    } catch(GRBException e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } catch(...) {
        cout << "Exception during optimization" << endl;
    }

    return 0;

}