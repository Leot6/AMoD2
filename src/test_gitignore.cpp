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
        env.set("LogFile", "mip1.log");
        env.start();

        // Create an empty model
        GRBModel model = GRBModel(env);

        // Create variables
        GRBVar x = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "x");
        GRBVar y = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "y");
        GRBVar z = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "z");

        // Set objective: maximize x + y + 2 z
        model.setObjective(x + y + 2 * z, GRB_MAXIMIZE);

        // Add constraint: x + 2 y + 3 z <= 4
        model.addConstr(x + 2 * y + 3 * z <= 4, "c0");

        // Add constraint: x + y >= 1
        model.addConstr(x + y >= 1, "c1");

        // Optimize model
        model.optimize();

        cout << x.get(GRB_StringAttr_VarName) << " "
             << x.get(GRB_DoubleAttr_X) << endl;
        cout << y.get(GRB_StringAttr_VarName) << " "
             << y.get(GRB_DoubleAttr_X) << endl;
        cout << z.get(GRB_StringAttr_VarName) << " "
             << z.get(GRB_DoubleAttr_X) << endl;

        cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

    } catch(GRBException e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } catch(...) {
        cout << "Exception during optimization" << endl;
    }

    return 0;

}