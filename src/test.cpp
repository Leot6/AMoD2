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
#include <stdio.h>
#include <iostream>
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
    // Get the root directory.
    const int MAXPATH=250;
    char buffer[MAXPATH];
    getcwd(buffer, MAXPATH);
    std::string build_file_directory = buffer;
    auto root_directory = build_file_directory.substr(0, build_file_directory.find("AMoD2") + 5);
    auto path_to_config_file = root_directory + "/config/platform_demo.yml";
    auto platform_config = load_platform_config(path_to_config_file, root_directory);

    int num_length = 1;
    auto max_cost_ms = 290010;
    while ( max_cost_ms /= 10 ) { num_length++; }
    max_cost_ms = 290010;
    max_cost_ms -= 100-90;
    fmt::print("num_length {} ({})", num_length, max_cost_ms);

    return 0;

}