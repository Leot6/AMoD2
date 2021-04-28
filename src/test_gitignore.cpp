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
    // get the root directory
    const int MAXPATH=250;
    char buffer[MAXPATH];
    getcwd(buffer, MAXPATH);
    std::string build_file_directory = buffer;
    auto root_directory = build_file_directory.substr(0, build_file_directory.find("AMoD2") + 5);
    auto path_to_config_file = root_directory + "/config/platform_demo.yml";
    auto platform_config = load_platform_config(path_to_config_file, root_directory);

    for (int i = 0; i < 100; i++) {
        DoProgress(i, 100); // 显示进度条
        fflush(stdout);
        sleep(1000); // 每次显示延迟1s
    }

    return 0;

}