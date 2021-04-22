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

//    // Create the demand generator based on the input demand file.
//    DemandGenerator demand_generator{platform_config.data_file_path.path_to_taxi_data,
//                                     platform_config.simulation_config.simulation_start_time,
//                                     platform_config.mod_system_config.request_config.request_density};

//    auto request_data = LoadRequestsFromCsvFile(path + platform_config.data_file_path.path_to_taxi_data);

    int32_t a = 0;
    std::cout<<"a的类型是"<<typeid(a).name()<<std::endl;
    float b = 3.2;
    a = b * 100;
    std::cout<<"a的类型是"<<typeid(a).name()<< a <<std::endl;
    assert(a > 10 && "a<=10");


    std::vector<int> ss = {1, 2, 3, 4};

    std::vector<Waypoint> aa;

    fmt::print("aa size {}, aa type {}\n", aa.size(), typeid(aa).name());

    std::vector<int> v = {1, 2, 3, 4, 5};
    fmt::print("v: {}\n", v);
    test(v, 4);
    fmt::print("v1: {}\n", v);

    std::vector<std::pair<size_t, std::vector<int>>> rebalancing_candidates;
//    for (int i = 0; i < 10; i++) {
//        std::vector<int> cc = {1+i, 3, 5};
//        rebalancing_candidates.push_back({i, cc});
//    }
    fmt::print("rebalancing_candidates: {}\n", rebalancing_candidates);
    std::sort(rebalancing_candidates.begin(), rebalancing_candidates.end(),
              [](std::pair<size_t, std::vector<int>> a, std::pair<size_t, std::vector<int>> b) {
                  return a.second[0] > b.second[0];
              });

    fmt::print("rebalancing_candidates: {}\n", rebalancing_candidates);




//    PreprocessRequestDate("../datalog-gitignore/taxi-data/manhattan-taxi-20160525");
//    PreprocessNodeDate("../datalog-gitignore/map-data/stations-101");
//    PreprocessNodeDate("../datalog-gitignore/map-data/nodes");
//    PreprocessShortestPathDate("../datalog-gitignore/map-data/path-table");
//    PreprocessMeanTravelTimeDate("../datalog-gitignore/map-data/mean-table");
//    PreprocessMeanTravelTimeDate("../datalog-gitignore/map-data/dist-table");

//    std::string a = "test";
//    fmt::print("a is {}\n", a);
//    std::string b = std::move(a);
//    fmt::print("a is {}\n", a);

}