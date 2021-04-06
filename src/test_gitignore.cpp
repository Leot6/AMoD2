//
// Created by Leot on 2021/3/30.
//
#undef NDEBUG
#include <assert.h>
#include "config.hpp"
#include "types.hpp"
#include "utility.hpp"
#include "demand_generator.hpp"
# include "router.hpp"

int test(int & a) {
    a = 3;
//    a += 1;
    std::cout<<"()a "<<a<<std::endl;
    return a;
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
//    DemandGenerator demand_generator{path + platform_config.data_file_path.path_to_taxi_data,
//                                     platform_config.simulation_config.simulation_start_time,
//                                     platform_config.mod_system_config.request_config.request_density};

//    auto request_data = LoadRequestsFromCsvFile(path + platform_config.data_file_path.path_to_taxi_data);

    int32_t a = 0;
    std::cout<<"a的类型是"<<typeid(a).name()<<std::endl;
    float b = 3.2;
    a = b * 100;
    std::cout<<"a的类型是"<<typeid(a).name()<< a <<std::endl;
    assert(a > 10 && "a<=10");

    std::cout<<"b "<<b<<std::endl;
    b = test(a);
    std::cout<<"a "<<a<<std::endl;
    std::cout<<"b "<<b<<std::endl;

    std::vector<int> ss = {1, 2, 3, 4};

    auto dd = ss;

    ss[2] = 100;
    std::cout << "ss[2] " << ss[2] << std::endl;
    std::cout << "dd[2] " << dd[2] << std::endl;


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