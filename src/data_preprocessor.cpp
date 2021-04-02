//
// Created by Leot on 2021/3/30.
//

#include "types.hpp"
#include "utility.hpp"
//#include "demand_generator.hpp"

//bool compare_request(Request req1, Request req2) {
//    return (req1.origin_node_id == req2.origin_node_id) && (req1.destination_node_id == req2.destination_node_id)
//    && (req1.request_time_ms == req2.request_time_ms) && (strcmp(req1.request_time_date, req2.request_time_date) == 0);
//}

void PreprocessRequestDate(const std::string& path_to_data_file) {
    std::string path_to_csv = path_to_data_file + ".csv";
    std::string path_to_bin = path_to_data_file + ".bin";
    auto request_data = LoadRequestsFromCsvFile(path_to_csv);
}

bool compare_node(Pos sta1, Pos sta2) {
    return (sta1.node_id == sta2.node_id) && (sta1.lat == sta2.lat) && (sta1.lon == sta2.lon);
}

//void PreprocessNodeDate(const std::string& path_to_data_file) {
//    std::string path_to_csv = path_to_data_file + ".csv";
//    std::string path_to_bin = path_to_data_file + ".bin";
//    auto node_data = LoadNetworkNodesFromCsvFile(path_to_csv);
//    WriteObjectVectorToBinary<Pos>(node_data, path_to_bin);
//    auto node_data_from_bin = ReadObjectVectorFromBinary<Pos>(path_to_bin);
//    if(std::equal(node_data.begin(), node_data.end(), node_data_from_bin.begin(), compare_node)){
//        fmt::print("[DEBUG] Sucessfully read data from the binary file!\n");
//    }
//}

//void PreprocessShortestPathDate(const std::string& path_to_data_file) {
//    std::string path_to_csv = path_to_data_file + ".csv";
//    std::string path_to_bin = path_to_data_file + ".bin";
//    auto path_data = LoadShortestPathTableFromCsvFile(path_to_csv);
//    WriteObjectVectorToBinary<std::vector<size_t>>(path_data, path_to_bin);
//    auto path_data_from_bin = ReadObjectVectorFromBinary<std::vector<size_t>>(path_to_bin);
//    fmt::print("[DEBUG] The table has {} * {} elements.\n",
//               path_data_from_bin.size(), path_data_from_bin[1].size());
//}

void PreprocessMeanTravelTimeDate(const std::string& path_to_data_file) {
    std::string path_to_csv = path_to_data_file + ".csv";
    std::string path_to_bin = path_to_data_file + ".bin";
    auto travel_time_data = LoadMeanTravelTimeTableFromCsvFile(path_to_csv);
//    WriteObjectVectorToBinary<std::vector<float>>(travel_time_data, path_to_bin);
//    auto travel_time_data_from_bin = ReadObjectVectorFromBinary<std::vector<float>>(path_to_bin);
//    fmt::print("[DEBUG] The table has {} * {} elements.\n",
//               travel_time_data_from_bin.size(), travel_time_data_from_bin[1].size());
}

int main(int argc, const char *argv[]) {

    PreprocessRequestDate("../datalog/taxi-data/manhattan-taxi-20160525");
//    PreprocessNodeDate("../datalog/map-data/stations-101");
//    PreprocessNodeDate("../datalog/map-data/nodes");
//    PreprocessShortestPathDate("../datalog/map-data/path-table");
//    PreprocessMeanTravelTimeDate("../datalog/map-data/mean-table");
//    PreprocessMeanTravelTimeDate("../datalog/map-data/dist-table");

//    std::string a = "test";
//    fmt::print("a is {}\n", a);
//    std::string b = std::move(a);
//    fmt::print("a is {}\n", a);

}