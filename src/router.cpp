/// \author Jian Wen
/// \date 2021/01/29

#include "router.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <iostream>
#include <fstream>

Router::Router(DataFilePath _date_file_path_config) {
    auto &path_to_station_data = _date_file_path_config.path_to_vehicle_stations;
    auto &path_to_node_data = _date_file_path_config.path_to_network_nodes;
    auto &path_to_shortest_path_data = _date_file_path_config.path_to_shortest_path_table;
    auto &path_to_mean_travel_time_data = _date_file_path_config.path_to_mean_travel_time_table;
    auto &path_to_travel_distance_data = _date_file_path_config.path_to_travel_distance_table;
    vehicle_stations_ = LoadNetworkNodesFromCsvFile(path_to_station_data);
    network_nodes_ = LoadNetworkNodesFromCsvFile(path_to_node_data);
    shortest_path_table_ = LoadShortestPathTableFromCsvFile(path_to_shortest_path_data);
    mean_travel_time_table_ = LoadMeanTravelTimeTableFromCsvFile(path_to_mean_travel_time_data);
    travel_distance_table_ = LoadMeanTravelTimeTableFromCsvFile(path_to_travel_distance_data);
    fmt::print("[INFO] Router is ready.\n");
}

RoutingResponse Router::operator()(const Pos &origin, const Pos &destination, RoutingType type) {
    RoutingResponse response;
    // onid: origin node id; dnid: destination node id
    auto onid = origin.node_id;
    auto dnid = destination.node_id;
    response.route.duration_ms = mean_travel_time_table_[onid - 1][dnid - 1] * 1000;
    response.route.distance_mm = travel_distance_table_[onid - 1][dnid - 1] * 1000;
    if (response.route.duration_ms >= 0) {
        response.status = RoutingStatus::OK;
    }
    if (type == RoutingType::FULL_ROUTE) {
        // build the simple node path from the shortest path table
        std::vector<size_t> path = {dnid};
        // we use int here because some value in the shortest_path_table is -1
        int pre_node = shortest_path_table_[onid - 1][dnid - 1];
        while (pre_node > 0) {
            path.push_back(pre_node);
            pre_node = shortest_path_table_[onid - 1][pre_node - 1];
        }
        std::reverse(path.begin(), path.end());

        // build the detailed route from the path
        Leg leg;
        for (int i = 0; i < path.size()-1; i++) {
            Step step;
            size_t u = path[i];
            size_t v = path[i + 1];
            step.distance_mm = travel_distance_table_[u - 1][v - 1] * 1000;
            step.duration_ms = mean_travel_time_table_[u - 1][v - 1] * 1000;
            step.poses.push_back(getNodePos(u));
            step.poses.push_back(getNodePos(v));
            leg.distance_mm += step.distance_mm;
            leg.duration_ms += step.duration_ms;
            leg.steps.push_back(step);
        }
        // the last step of a leg is always of length 2,
        // consisting of 2 identical points as a flag of the end of the leg
        Step flag_step;
        flag_step.distance_mm = 0;
        flag_step.duration_ms = 0;
        flag_step.poses.push_back(getNodePos(dnid));
        flag_step.poses.push_back(getNodePos(dnid));
        leg.steps.push_back(flag_step);
        response.route.legs.push_back(leg);

        // Debug code
        if (response.route.duration_ms != leg.duration_ms) {
            if (abs(response.route.duration_ms - leg.duration_ms) < 10) {

            } else {
                fmt::print("[ERROR]onid{}, dnid{}\n", onid, dnid);
                fmt::print("leg.duration_ms {}, route.duration_ms{}\n",
                           leg.duration_ms, response.route.duration_ms);
                exit(0);
            }
        }
        if (response.route.distance_mm != leg.distance_mm) {
            if (abs(response.route.distance_mm - leg.distance_mm) < 10) {

            } else {
                fmt::print("[ERROR]onid{}, dnid{}\n", onid, dnid);
                fmt::print("leg.distance_mm {}, route.distance_mm{}\n",
                           leg.distance_mm, response.route.distance_mm);
                exit(0);
            }
        }
    }
    response.status = RoutingStatus::OK;
    return response;
}

size_t Router::getVehicleStationId(const size_t &station_index) {
    return vehicle_stations_[station_index].node_id;
}

size_t Router::getNumOfVehicleStations() {
    return vehicle_stations_.size();
}

Pos Router::getNodePos(const size_t &node_id) {
    return network_nodes_[node_id - 1];
}

std::vector<Pos> LoadNetworkNodesFromCsvFile(std::string path_to_csv) {
    CheckFileExistence(path_to_csv);
    auto s_time = getTimeStamp();
    std::vector<Pos> all_nodes = {};
    std::ifstream data_csv(path_to_csv); //load the data file
    std::string line;
    getline(data_csv, line);  // ignore the first line
    while (getline(data_csv, line)) {  // read every line
        std::istringstream readstr(line); // string every line
        std::vector<std::string> data_line;
        std::string info;
        while (getline(readstr, info, ',')) {
            data_line.push_back(info);
        }
        Pos node;
        node.node_id = std::stoi(data_line[0]);
        node.lon = std::stof(data_line[1]);
        node.lat = std::stof(data_line[2]);
        all_nodes.push_back(node);
    }
    fmt::print("[DEBUG] ({}s) Loaded node data from {}, with {} nodes.\n",
               float(getTimeStamp() - s_time) / 1000, path_to_csv, all_nodes.size());
    return std::move(all_nodes);
}

std::vector<std::vector<int>> LoadShortestPathTableFromCsvFile(std::string path_to_csv) {
    CheckFileExistence(path_to_csv);
    auto s_time = getTimeStamp();
    std::vector<std::vector<int>> shortest_path_table = {};
    using namespace csv;
    csv::CSVReader csv_reader(path_to_csv);
    for (CSVRow &row: csv_reader) { // Input iterator
        std::vector<int> int_row;
        int_row.reserve(row.size());
        long i = 0;
        for (CSVField &field: row) {
            if (i == 0) {
                i++;
                continue;
            }
            int_row.push_back(field.get<int>());
        }
        shortest_path_table.push_back(int_row);
    }
    fmt::print("[DEBUG] ({}s) Loaded shortest path data from {}, with {} * {} node pairs.\n",
               float(getTimeStamp() - s_time) / 1000, path_to_csv,
               shortest_path_table.size(), shortest_path_table[0].size());
    return shortest_path_table;
}

std::vector<std::vector<float>> LoadMeanTravelTimeTableFromCsvFile(std::string path_to_csv) {
    CheckFileExistence(path_to_csv);
    auto s_time = getTimeStamp();
    std::vector<std::vector<float>> mean_travel_time_table = {};
    using namespace csv;
    csv::CSVReader csv_reader(path_to_csv);
    for (CSVRow &row: csv_reader) { // Input iterator
        std::vector<float> float_row;
        float_row.reserve(row.size());
        long i = 0;
        for (CSVField &field: row) {
            if (i == 0) {
                i++;
                continue;
            }
            float_row.push_back(field.get<float>());
        }
        mean_travel_time_table.push_back(float_row);
    }
    fmt::print("[DEBUG] ({}s) Loaded shortest path data from {}, with {} * {} node pairs.\n",
               float(getTimeStamp() - s_time) / 1000, path_to_csv,
               mean_travel_time_table.size(), mean_travel_time_table[0].size());
    return mean_travel_time_table;
}