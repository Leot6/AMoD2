/// \author Jian Wen
/// \date 2021/01/29

#include "router.hpp"

#include <osrm/engine_config.hpp>
#include <osrm/json_container.hpp>
#include <osrm/route_parameters.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <iostream>
#include <fstream>

Router::Router(DataFilePath _date_file_path_config) {

    auto &path_to_osrm_data = _date_file_path_config.path_to_osrm;

    // Set up the OSRM backend routing engine.
    osrm::EngineConfig config;

    // Path to the base osrm map data.
    config.storage_config = {std::move(path_to_osrm_data)};

    // No shared memory.
    config.use_shared_memory = false;

    // Use Multi-Level Dijkstra (MLD) for routing. This requires
    // extract+partition+customize pre-processing.
    config.algorithm = osrm::EngineConfig::Algorithm::MLD;

    // Create the routing engine instance.
    osrm_ptr_ = std::make_unique<osrm::OSRM>(config);

    fmt::print("[INFO] Initiated the OSRM routing engine using map data from {}.\n",
               path_to_osrm_data);

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

RoutingResponse Router::Routing(const Pos &origin, const Pos &destination, RoutingType type) {
    // Convert to the osrm route request params.
    osrm::RouteParameters params;

    // Origin -> Destination.
    params.coordinates.push_back(
            {osrm::util::FloatLongitude{origin.lon}, osrm::util::FloatLatitude{origin.lat}});
    params.coordinates.push_back(
            {osrm::util::FloatLongitude{destination.lon}, osrm::util::FloatLatitude{destination.lat}});

    // Set up other params.
    if (type == RoutingType::TIME_ONLY) {
        params.steps = false; // returns only the time and distance
    } else if (type == RoutingType::FULL_ROUTE) {
        params.steps = true; // returns the detailed steps of the route
    } else {
        assert(false && "Uninitialized RoutingType in Router!");
    }
    params.alternatives = false; // no alternative routes, just find the best one
    params.geometries = osrm::RouteParameters::GeometriesType::GeoJSON; // route geometry encoded
    // in GeoJSON
    params.overview = osrm::RouteParameters::OverviewType::False;       // no route overview

    // Response is in JSON format.
    osrm::engine::api::ResultT result = osrm::json::Object();

    // Execute routing request, which does the heavy lifting.
    const auto status = osrm_ptr_->Route(params, result);

    // Parse the result.
    auto &json_result = result.get<osrm::json::Object>();
    RoutingResponse response;

    if (status == osrm::Status::Ok) {
        auto &routes = json_result.values["routes"].get<osrm::json::Array>();

        // Return empty response if empty route.
        if (routes.values.empty()) {
            response.message = "No routes returned between the requested origin and destination.";
            return response;
        }

        // Let's just use the first route.
        auto &route = routes.values.at(0).get<osrm::json::Object>();
        const uint64_t distance_mm =
                route.values["distance"].get<osrm::json::Number>().value * 1000;
        const uint64_t duration_ms =
                route.values["duration"].get<osrm::json::Number>().value * 1000;

        // Return empty response if extract does not contain the default coordinates
        // from above.
        if (distance_mm == 0 || duration_ms == 0) {
            response.status = RoutingStatus::EMPTY;
            response.message = "Distance or duration of route is zero. You are "
                               "probably doing a query outside of the OSM extract.";

            return response;
        }

        response.status = RoutingStatus::OK;

        if (type == RoutingType::TIME_ONLY) {
            response.route.distance_mm = distance_mm;
            response.route.duration_ms = duration_ms;
        } else if (type == RoutingType::FULL_ROUTE) {
            response.route = convert_json_to_route(std::move(route));
        }

        return response;
    }

    const auto code = json_result.values["code"].get<osrm::json::String>().value;
    const auto message = json_result.values["message"].get<osrm::json::String>().value;

    response.status = RoutingStatus::ERROR;
    response.message = fmt::format("Code: {}, Message {}", code, message);

    return response;
}

RoutingResponse Router::operator()Routing(const Pos &origin, const Pos &destination, RoutingType type) {
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

Route convert_json_to_route(osrm::json::Object route_json) {
    Route route;

    route.distance_mm = route_json.values["distance"].get<osrm::json::Number>().value * 1000;
    route.duration_ms = route_json.values["duration"].get<osrm::json::Number>().value * 1000;

    auto &legs_json = route_json.values["legs"].get<osrm::json::Array>();

    for (auto &leg_json : legs_json.values) {
        auto &leg_json_obejct = leg_json.get<osrm::json::Object>();

        Leg leg;
        leg.distance_mm = leg_json_obejct.values["distance"].get<osrm::json::Number>().value * 1000;
        leg.duration_ms = leg_json_obejct.values["duration"].get<osrm::json::Number>().value * 1000;

        auto &steps_json = leg_json_obejct.values["steps"].get<osrm::json::Array>();

        for (auto &step_json : steps_json.values) {
            auto &step_json_obejct = step_json.get<osrm::json::Object>();

            Step step;
            step.distance_mm =
                    step_json_obejct.values["distance"].get<osrm::json::Number>().value * 1000;
            step.duration_ms =
                    step_json_obejct.values["duration"].get<osrm::json::Number>().value * 1000;

            auto &poses_json = step_json_obejct.values["geometry"]
                    .get<osrm::json::Object>()
                    .values["coordinates"]
                    .get<osrm::json::Array>();

            for (auto &pos_json : poses_json.values) {
                auto &pos_json_obejct = pos_json.get<osrm::json::Array>();

                Pos pos;
                pos.lon = pos_json_obejct.values[0].get<osrm::json::Number>().value;
                pos.lat = pos_json_obejct.values[1].get<osrm::json::Number>().value;

                step.poses.emplace_back(std::move(pos));
            }

            leg.steps.emplace_back(std::move(step));
        }

        route.legs.emplace_back(std::move(leg));
    }

    return route;
}
