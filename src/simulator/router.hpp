/// \author Jian Wen
/// \date 2021/01/29

#pragma once

#include <memory>
#include <string>

#include "types.hpp"
#include "config.hpp"
#include "../utility/utility.hpp"

/// \brief Stateful functor that finds the shortest route for an O/D pair on request.
class Router {
  public:
    /// \brief Constructor.
    explicit Router(DataFilePath _date_file_path_config);

    /// \brief Main functor that finds the shortest route for an O/D pair on request.
    RoutingResponse operator()(const Pos &origin, const Pos &destination, RoutingType type);

    /// \brief Get the node_id of a station.
    size_t getVehicleStationId(const size_t &station_index);

    /// \brief Get the number of vehicle stations.
    size_t getNumOfVehicleStations();

    /// \brief Get the pos of a node.
    Pos getNodePos(const size_t &node_id);

private:
    /// \brief The station node where vehicles are initially placed.
    std::vector<Pos> vehicle_stations_;

    /// \brief The road network of manhattan, consisting of 4091 poses (e.g. <1, -74.017946, 40.706991>).
    /// the node id starts from 1.
    std::vector<Pos> network_nodes_;

    /// \brief The precomputed look-up table, storing the minimum mean travel time path between each road node pair.
    std::vector<std::vector<int>> shortest_path_table_;

    /// \brief The precomputed look-up table, storing the mean travel time between each road node pair.
    std::vector<std::vector<float>> mean_travel_time_table_;

    /// \brief The precomputed look-up table, storing the travel distance between each road node pair.
    std::vector<std::vector<float>> travel_distance_table_;
};

/// \brief A function loading the road network node data from a csv file.
std::vector<Pos> LoadNetworkNodesFromCsvFile(std::string path_to_csv);

/// \brief A function loading the precomputed minimum mean travel time path of each node pair from a csv file.
std::vector<std::vector<int>> LoadShortestPathTableFromCsvFile(std::string path_to_csv);

/// \brief A function loading the precomputed mean travel time of each node pair from a csv file.
std::vector<std::vector<float>> LoadMeanTravelTimeTableFromCsvFile(std::string path_to_csv);
