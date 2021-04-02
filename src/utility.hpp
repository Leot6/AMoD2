//
// Created by Leot on 2021/3/26.
//

#pragma once
#include "types.hpp"
//#include "rapidcsv.h"
#include "csv.hpp"

#include <sys/stat.h>
#include <fmt/format.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <ctime>
#include <chrono>

/// \brief A function converting "%Y-%m-%d %H:%M:%S" to seconds.
int32_t ConvertTimeDateToSeconds(std::string time_date);

/// \brief A function converting seconds to "%Y-%m-%d %H:%M:%S".
std::string ConvertTimeSecondToDate(int32_t time_sec);

/// \brief A function computing the accumulated seconds of the day from "%Y-%m-%d %H:%M:%S",
/// e.g. "2015-05-25 01:00:00" -> 3600
int32_t ComputeTheAccumulatedSecondsFrom0Clock(std::string time_date);

/// \brief The current time of the system, used to calculate the computational time
std::time_t getTimeStamp();

#define TIMER_START(FUNC) auto FUNC = getTimeStamp();
#define TIMER_END(FUNC) std::cout << "[" << #FUNC << "]" << " cost time: " << (static_cast<float>(getTimeStamp() - FUNC)/1000.0f) << "s" << std::endl;

/// \brief A function to check whether the data file is existing.
void CheckFileExistence(const std::string& path_to_file);

/// \brief A function loading the taxi trip data from a csv file.
std::vector<Request> LoadRequestsFromCsvFile(std::string path_to_csv);

/// \brief A function loading the road network node data from a csv file.
std::vector<Pos> LoadNetworkNodesFromCsvFile(std::string path_to_csv);

/// \brief A function loading the precomputed minimum mean travel time path of each node pair from a csv file.
std::vector<std::vector<size_t>> LoadShortestPathTableFromCsvFile(std::string path_to_csv);

/// \brief A function loading the precomputed mean travel time of each node pair from a csv file.
std::vector<std::vector<float>> LoadMeanTravelTimeTableFromCsvFile(std::string path_to_csv);