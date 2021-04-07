/// \author Jian Wen
/// \date 2021/02/01

#include "demand_generator.hpp"

#include <string>
#include <cstdint>
#include <fmt/format.h>

#include <algorithm>

DemandGenerator::DemandGenerator(std::string _path_to_taxi_data, std::string _simulation_start_time,
                                 float _request_density) {
    all_requests_ = LoadRequestsFromCsvFile(_path_to_taxi_data);
//    auto all_request1 = LoadRequestsFromCsvFile_naive_version(_path_to_taxi_data);
    auto s_time = getTimeStamp();
    init_request_time_ms_ = ComputeTheAccumulatedSecondsFrom0Clock(_simulation_start_time) * 1000;
    while (all_requests_[init_request_idx_].request_time_ms < init_request_time_ms_){
        init_request_idx_ += 1;
    }
    request_density_ = _request_density;
    fmt::print("[INFO] ({}s) Calculate the initial request index {} ({}).\n",
               float (getTimeStamp() - s_time)/1000, init_request_idx_, _simulation_start_time);
    fmt::print("[INFO] # Demand Generator is ready. #\n");
}

std::vector<Request> DemandGenerator::operator()(uint64_t target_system_time_ms) {
    assert(system_time_ms_ <= target_system_time_ms &&
           "[ERROR] The target_system_time should be no less than the current system time in "
           "Demand Generator!");

    // System time moves to the target.
    system_time_ms_ = target_system_time_ms;
    std::vector<Request> requests = {};

    size_t new_request_idx = init_request_idx_ + (size_t)(current_request_count_ / request_density_);
    while (all_requests_[new_request_idx].request_time_ms < system_time_ms_ + init_request_time_ms_){
        Request new_request = all_requests_[new_request_idx];
//        fmt::print("[DEBUG] Generated request index {} ({}): origin({}), dest({}).\n",
//                   new_request_idx - init_request_idx_,new_request.request_time_date,
//                   new_request.origin_node_id, new_request.destination_node_id);
        if (new_request.origin_node_id == 0){
            break;
        }
        current_request_count_ += 1;
        new_request_idx = init_request_idx_ + (int32_t)(current_request_count_ / request_density_);
        requests.push_back(new_request);
    }
    return requests;
}

const std::vector<Request> &DemandGenerator::GetAllRequests() const {
    return all_requests_;
}

std::vector<Request> LoadRequestsFromCsvFile(std::string path_to_csv) {
    CheckFileExistence(path_to_csv);
    auto s_time = getTimeStamp();
    std::vector<Request> all_requests = {};
    using namespace csv;
    CSVReader reader(path_to_csv);
    for (CSVRow& row: reader) { // Input iterator
        auto onid = row["onid"].get<size_t>();
        auto dnid = row["dnid"].get<size_t>();
        auto request_time_date = row["ptime"].get();
        auto request_time_ms = ComputeTheAccumulatedSecondsFrom0Clock(request_time_date) * 1000;
        all_requests.emplace_back(onid,dnid,request_time_ms, request_time_date);
    }
    fmt::print("[INFO] ({}s) Load request data from {}, with {} requests.\n",
               float (getTimeStamp() - s_time)/1000, path_to_csv, all_requests.size());
    return std::move(all_requests);
}