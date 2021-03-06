/// \author Jian Wen
/// \date 2021/02/01

#include "demand_generator.hpp"

#include <cstdint>
#include <algorithm>
#include <fmt/format.h>
#undef NDEBUG
#include <assert.h>


DemandGenerator::DemandGenerator(std::string _path_to_taxi_data,
                                 std::string _simulation_start_time,
                                 float _request_density) {

    TIMER_START(t)
    all_requests_ = LoadRequestsFromCsvFile(_path_to_taxi_data);
    init_request_time_ms_ = ComputeTheAccumulatedSecondsFrom0Clock(_simulation_start_time) * 1000;
    while (all_requests_[init_request_idx_].request_time_ms < init_request_time_ms_){
        init_request_idx_++;
    }
    request_density_ = _request_density;

    fmt::print("[INFO] Demand Generator is ready.");
    TIMER_END(t)
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
        new_request.request_time_ms -= init_request_time_ms_;

//        if (DEBUG_PRINT) {
//            fmt::print("[DEBUG] Generated request index {} ({}): origin({}), dest({}).\n",
//                       new_request_idx - init_request_idx_, new_request.request_time_date,
//                       new_request.origin_node_id, new_request.destination_node_id);
//        }

        if (new_request.origin_node_id == 0){
            break;
        }
        current_request_count_++;
        new_request_idx = init_request_idx_ + (size_t)(current_request_count_ / request_density_);
        requests.push_back(new_request);
    }
    return requests;
}

std::vector<Request> LoadRequestsFromCsvFile(std::string path_to_csv) {
    CheckFileExistence(path_to_csv);
    std::vector<Request> all_requests = {};
    csv::CSVReader reader(path_to_csv);

    for (csv::CSVRow& row: reader) {     // input iterator
        Request request;
        request.origin_node_id = row["onid"].get<size_t>();
        request.destination_node_id = row["dnid"].get<size_t>();
        request.request_time_date = row["ptime"].get();
        request.request_time_ms = ComputeTheAccumulatedSecondsFrom0Clock(request.request_time_date) * 1000;
        all_requests.push_back(request);
    }
//    fmt::print("[DEBUG] Load request data from {}, with {} requests.\n", path_to_csv, all_requests.size());
    return std::move(all_requests);
}