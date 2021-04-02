/// \author Jian Wen
/// \date 2021/02/01

#pragma once

#include "types.hpp"
#include "utility.hpp"

#include <cstdint>
#include <memory>
#include <string>

/// \brief Stateful functor that generates orders based on demand data.
class DemandGenerator {
  public:
    /// \brief Constructor.
    explicit DemandGenerator(std::string _path_to_taxi_data, std::string _simulation_start_time,
                             float _request_density);

    /// \brief Main functor that generates the requests til the target system time.
    std::vector<Request> operator()(uint64_t target_system_time_ms);

    const std::vector<Request> & GetAllRequests() const;


  private:
    /// \brief The system time starting from 0.
    uint64_t system_time_ms_ = 0;

    /// \brief The real taxi trip data loaded from a csv file.
    std::vector<Request> all_requests_ = {};

    /// \brief The init request start time, determined by the simulation time.
    uint64_t init_request_time_ms_ = 0;

    /// \brief The init request index when the simulation starts.
    size_t init_request_idx_ = 0;

    /// \brief The number of requests that have been generated.
    size_t current_request_count_ = 0;

    /// \brief // the percentage of taxi data considered.
    float request_density_ = 1.0;

};
