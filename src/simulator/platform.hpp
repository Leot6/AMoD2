/// \author Jian Wen
/// \date 2021/02/01

#pragma once

#include "config.hpp"
#include "types.hpp"
#include "vehicle.hpp"
#include "utility/utility_functions.hpp"
#include "utility/tqdm.h"
#include "dispatcher/dispatch_gi.hpp"
#include "rebalancer/rebalancing_nr.hpp"

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>

/// \brief The agent-based modeling platform that simulates the mobility-on-demand system.
template <typename RouterFunc, typename DemandGeneratorFunc>
class Platform {
  public:
    /// \brief Constructor.
    explicit Platform(PlatformConfig _platform_config,
                      RouterFunc _router_func,
                      DemandGeneratorFunc _demand_generator_func);

    /// \brief Destructor.
    ~Platform();

    /// \brief Delete the other constructors. Rule of five.
    Platform(const Platform &other) = delete;
    Platform(Platform &&other) = delete;
    Platform &operator=(const Platform &other) = delete;
    Platform &operator=(Platform &&other) = delete;

    /// \brief Run simulation. The master function that manages the entire simulation process.
    void RunSimulation(std::string simulation_init_time, float total_init_time_s);

  private:
    /// \brief Run simulation for one cycle. Invoked repetetively by run_simulation().
    void RunCycle();

    /// \brief Advance all vehicles for the given time and move forward the system time.
    void UpdVehiclesPositions(uint64_t time_ms);

    /// \brief Generate orders at the end of each cycle.
    std::vector<size_t> GenerateOrders();

    /// \brief Write the data of the current simulation state to datalog.
    void WriteToDatalog();

    /// \brief Create the report based on the statistical analysis using the simulated data.
    void CreateReport(std::string simulation_init_time, float total_init_time_s, float total_runtime_s);

    /// \brief The set of config parameters for the simulation platform.
    PlatformConfig platform_config_;

    /// \brief The router functor which finds the shortest route for an O/D pair on request.
    RouterFunc router_func_;

    /// \brief The demand generator functor which generates order requests in each cycle.
    DemandGeneratorFunc demand_generator_func_;

    /// \brief The system time in milliseconds starting from 0.
    uint64_t system_time_ms_ = 0;

    /// \brief The cycle time im milliseconds.
    uint64_t cycle_ms_ = 0;

    /// \brief The frame time im milliseconds (identical to cycle time if render_video is off).
    uint64_t frame_ms_ = 0;

    /// \brief The main simulation start time in milliseconds.
    uint64_t main_sim_start_time_ms_ = 0;

    /// \brief The main simulation end time in milliseconds.
    uint64_t main_sim_end_time_ms_ = 0;

    /// \brief The time when the simulation is terminated in milliseconds.
    uint64_t system_shutdown_time_ms_ = 0;

    /// \brief The vector of orders created during the entire simulation process.
    std::vector<Order> orders_ = {};

    /// \brief The vector of vehicles.
    std::vector<Vehicle> vehicles_ = {};

    /// \brief The method used to assign orders to vehicles.
    DispatcherMethod dispatcher_ = DispatcherMethod::GI;

    /// \brief The method used to reposition idle vehicles.
    RebalancerMethod rebalancer_ = RebalancerMethod::NONE;

    /// \brief The ofstream that outputs to the datalog.
    std::ofstream datalog_ofstream_;
};

// Implementation is put in a separate file for clarity and maintainability.
#include "platform_impl.hpp"
