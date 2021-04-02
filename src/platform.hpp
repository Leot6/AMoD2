/// \author Jian Wen
/// \date 2021/02/01

#pragma once

#include "config.hpp"
#include "types.hpp"
#include "utility.hpp"
#include "vehicle.hpp"

#include <chrono>
#include <ctime>
#include <fstream>

/// \brief The agent-based modeling platform that simulates the mobility-on-demand system.
template <typename RouterFunc, typename DemandGeneratorFunc> class Platform {
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
    void run_simulation();

  private:
    /// \brief Run simulation for one cycle. Invoked repetetively by run_simulation().
    void run_cycle();

    /// \brief Advance all vehicles for the given time and move forward the system time.
    void advance_vehicles(uint64_t time_ms);

    /// \brief Generate orders at the end of each cycle.
    std::vector<size_t> generate_orders();

    /// \brief Dispatch vehicles to serve pending orders.
    void dispatch(const std::vector<size_t> &pending_order_ids);

    /// \brief Write the data of the current simulation state to datalog.
    void write_to_datalog();

    /// \brief Create the report based on the statistical analysis using the simulated data.
    void create_report(double total_runtime_s);

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

    /// \brief The ofstream that outputs to the datalog.
    std::ofstream datalog_ofstream_;
};

// Implementation is put in a separate file for clarity and maintainability.
#include "platform_impl.hpp"
