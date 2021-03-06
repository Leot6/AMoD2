/// \author Jian Wen
/// \date 2021/01/29

#pragma once

#include "config.hpp"

#include <yaml-cpp/yaml.h>
#include <string>

#include <fmt/format.h>
#include <fmt/ranges.h>

#undef NDEBUG
#include <assert.h>



//////////////////////////////////////////////////////////////////////////////////////////////////
/// Geo Types
//////////////////////////////////////////////////////////////////////////////////////////////////

/// \brief Position encoded in longitude/latitude.
/// \details lon in [-180, 180), lat in [-90, 90].
struct Pos {
    size_t node_id = 1;      // Note: the node id starts from 1, for the provided manhattan data.
    float lon = 0.0;
    float lat = 0.0;
};

/// \brief Step of route consisting of distance, duration and a vector of continuous positions.
struct Step {
    int32_t distance_mm = 0;
    int32_t duration_ms = 0;
    std::vector<Pos> poses;     // A step always has two poses, indicating the start and the end nodes.
};


/// \brief Route consisting of total distance, total duration as well as a vector of steps.
struct Route {
    int32_t distance_mm = 0;
    int32_t duration_ms = 0;
    //The last step of a route is always consisting of 2 identical points as a flag of the end of the leg.
    std::vector<Step> steps;
};

/// \brief The type of the routing call.
enum class RoutingType {
    TIME_ONLY, // only return the total travel time and distance
    FULL_ROUTE // return the full route with detailed maneuvers
};

//////////////////////////////////////////////////////////////////////////////////////////////////
/// Order Types
//////////////////////////////////////////////////////////////////////////////////////////////////

/// \brief The order request generated by the demand generator.
/// \param request_time_ms The request sent time relative to the start of the simulation, starting from 0.
/// \param request_time_date The request sent time, same as recorded in the real word.
/// note: request_time_ms = request_time_date - simulation_start_time_date
struct Request {
  size_t origin_node_id;
  size_t destination_node_id;
  uint64_t request_time_ms;
  std::string request_time_date = "0000-00-00 00:00:00";
};

/// \brief The status of the order.
enum class OrderStatus {
    PENDING,     // the order has been generated by the demand generator and is currently in the waiting queue
    PICKING,     // the order has been assigned a vehicle and will be picked up later
    ONBOARD,     // the order has been picked up and is currently onboard
    COMPLETE,    // the order has been dropped off, order completed
    WALKAWAY     // the order isn't served due to constraints in dispatch time/pickup time etc
};

inline std::string order_status_to_string(const OrderStatus &s) {
    if (s == OrderStatus::PENDING) {
        return "PENDING";
    } else if (s == OrderStatus::PICKING) {
        return "PICKING";
    } else if (s == OrderStatus::ONBOARD) {
        return "ONBOARD";
    } else if (s == OrderStatus::COMPLETE) {
        return "COMPLETE";
    } else if (s == OrderStatus::WALKAWAY) {
        return "WALKAWAY";
    }
    assert(false && "Bad OrderStatus type!");
    return "";
}

/// \brief The order that the simulation managed, containing all relavant data.
/// \param request_time_ms Same as the definition in Request.
/// \param request_time_date Same as the definition in Request.
struct Order {
    size_t id;  // Note: the order id starts from 0, equaling to its idx.
    Pos origin;
    Pos destination;
    OrderStatus status = OrderStatus::PENDING;
    int32_t request_time_ms = 0;
    std::string request_time_date = "0000-00-00 00:00:00";
    int32_t shortest_travel_time_ms = 0;
    int32_t max_pickup_time_ms = 0;
    int32_t max_dropoff_time_ms = 0;
    int32_t pickup_time_ms = 0;
    int32_t dropoff_time_ms = 0;
};

//////////////////////////////////////////////////////////////////////////////////////////////////
/// Vehicle Types
//////////////////////////////////////////////////////////////////////////////////////////////////

/// \brief The operation associated with a waypoint.
enum class WaypointOp {
    PICKUP,    // we pick up a order at this waypoint
    DROPOFF,   // we drop off a order at this waypoint
    REPOSITION // we reposition a idle vehicle to this waypoint
};

/// \brief The waypoint represents a stop along the way when the vehicle serves orders.
struct Waypoint {
    Pos pos;
    WaypointOp op;
    size_t order_id;
    Route route;   // the route from the previous waypoint's pos to this waypoint's pos
};

/// \brief The operation associated with a waypoint.
enum class VehicleStatus {
    IDLE,          // the vehicle has no task and stays stationary
    WORKING,       // the vehicle is serving orders
    REBALANCING    // the vehicle is moving towards a reposition location to improve future performance
};

inline std::string vehicle_status_to_string(const VehicleStatus &s) {
    if (s == VehicleStatus::IDLE) {
        return "IDLE";
    } else if (s == VehicleStatus::WORKING) {
        return "WORKING";
    } else if (s == VehicleStatus::REBALANCING) {
        return "REBALANCING";
    }
    assert(false && "Bad VehicleStatus type!");
    return "";
}

/// \brief The vehicle type that holds dispatched orders and a schedule to serve them.
struct Vehicle {
    size_t id;  // Note: the vehicle id starts from 0, equaling to its idx.
    Pos pos;
    VehicleStatus status = VehicleStatus::IDLE;
    bool schedule_has_been_updated_at_current_epoch = false; // false at the start of each epoch,
    // true if vehicle's schedule is rebuilt. Only used in func UpdScheduleForVehiclesHavingOrdersRemoved().
    Step step_to_pos;
    size_t capacity = 1;
    size_t load = 0;
    std::vector<Waypoint> schedule;
    std::vector<size_t> onboard_order_ids;
    uint32_t dist_traveled_mm = 0; // accumulated distance traveled in millimeters (include empty and rebalancing)
    uint32_t loaded_dist_traveled_mm = 0; // accumulated distance traveled, weighted by the load (include empty and rebalancing)
    uint32_t empty_dist_traveled_mm = 0; // accumulated distance traveled, when no passenger onboard (not include rebalancing)
    uint32_t rebl_dist_traveled_mm = 0; // accumulated distance traveled, when executing rebalancing tasks
    uint32_t time_traveled_ms = 0; // accumulated time traveled in milliseconds (include empty and rebalancing)
    uint32_t loaded_time_traveled_ms = 0; // accumulated time traveled, weighted by the load (include empty and rebalancing)
    uint32_t empty_time_traveled_ms = 0; // accumulated time traveled, when no passenger onboard (not include rebalancing)
    uint32_t rebl_time_traveled_ms = 0; // accumulated time traveled, when executing rebalancing tasks
    // e.g. a vehicle travels 100 (10+10+80) s, consisting of 10 s rebalancing until get an assignment,
    // 10 s picking (no passenger onboard) and 80 s travelling with passenger onboard.
};

//////////////////////////////////////////////////////////////////////////////////////////////////
/// Dispatch Types
//////////////////////////////////////////////////////////////////////////////////////////////////

/// \brief The dispatch method used to assign orders to vehicles.
enum class DispatcherMethod {
    GI,      // greedy insertion
    SBA,     // single-request batch assignment
    OSP,     // optimal schedule pool
};

/// \brief The rebalancing method used to reposition idle vehicles to high demand area.
enum class RebalancerMethod {
    NONE,      // no rebalancing
    RVS,       // random vehicle station
    NPO,       // nearest pending order
};
