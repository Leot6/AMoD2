//
// Created by Leot on 2021/3/30.
//
#include "utility_functions.hpp"
#include <iomanip>

int32_t ConvertTimeDateToSeconds(std::string time_date) {
    tm tm_time1;
    time_t time_sec1;
    tm_time1.tm_isdst = -1;
    const char *time_date1 = time_date.c_str();
    strptime(time_date1, "%Y-%m-%d %H:%M:%S", &tm_time1);
    time_sec1 = mktime(&tm_time1);
    int32_t converted_time_sec = time_sec1;
    return converted_time_sec;
}

int32_t ComputeTheAccumulatedSecondsFrom0Clock(std::string time_date) {
  std::string time_0_clock = time_date.substr(0,10) + "00:00:00";
  int32_t accumulated_sec = ConvertTimeDateToSeconds(time_date) - ConvertTimeDateToSeconds(time_0_clock);
  return accumulated_sec;
}

std::string ConvertTimeSecondToDate(int32_t time_sec) {
    int32_t time_test = time_sec;
    time_t time_sec2;
    tm tm_time2;
    tm_time2.tm_isdst = -1;
    char time_date2[128]= {0};
    time_sec2 = time_test;
    tm_time2 = *localtime(&time_sec2);
    strftime(time_date2, 64, "%Y-%m-%d %H:%M:%S", &tm_time2);
    std::string converted_time_date = time_date2;
    return converted_time_date;
}

std::time_t getTimeStampMs() {
    std::chrono::time_point<std::chrono::system_clock,std::chrono::milliseconds> tp =
            std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
    std::time_t timestamp_ms = tp.time_since_epoch().count();
    return timestamp_ms;
}

void CheckFileExistence(const std::string& path_to_file) {
    struct stat buffer;
    if (stat(path_to_file.c_str(), &buffer) != 0){
        fmt::print("[ERROR] File \"{}\" does not exist! \n", path_to_file);
        exit(0);
    }
}

void SchedulePrinter(const Vehicle &vehicle, const std::vector<Waypoint> &schedule) {
    fmt::print("[DEBUG] Vehicle #{} ({}) schedule ([node_id, pod, order_id]):",
               vehicle_status_to_string(vehicle.status), vehicle.id);
    for (const auto &wp : schedule) {
        auto pod = 0;
        if (wp.op == WaypointOp::PICKUP) { pod = 1; }
        else if (wp.op == WaypointOp::DROPOFF) { pod = -1; }
        fmt::print(" [{}, {}, {}]", wp.pos.node_id, pod, wp.order_id);
    }
    fmt::print("\n");

}

//void saveTwoDimentionArr2csv(std::vector<std::vector<float>> &arr, int row, int col, char*filename) {
//    std::ofstream outFile;
//    outFile.open(filename);
//    outFile.precision(7);
//
//    outFile << "" << ',';
//    for (int i = 0; i < col; i++) {
//        if (i < col - 1) {
//            outFile << i + 1 << ',';
//        } else {
//            outFile << i + 1;
//        }
//    }
//    outFile << '\n';
//
//    for (int j = 0; j < row; j++) {
//        outFile << j + 1 << ',';  // row index
//        for (int i = 0; i < col; i++) {
//            if (i < col - 1) {
//                outFile << arr[j][i] << ',';
//            } else {
//                outFile << arr[j][i];
//            }
//        }
//        outFile << '\n';
//    }
//    outFile.close();
//}