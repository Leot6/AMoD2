//
// Created by Leot on 2021/3/30.
//
#include "utility.hpp"
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

std::time_t getTimeStamp() {
    std::chrono::time_point<std::chrono::system_clock,std::chrono::milliseconds> tp =
            std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
    std::time_t timestamp = tp.time_since_epoch().count();
    return timestamp;
}

void CheckFileExistence(const std::string& path_to_file) {
    struct stat buffer;
    if (stat(path_to_file.c_str(), &buffer) != 0){
        fmt::print("[ERROR] File \"{}\" does not exist! \n", path_to_file);
        exit(0);
    }
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
    auto request_time = row["ptime"].get();
    auto request_time_ms = ComputeTheAccumulatedSecondsFrom0Clock(request_time);
    all_requests.emplace_back(onid,dnid,request_time_ms, request_time);
  }

  fmt::print("[DEBUG] ({}s) Load request data from {}, with {} requests.\n",
             float (getTimeStamp() - s_time)/1000, path_to_csv, all_requests.size());
  return std::move(all_requests);
}

std::vector<Pos> LoadNetworkNodesFromCsvFile(std::string path_to_csv) {
    CheckFileExistence(path_to_csv);
    auto s_time = getTimeStamp();
    std::vector<Pos> all_nodes = {};
    std::ifstream data_csv(path_to_csv); //load the data file
    std::string line;
    getline(data_csv,line);  // ignore the first line
    while (getline(data_csv,line)){  // read every line
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
               float (getTimeStamp() - s_time)/1000, path_to_csv, all_nodes.size());
    return std::move(all_nodes);
}

std::vector<std::vector<size_t>> LoadShortestPathTableFromCsvFile(std::string path_to_csv) {
    CheckFileExistence(path_to_csv);
    auto s_time = getTimeStamp();
    std::vector<std::vector<size_t>> shortest_path_table = {};
    std::ifstream data_csv(path_to_csv); //load the data file
    std::string line;
    getline(data_csv,line);  // ignore the first line
    while (getline(data_csv,line)){  // read every line
        std::istringstream readstr(line); // string every line
        std::vector<std::string> data_line;
        std::string info;
        while (getline(readstr, info, ',')) {
            data_line.push_back(info);
        }
        std::vector<size_t> shortest_path_row = {};
        for (auto i = 1; i < data_line.size(); i++) {
            shortest_path_row.push_back(std::stoi(data_line[i]));
        }
        shortest_path_table.push_back(shortest_path_row);
        if (shortest_path_table[0].size() != shortest_path_table.back().size()) {
            fmt::print("[ERROR]!The size is not right.");
            exit(0);
        }
    }
    fmt::print("[DEBUG] ({}s) Loaded shortest path data from {}, with {} * {} node pairs.\n",
               float (getTimeStamp() - s_time)/1000, path_to_csv,
               shortest_path_table.size(), shortest_path_table[0].size());
    return shortest_path_table;
}

//std::vector<std::vector<float>> LoadMeanTravelTimeTableFromCsvFile(std::string path_to_csv) {
//    CheckFileExistence(path_to_csv);
//    auto s_time = getTimeStamp();
//    TIMER_START(read_csv);
//    std::vector<std::vector<float>> mean_travel_time_table = {};
//    std::ifstream data_csv(path_to_csv); //load the data file
//    std::string line;
//    getline(data_csv,line);  // ignore the first line
//    while (getline(data_csv,line)){  // read every line
//        std::istringstream readstr(line); // string every line
//        std::vector<std::string> data_line;
//        std::string info;
//        while (getline(readstr, info, ',')) {
//            data_line.push_back(info);
//        }
//        std::vector<float> mean_travel_time_row = {};
//        for (auto i = 1; i < data_line.size(); i++) {
//            mean_travel_time_row.push_back(std::stof(data_line[i]));
//        }
//        mean_travel_time_table.push_back(mean_travel_time_row);
//        if (mean_travel_time_table[0].size() != mean_travel_time_table.back().size()) {
//            fmt::print("[ERROR]!The size is not right.");
//            exit(0);
//        }
//    }
//    TIMER_END(read_csv);
//    fmt::print("[DEBUG] ({}s) Loaded shortest path data from {}, with {} * {} node pairs.\n",
//               float (getTimeStamp() - s_time)/1000, path_to_csv,
//               mean_travel_time_table.size(), mean_travel_time_table[0].size());
//    return mean_travel_time_table;
//}

//std::vector<std::vector<float>> LoadMeanTravelTimeTableFromCsvFile(std::string path_to_csv){
//  CheckFileExistence(path_to_csv);
//
//  std::vector<std::vector<float>> mean_travel_time_table = {};
//  rapidcsv::Document csv_file(path_to_csv, rapidcsv::LabelParams(0, 0));
//  std::cout << "CSV: " << csv_file.GetRowCount() << " x " <<   csv_file.GetColumnCount() << std::endl;
//  auto colnum = csv_file.GetColumnCount();
//  auto rownum = csv_file.GetRowCount();
//  mean_travel_time_table.resize(rownum);
//  TIMER_START(read_csv);
//  for(int i = 0; i < rownum; i++){
////    mean_travel_time_table.resize(csv_file.GetColumnCount());
//    mean_travel_time_table.at(i) = csv_file.GetRow<float>(i);
////    std::cout << i << std::endl;
//  }
//  TIMER_END(read_csv);
//
//
//  return mean_travel_time_table;
//}

std::vector<std::vector<float>> LoadMeanTravelTimeTableFromCsvFile(std::string path_to_csv){
  CheckFileExistence(path_to_csv);

  std::vector<std::vector<float>> mean_travel_time_table = {};
  TIMER_START(read_csv);
  using namespace csv;
  csv::CSVReader csv_reader(path_to_csv);
  for (CSVRow& row: csv_reader) { // Input iterator
    std::vector<float> float_row;
    float_row.reserve(row.size());
    long i = 0;
    for (CSVField& field: row) {
      if(i==0){
        i++;
        continue;
      }
      float_row.push_back(field.get<float>());
    }
    mean_travel_time_table.push_back(float_row);
  }
  std::cout << "Table: " << mean_travel_time_table.size() << " x " << mean_travel_time_table[0].size() << std::endl;
  TIMER_END(read_csv);


  return mean_travel_time_table;
}