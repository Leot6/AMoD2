# `AMoD2` 
An autonomous mobility-on-demand simulator, based on [mod-abm-2.0](https://github.com/wenjian0202/mod-abm-2.0). This simulator uses map data and taxi data from Manhattan. Three vehicle dispatch algorithms and one simple rebalancing algorithm are implemented. 
- Dispatcher
  - Greedy Insertion (GR) [[1]](https://github.com/Leot6/AMoD#references): It assigns orders sequentially to the best available vehicle in a first-in-first-out manner (i.e., an exhaustive version of [[2]](https://github.com/Leot6/AMoD#references)).
  - Single-Request Batch Assignment (SBA) [[3]](https://github.com/Leot6/AMoD#references): It takes the new orders for a batch period and assigns them together in a one-to-one match manner, where at most one new order is assigned to a single vehicle.
  - Optimal Schedule Pool (OSP): It takes all picking and pending orders for a batch period and assigns them together in a multi-to-one match manner, where multiple orders (denoted by trip) can be assigned to a single vehicle. Trips are also allowed to be reassigned to different vehicles for better system performance. OSP is an improved version of Request Trip Vehicle (RTV) assignment [[4]](https://github.com/Leot6/AMoD#references), it computes all possible vehicle-trip pairs along with the optimal schedule of each pair. The computation of the optimal schedule ensures that no feasible trip is mistakenly ignored. Based on this complete feasible solution space (called optimal schedule pool, each optimal schedule representing a vehicle-trip pair), the optimal assignment policy could be found by an ILP solver.
- Rebalancer
  - Naive rebanlancer (NR) [[4]](https://github.com/Leot6/AMoD#references): It reposition idle vehicles to the locations of unassigned orders, under the assumption that it is likely that more requests occur in the same area where all requests cannot be satisfied.


### An Example Simulation Result
The following simulation result is from a senariao with 400k requests during the whole day. A 30 mins warm up phase and a 30 mins cool down phase are considered.
```
[INFO] Loaded the platform configuration yaml file from AMoD2/config/platform_demo.yml.
[INFO] Router is ready.  (6.03s)
[INFO] Demand Generator is ready.  (2.05s)
[INFO] Platform is ready.
-------------------------------------------------------------------------------------------------
 AMoD: 100.0% ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿2880/2880 [04:29<00:00|10.7Hz]
[INFO] Simulation completed. Creating report.
-------------------------------------------------------------------------------------------------
# Simulation Runtime
  - Start: 2021-04-28 23:47:43, End: 2021-04-28 23:52:13, Time: 0:04:29.
  - Main Simulation: init_time = 8.08 s, runtime = 0:04:19, avg_time = 0.09 s.
# System Configurations
  - From 00:00:00 to 00:00:00. (main simulation between 00:30:00 and 23:30:00).
  - Fleet Config: size = 2000, capacity = 6. (60 + 2760 + 60 = 2880 epochs).
  - Order Config: density = 1 (400k), max_wait = 300 s. (Δt = 30 s).
  - Dispatch Config: dispatcher = SBA, rebalancer = NR.
  - Video Config: false, frame_length = 10 s, fps = 20, duration = 414 s.
# Orders (362614/390679)
  - complete = 362557 (92.80%), onboard = 57 (0.01%), total_service = 362614 (92.82%).
  - avg_shortest_travel = 610.74 s, avg_wait = 134.47 s, avg_delay = 264.81 s.
# Vehicles (2000)
  - Service Distance: total_dist = 706894.42 km, avg_dist = 353.45 km.
  - Service Duration: avg_time = 67711.42 s (81.78% of the main simulation time).
  - Empty Travel: avg_time = 1968.11 s (2.91%), avg_dist = 0.33 km (0.09%).
  - Rebl Travel: avg_time = 61.60 s (0.09%), avg_dist = 1.92 km (0.54%).
  - Load: average_load_dist = 1.85, average_load_time = 1.97.
-------------------------------------------------------------------------------------------------
```


### Build `AMoD2`

To run a simulation, download data files from [this onedrive link](https://1drv.ms/u/s!AsqflzzqZj9qg-8-rT_CpBIZhc2pzw?e=TtYGfD) or [this google drive link](https://drive.google.com/drive/folders/1Q0ZK3c8B8tjd7vO5UsgKXCJPDVr8mGVt?usp=sharing), and put the downloaded folders in the root directory.
```
|-- AMoD2
   |-- datalog-gitignore
   |-- media-gitignore
```

To configure the build system, run once:
```
cmake -S . -B build
```
Future builds can be run as simply as:
```
cmake --build build
```

Once the build is complete, try the exmaple command line that runs the demo simulation:
```
./build/main
```
Or:
```
./build/main "./config/platform_demo.yml"
```

If two flags, `output_datalog` and `render_video`, in platform config (a `.yml` file) are turned on, then the statuses of vehicles and orders are outputed at `datalog/demo.yml`, which can be processed to generate animation video by:
```
python3 ./python/fetch_map.py
```

### Code File Structure
<img src="https://github.com/Leot6/AMoD2/blob/main/doc/code%20file%20structure.png" width="720">


### Dependencies

Install [Homebrew](https://brew.sh/), a software package manager that helps install all other dependencies.
```
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```
Install `wget` (and verify `brew` is working properly):
```
brew install wget
```
And install all other dependencies (At least cmake 3.14.5 is needed to support CMake command "FetchContent_MakeAvailable"):
```
brew install git 
brew install cmake python ffmpeg
brew install boost libzip libxml2 tbb ccache GDAL
```
Finially install `gurobi` working as an ILP solver, please refer to the [Gurobi officical website](https://www.gurobi.com/downloads/) and download the suitable Gurobi Optimizer according to the operation system. (Do not forget to change the directory/version info of `gurobi` in `CMakeLists.txt`.) Other optimization solvers will also work, such as [CPLEX](https://www.ibm.com/analytics/cplex-optimizer), [SCIP](https://www.scipopt.org/)(non-commercial) and [CLP](https://github.com/coin-or/Clp)(non-commercial). ([BENCHMARKS FOR OPTIMIZATION SOFTWARE](http://plato.asu.edu/bench.html) could help choosing solver.)

AMoD2 can run without the ILP solver, simply using function `GreedyAssignment(...)` to replace function `IlpAssignment(...)` (used in `dispatche_sba_impl.hpp` and `dispatche_osp_impl.hpp`), and commenting the codes related to `gurobi` in `ilp_assign.cpp` and `CMakeLists.txt`. Do note that using `GreedyAssignment(...)` surely will yield worse performance.

### Acknowledgment
Special thanks to [wenjian0202](https://github.com/wenjian0202) and [KevinLADLee](https://github.com/KevinLADLee).

### References
1. Tong, Y., Zeng, Y., Zhou, Z., Chen, L., Ye, J. and Xu, K., 2018. [A unified approach to route planning for shared mobility](https://ink.library.smu.edu.sg/cgi/viewcontent.cgi?article=5889&context=sis_research). Proceedings of the VLDB Endowment, 11(11), p.1633.
2. Ma, S., Zheng, Y. and Wolfson, O., 2013, April. [T-share: A large-scale dynamic taxi ridesharing service](https://www.db.ics.keio.ac.jp/seminar/2013/20131126_kita/Taxi%20ridesharing.pdf). In 2013 IEEE 29th International Conference on Data Engineering (ICDE) (pp. 410-421). IEEE.
3. Simonetto, A., Monteil, J. and Gambella, C., 2019. [Real-time city-scale ridesharing via linear assignment problems](https://arxiv.org/pdf/1902.10676.pdf). Transportation Research Part C: Emerging Technologies, 101, pp.208-232.
4. Alonso-Mora, J., Samaranayake, S., Wallar, A., Frazzoli, E. and Rus, D., 2017. [On-demand high-capacity ride-sharing via dynamic trip-vehicle assignment](https://www.pnas.org/content/114/3/462.short). Proceedings of the National Academy of Sciences, 114(3), pp.462-467
