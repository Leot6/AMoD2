# `AMoD2`
<img src="https://github.com/Leot6/AMoD2/blob/main/doc/demo-100vehs.gif" width="300">

An autonomous mobility-on-demand (AMoD) simulator, based on [mod-abm-2.0](https://github.com/wenjian0202/mod-abm-2.0). It is written in C++, a python implementation is available in [AMoD](https://github.com/Leot6/AMoD). This simulator uses map data and taxi data from Manhattan. Three vehicle dispatch algorithms and one simple rebalancing algorithm are implemented. (The illustration above shows an example instance with 100 six-seat vehicles and 2574 orders received in an hour.) 

- Dispatcher
    - Greedy Insertion (GI) [[1]](https://github.com/Leot6/AMoD2#references): It assigns orders sequentially to the best available vehicle in a first-in-first-out manner (i.e., an exhaustive version of [[2]](https://github.com/Leot6/AMoD2#references)).
    - Single-Request Batch Assignment (SBA) [[3]](https://github.com/Leot6/AMoD2#references): It takes the new orders for a batch period and assigns them together in a one-to-one match manner, where at most one new order is assigned to a single vehicle.
    - Optimal Schedule Pool (OSP) [[4]](https://github.com/Leot6/AMoD2#references): It takes all picking and pending orders received so far and assigns them together in a multi-to-one match manner, where multiple orders (denoted by a trip) can be assigned to a single vehicle. Trips are also allowed to be reassigned to different vehicles for better system performance. OSP is an improved version of Request Trip Vehicle (RTV) assignment [[5]](https://github.com/Leot6/AMoD2#references), it computes all possible vehicle-trip pairs along with the optimal schedule of each pair. The computation of the optimal schedule ensures that no feasible trip is mistakenly ignored. Based on this complete feasible solution space (called optimal schedule pool, each optimal schedule representing a vehicle-trip pair), the optimal assignment policy could be found by an ILP solver.
- Rebalancer
    - Random Vehicle Station (RVS): It repositions idle vehicles randomly to vehicle stations, which uniformly distributed in the city.
    - Nearest Pending Order (NPO): It repositions idle vehicles to the nearest locations of unassigned pending orders, under the assumption that it is likely that more requests occur in the same area where all requests cannot be satisfied.
  

### An Example Simulation Result
The following simulation result is from a scenario with 400k requests during the whole day. A 30 min warm up phase and a 39 min cool down phase are considered.
```
[INFO] Loaded the platform configuration yaml file from AMoD2/config/platform_demo.yml.
[INFO] Router is ready.  (5.396s)
[INFO] Demand Generator is ready.  (1.98s)
[INFO] Platform is ready.
-------------------------------------------------------------------------------------------------
 AMoD: 100.0% ??????????????????????????????????????????????????????????????????????????????????????????2878/2878 [06:25<00:00|7.46Hz]
[INFO] Simulation completed. Creating report.
-------------------------------------------------------------------------------------------------
# Simulation Runtime
  - Start: 2021-08-20 16:40:52, End: 2021-08-20 16:47:18, Time: 0:06:25.
  - Main Simulation: init_time = 7.38 s, runtime = 0:06:07, avg_time = 0.13 s.
# System Configurations
  - From 00:00:00 to 23:59:00. (main simulation between 00:30:00 and 23:20:00).
  - Fleet Config: size = 2000, capacity = 6. (60 + 2740 + 78 = 2878 epochs).
  - Order Config: density = 1 (400k), max_wait = 300 s. (??t = 30 s).
  - Dispatch Config: dispatcher = SBA, rebalancer = NPO.
  - Video Config: false, frame_length = 10 s, fps = 20, duration = 411 s.
# Orders (364232/387512)
  - complete = 364230 (93.99%), onboard = 2 (0.00%), total_service = 364232 (93.99%).
  - avg_shortest_travel = 609.48 s, avg_wait = 124.49 s, avg_delay = 216.20 s.
# Vehicles (2000)
  - Travel Distance: total_dist = 702260.47 km, avg_dist = 351.13 km.
  - Travel Duration: avg_time = 66621.76 s (81.05% of the main simulation time).
  - Empty Travel: avg_time = 2457.29 s (3.69%), avg_dist = 10.99 km (3.13%).
  - Rebl Travel: avg_time = 4594.70 s (6.90%), avg_dist = 33.93 km (9.66%).
  - Travel Load: average_load_dist = 1.78, average_load_time = 1.91.
-------------------------------------------------------------------------------------------------
```


### Build `AMoD2`

Download code and data files from the [releases](https://github.com/Leot6/AMoD2/releases). Data files should be located in the root directory of the code.
```
|-- AMoD2
   |-- datalog-gitignore
   |-- media-gitignore
```

To configure the build system, go to the root directory `AMoD2` and run once:
```
cmake -S . -B build
```
Future builds can be run as simply as:
```
cmake --build build
```

Once the build is complete, try the exmaple command line that runs the demo simulation:
```
# load the default config file
./build/main

# or specify a config file
./build/main "./config/platform_demo.yml"
```

If two flags, `output_datalog` and `render_video`, in platform config (a `.yml` file) are turned on, the statuses of vehicles and orders will be outputed at `datalog/demo.yml`, which can be processed to generate animation video by:
```
# load the default config file
python3 ./python/render_video.py

# or specify a config file
python3 ./python/render_video.py "./config/platform_demo.yml"
```
### Code Diagram
<img src="https://github.com/Leot6/AMoD2/blob/main/doc/code-diagram.png" width="720">

### Code File Structure
<img src="https://github.com/Leot6/AMoD2/blob/main/doc/code-file-structure.png" width="720">


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
Finially, install `gurobi` working as an ILP solver, please refer to the [Gurobi officical website](https://www.gurobi.com/downloads/) and download the suitable Gurobi Optimizer according to the operation system. Do not forget to change the directory/version info of `gurobi` in `CMakeLists.txt`. Other optimization solvers will also work, such as [CPLEX](https://www.ibm.com/analytics/cplex-optimizer), [SCIP](https://www.scipopt.org/)(non-commercial) and [CLP](https://github.com/coin-or/Clp)(non-commercial). [BENCHMARKS FOR OPTIMIZATION SOFTWARE](http://plato.asu.edu/bench.html) could help choosing solver. Using a different solver needs to re-write the function `IlpAssignment(...)` in `ilp_assign.cpp`, which should be easy referring the api document of the solver.

AMoD2 can run without the ILP solver, simply using the function `GreedyAssignment(...)` to replace the function `IlpAssignment(...)` (used in `dispatche_sba_impl.hpp` and `dispatche_osp_impl.hpp`), and commenting the codes related to `gurobi` in `ilp_assign.cpp` and `CMakeLists.txt`. Do note that using `GreedyAssignment(...)` surely will yield worse performance. Also, using `GreedyAssignment(...)` in OSP with "enable_reoptimization = true" requires enabling the function `UpdScheduleForVehiclesHavingOrdersRemoved()` in `dispatch_osp_impl.hpp`.

### Acknowledgment
Special thanks to [wenjian0202](https://github.com/wenjian0202) and [KevinLADLee](https://github.com/KevinLADLee).

### References
1. Tong, Y., Zeng, Y., Zhou, Z., Chen, L., Ye, J. and Xu, K., 2018. [A unified approach to route planning for shared mobility](https://ink.library.smu.edu.sg/cgi/viewcontent.cgi?article=5889&context=sis_research). Proceedings of the VLDB Endowment, 11(11), p.1633.
2. Ma, S., Zheng, Y. and Wolfson, O., 2013, April. [T-share: A large-scale dynamic taxi ridesharing service](https://www.db.ics.keio.ac.jp/seminar/2013/20131126_kita/Taxi%20ridesharing.pdf). In 2013 IEEE 29th International Conference on Data Engineering (ICDE) (pp. 410-421). IEEE.
3. Simonetto, A., Monteil, J. and Gambella, C., 2019. [Real-time city-scale ridesharing via linear assignment problems](https://arxiv.org/pdf/1902.10676.pdf). Transportation Research Part C: Emerging Technologies, 101, pp.208-232.
4. Li, C., Parker, D. and Hao, Q., 2021. [Optimal Online Dispatch for High-Capacity Shared Autonomous Mobility-on-Demand Systems](https://www.cs.bham.ac.uk/~parkerdx/papers/icra21samod.pdf). In Proc. IEEE International Conference on Robotics and Automation (ICRA'21).
5. Alonso-Mora, J., Samaranayake, S., Wallar, A., Frazzoli, E. and Rus, D., 2017. [On-demand high-capacity ride-sharing via dynamic trip-vehicle assignment](https://www.pnas.org/content/114/3/462.short). Proceedings of the National Academy of Sciences, 114(3), pp.462-467
