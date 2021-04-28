# `AMoD2` 
An autonomous mobility-on-demand simulator, based on [mod-abm-2.0](https://github.com/wenjian0202/mod-abm-2.0). This simulator uses map data and taxi data from Manhattan. Three vehicle dispatch algorithms and one simple rebalancing algorithm are implemented. 
- Dispatcher
  - Greedy Insertion (GR) [[1]](https://github.com/Leot6/AMoD#references): It assigns orders sequentially to the best available vehicle in a first-in-first-out manner (i.e., an exhaustive version of [[2]](https://github.com/Leot6/AMoD#references)).
  - Single-Request Batch Assignment (SBA) [[3]](https://github.com/Leot6/AMoD#references): It takes the new orders for a batch period and assigns them together in a one-to-one match manner, where at most one new order is assigned to a single vehicle.
  - Optimal Schedule Pool (OSP): It takes all picking and pending orders for a batch period and assigns them together in a multi-to-one match manner, where multiple orders (denoted by trip) can be assigned to a single vehicle. Trips are also allowed to be reassigned to different vehicles for better system performance. OSP is an improved version of Request Trip Vehicle (RTV) assignment [[4]](https://github.com/Leot6/AMoD#references), it computes all possible vehicle-trip pairs along with the optimal schedule of each pair. The computation of the optimal schedule ensures that no feasible trip is mistakenly ignored. Based on this complete feasible solution space (called optimal schedule pool, each optimal schedule representing a vehicle-trip pair), the optimal assignment policy could be found by an ILP solver.
- Rebalancer
  - Naive rebanlancer (NR) [[4]](https://github.com/Leot6/AMoD#references): It reposition idle vehicles to the locations of unassigned orders, under the assumption that it is likely that more requests occur in the same area where all requests cannot be satisfied.


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

### Code File Structure

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
Then install `gurobi` working as an ILP solver, please refer to the [Gurobi officical website](https://www.gurobi.com/downloads/) and download the suitable Gurobi Optimizer according to your operation system. (Do not forget to change the directory/version info of `gurobi`.) You can also use other optimization solvers, such as [CPLEX](https://www.ibm.com/analytics/cplex-optimizer), [SCIP](https://www.scipopt.org/)(non-commercial), [CLP](https://github.com/coin-or/Clp)(non-commercial). ([BENCHMARKS FOR OPTIMIZATION SOFTWARE](http://plato.asu.edu/bench.html) could helps choosing solver.)

AMoD2 can be run without the ILP solver, simply using function `GreedyAssignment(...)` to replace function `IlpAssignment(...)` (used in `dispatche_sba_impl.hpp` and `dispatche_osp_impl.hpp`), and commenting the codes related to `gurobi` in `ilp_assign.cpp` and `CMakeLists.txt`. Do note that using `GreedyAssignment(...)` surely will yielding worse performance.

### References
1. Tong, Y., Zeng, Y., Zhou, Z., Chen, L., Ye, J. and Xu, K., 2018. [A unified approach to route planning for shared mobility](https://ink.library.smu.edu.sg/cgi/viewcontent.cgi?article=5889&context=sis_research). Proceedings of the VLDB Endowment, 11(11), p.1633.
2. Ma, S., Zheng, Y. and Wolfson, O., 2013, April. [T-share: A large-scale dynamic taxi ridesharing service](https://www.db.ics.keio.ac.jp/seminar/2013/20131126_kita/Taxi%20ridesharing.pdf). In 2013 IEEE 29th International Conference on Data Engineering (ICDE) (pp. 410-421). IEEE.
3. Simonetto, A., Monteil, J. and Gambella, C., 2019. [Real-time city-scale ridesharing via linear assignment problems](https://arxiv.org/pdf/1902.10676.pdf). Transportation Research Part C: Emerging Technologies, 101, pp.208-232.
4. Alonso-Mora, J., Samaranayake, S., Wallar, A., Frazzoli, E. and Rus, D., 2017. [On-demand high-capacity ride-sharing via dynamic trip-vehicle assignment](https://www.pnas.org/content/114/3/462.short). Proceedings of the National Academy of Sciences, 114(3), pp.462-467
