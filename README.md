# `AMoD2` 
An autonomous mobility-on-demand simulator, based on [mod-abm-2.0](https://github.com/wenjian0202/mod-abm-2.0). This simulator uses map data and taxi data from Manhattan.


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


