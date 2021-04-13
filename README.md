# `AMoD2` 
An autonomous mobility-on-demand simulator, based on [mod-abm-2.0](https://github.com/wenjian0202/mod-abm-2.0). This simulator uses map data and taxi data from Manhattan.


### Build `mod-abm-2.0`

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
And install all other dependencies:
```
brew install git 
brew install cmake python ffmpeg
brew install boost libzip libxml2 tbb ccache GDAL
```
At least cmake 3.14.5 is needed to support CMake command "FetchContent_MakeAvailable".
