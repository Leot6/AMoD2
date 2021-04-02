FROM ubuntu:focal

ENV MAP_URL=https://raw.githubusercontent.com/KevinLADLee/KevinLADLee.github.io/master/hongkong.osm.pbf
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install --no-install-recommends -y build-essential sudo curl wget apt-transport-https ca-certificates locales git vim \
    && rm -rf /var/lib/apt/lists/* /tmp/*

RUN apt-get update \
    && apt-get install --no-install-recommends -y \
        libboost-all-dev \
        libxml2-dev \
        ffmpeg \
        cmake \
        libzip-dev \
        libtbb-dev \
        ccache \
        gdal-bin \
        libreadline-dev \
        libbz2-dev \
        pkg-config \
    && rm -rf /var/lib/apt/lists/* /tmp/* 

ADD . /opt/mod-abm-2.0

WORKDIR /opt

RUN mkdir osrm \
    && cd osrm \
    && curl -R -O http://www.lua.org/ftp/lua-5.3.6.tar.gz \
    && tar zxf lua-5.3.6.tar.gz \
    && cd lua-5.3.6 \
    && make linux test \
    && make install \
    && cd .. \
    && rm lua-5.3.6.tar.gz 

RUN if [ "x$(nproc)" = "x1" ] ; then export USE_PROC=1 ; else export USE_PROC=$(($(nproc)/2)) ; fi \
    && cd osrm \
    && git clone https://github.com/Project-OSRM/osrm-backend.git \
    && cd osrm-backend \
    && mkdir build \
    && cd build \
    && cmake .. \
    && cmake --build . -j${USE_PROC} \ 
    && cmake --build . --target install  

RUN cd osrm \
    && mkdir map \
    && cd map \
    && wget -qc $MAP_URL \
    && cd .. \
    && ./osrm-backend/build/osrm-extract ./map/hongkong.osm.pbf -p osrm-backend/profiles/car.lua \
    && ./osrm-backend/build/osrm-partition ./map/hongkong.osrm \
    && ./osrm-backend/build/osrm-customize ./map/hongkong.osrm 

RUN if [ "x$(nproc)" = "x1" ] ; then export USE_PROC=1 ; else export USE_PROC=$(($(nproc)/2)) ; fi && \
    cd /opt/mod-abm-2.0 \
    && cmake -S . -B build \
    && cmake --build build -j${USE_PROC}

# Usage:
# docker run -it --rm --name=mod mod-able:latest bash
# cd /opt/mod-abm-2.0
# ./build/main "./config/platform_demo.yml" "../osrm/map/hongkong.osrm" "./config/demand_demo.yml" 1