cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zmk_feature_trackball_rpc)

if(CONFIG_TRACKBALL_RPC)
  target_sources(app PRIVATE src/trackball_rpc.c)
endif()
