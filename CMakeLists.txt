cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zmk_feature_trackball_rpc)

if(CONFIG_TRACKBALL_RPC)
  target_sources(app PRIVATE src/trackball_rpc.c src/lism_aml_temp_layer.c src/lism_runtime_scroll_invert.c src/lism_accel_cursor.c src/lism_gesture.c)
  # zmk-driver-paw3222 only exposes include/paw3222.h to its own
  # zephyr_library() target (zephyr_library_include_directories), not
  # globally, so we pull in its module dir explicitly.
  target_include_directories(app PRIVATE ${ZEPHYR_ZMK_DRIVER_PAW3222_MODULE_DIR}/include)
endif()

if(CONFIG_LISM_COMBO_RPC)
  target_sources(app PRIVATE src/lism_runtime_combo.c src/lism_combo_rpc.c)
endif()
