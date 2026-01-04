cmake_minimum_required(VERSION 3.10)
project(gmapping_json_server)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# OpenMP support
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP found: ${OpenMP_CXX_VERSION}")
else()
  message(WARNING "OpenMP not found, parallel processing disabled")
endif()

# ARM NEON support for SIMD optimization
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
  message(STATUS "ARM64 detected - enabling NEON SIMD optimizations")
  add_compile_options(-march=armv8-a+simd)
  add_compile_definitions(__ARM_NEON=1)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
  message(STATUS "ARM32 detected - enabling NEON SIMD optimizations")
  add_compile_options(-mfpu=neon -mfloat-abi=hard)
  add_compile_definitions(__ARM_NEON=1)
else()
  message(STATUS "Non-ARM processor (${CMAKE_SYSTEM_PROCESSOR}) - NEON disabled")
endif()

# Build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler warnings
add_compile_options(-Wall -Wextra -Wpedantic)

# ===== Build openslam_gmapping libraries directly =====

set(GMAPPING_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/openslam_gmapping)
set(EXPORT_HEADER_DIR "${CMAKE_CURRENT_BINARY_DIR}/gmapping_export_headers")

include(GenerateExportHeader)

# Create export header directories
file(MAKE_DIRECTORY "${EXPORT_HEADER_DIR}/gmapping/utils")
file(MAKE_DIRECTORY "${EXPORT_HEADER_DIR}/gmapping/sensor/sensor_base")
file(MAKE_DIRECTORY "${EXPORT_HEADER_DIR}/gmapping/sensor/sensor_odometry")
file(MAKE_DIRECTORY "${EXPORT_HEADER_DIR}/gmapping/sensor/sensor_range")
file(MAKE_DIRECTORY "${EXPORT_HEADER_DIR}/gmapping/log")
file(MAKE_DIRECTORY "${EXPORT_HEADER_DIR}/gmapping/configfile")
file(MAKE_DIRECTORY "${EXPORT_HEADER_DIR}/gmapping/scanmatcher")
file(MAKE_DIRECTORY "${EXPORT_HEADER_DIR}/gmapping/gridfastslam")

# utils library
add_library(gmapping_utils STATIC
  ${GMAPPING_SRC_DIR}/utils/stat.cpp
  ${GMAPPING_SRC_DIR}/utils/movement.cpp
)
target_include_directories(gmapping_utils PUBLIC
  ${GMAPPING_SRC_DIR}/include
  ${EXPORT_HEADER_DIR}
)
generate_export_header(gmapping_utils
  EXPORT_FILE_NAME ${EXPORT_HEADER_DIR}/gmapping/utils/utils_export.h
  BASE_NAME utils)

# sensor_base library
add_library(gmapping_sensor_base STATIC
  ${GMAPPING_SRC_DIR}/sensor/sensor_base/sensor.cpp
  ${GMAPPING_SRC_DIR}/sensor/sensor_base/sensorreading.cpp
)
target_include_directories(gmapping_sensor_base PUBLIC
  ${GMAPPING_SRC_DIR}/include
  ${EXPORT_HEADER_DIR}
)
generate_export_header(gmapping_sensor_base
  EXPORT_FILE_NAME ${EXPORT_HEADER_DIR}/gmapping/sensor/sensor_base/sensor_base_export.h
  BASE_NAME sensor_base)

# sensor_odometry library
add_library(gmapping_sensor_odometry STATIC
  ${GMAPPING_SRC_DIR}/sensor/sensor_odometry/odometrysensor.cpp
  ${GMAPPING_SRC_DIR}/sensor/sensor_odometry/odometryreading.cpp
)
target_include_directories(gmapping_sensor_odometry PUBLIC
  ${GMAPPING_SRC_DIR}/include
  ${EXPORT_HEADER_DIR}
)
target_link_libraries(gmapping_sensor_odometry gmapping_sensor_base)
generate_export_header(gmapping_sensor_odometry
  EXPORT_FILE_NAME ${EXPORT_HEADER_DIR}/gmapping/sensor/sensor_odometry/sensor_odometry_export.h
  BASE_NAME sensor_odometry)

# sensor_range library
add_library(gmapping_sensor_range STATIC
  ${GMAPPING_SRC_DIR}/sensor/sensor_range/rangesensor.cpp
  ${GMAPPING_SRC_DIR}/sensor/sensor_range/rangereading.cpp
)
target_include_directories(gmapping_sensor_range PUBLIC
  ${GMAPPING_SRC_DIR}/include
  ${EXPORT_HEADER_DIR}
)
target_link_libraries(gmapping_sensor_range gmapping_sensor_base)
generate_export_header(gmapping_sensor_range
  EXPORT_FILE_NAME ${EXPORT_HEADER_DIR}/gmapping/sensor/sensor_range/sensor_range_export.h
  BASE_NAME sensor_range)

# log library
add_library(gmapping_log STATIC
  ${GMAPPING_SRC_DIR}/log/configuration.cpp
  ${GMAPPING_SRC_DIR}/log/carmenconfiguration.cpp
  ${GMAPPING_SRC_DIR}/log/sensorlog.cpp
  ${GMAPPING_SRC_DIR}/log/sensorstream.cpp
)
target_include_directories(gmapping_log PUBLIC
  ${GMAPPING_SRC_DIR}/include
  ${EXPORT_HEADER_DIR}
)
target_link_libraries(gmapping_log
  gmapping_sensor_range
  gmapping_sensor_odometry
  gmapping_sensor_base
)
generate_export_header(gmapping_log
  EXPORT_FILE_NAME ${EXPORT_HEADER_DIR}/gmapping/log/log_export.h
  BASE_NAME log)

# configfile library
add_library(gmapping_configfile STATIC
  ${GMAPPING_SRC_DIR}/configfile/configfile.cpp
)
target_include_directories(gmapping_configfile PUBLIC
  ${GMAPPING_SRC_DIR}/include
  ${EXPORT_HEADER_DIR}
)
generate_export_header(gmapping_configfile
  EXPORT_FILE_NAME ${EXPORT_HEADER_DIR}/gmapping/configfile/configfile_export.h
  BASE_NAME configfile)

# scanmatcher library
add_library(gmapping_scanmatcher STATIC
  ${GMAPPING_SRC_DIR}/scanmatcher/smmap.cpp
  ${GMAPPING_SRC_DIR}/scanmatcher/scanmatcher.cpp
  ${GMAPPING_SRC_DIR}/scanmatcher/scanmatcherprocessor.cpp
  ${GMAPPING_SRC_DIR}/scanmatcher/eig3.cpp
)
target_include_directories(gmapping_scanmatcher PUBLIC
  ${GMAPPING_SRC_DIR}/include
  ${EXPORT_HEADER_DIR}
)
target_link_libraries(gmapping_scanmatcher
  gmapping_log
  gmapping_sensor_range
  gmapping_sensor_odometry
  gmapping_sensor_base
  gmapping_utils
)
generate_export_header(gmapping_scanmatcher
  EXPORT_FILE_NAME ${EXPORT_HEADER_DIR}/gmapping/scanmatcher/scanmatcher_export.h
  BASE_NAME scanmatcher)

# gridfastslam library
add_library(gmapping_gridfastslam STATIC
  ${GMAPPING_SRC_DIR}/gridfastslam/gridslamprocessor_tree.cpp
  ${GMAPPING_SRC_DIR}/gridfastslam/motionmodel.cpp
  ${GMAPPING_SRC_DIR}/gridfastslam/gridslamprocessor.cpp
  ${GMAPPING_SRC_DIR}/gridfastslam/gfsreader.cpp
)
target_include_directories(gmapping_gridfastslam PUBLIC
  ${GMAPPING_SRC_DIR}/include
  ${EXPORT_HEADER_DIR}
)
target_link_libraries(gmapping_gridfastslam
  gmapping_scanmatcher
  gmapping_log
  gmapping_sensor_range
  gmapping_sensor_odometry
  gmapping_sensor_base
  gmapping_utils
)
# Add OpenMP support for parallel scan matching
if(OpenMP_CXX_FOUND)
  target_link_libraries(gmapping_gridfastslam OpenMP::OpenMP_CXX)
  target_compile_definitions(gmapping_gridfastslam PUBLIC USE_OPENMP)
endif()
generate_export_header(gmapping_gridfastslam
  EXPORT_FILE_NAME ${EXPORT_HEADER_DIR}/gmapping/gridfastslam/gridfastslam_export.h
  BASE_NAME gridfastslam)

# ===== Main application =====

add_executable(gmapping_json_server
  src/main.cpp
  src/gmapping_wrapper.cpp
  src/rosbag_writer.cpp
)

target_include_directories(gmapping_json_server PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${GMAPPING_SRC_DIR}/include
  ${EXPORT_HEADER_DIR}
)

target_link_libraries(gmapping_json_server
  gmapping_gridfastslam
  gmapping_scanmatcher
  gmapping_log
  gmapping_configfile
  gmapping_sensor_range
  gmapping_sensor_odometry
  gmapping_sensor_base
  gmapping_utils
  pthread
)

# Install target
install(TARGETS gmapping_json_server
  RUNTIME DESTINATION bin
)

# Print configuration
message(STATUS "")
message(STATUS "=== GMapping JSON Server Configuration ===")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Processor: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64|arm")
  message(STATUS "NEON SIMD: ENABLED")
else()
  message(STATUS "NEON SIMD: DISABLED (non-ARM)")
endif()
message(STATUS "")
