cmake_minimum_required(VERSION 3.0)
project(cli_example)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -O2")
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

include_directories(./ ../)

option(USE_SLAM "Compile with SLAM support" ON)
set(TARGET_ARCH "host" CACHE STRING "Target architecture")
find_package(mobile-cv-suite REQUIRED PATHS ../mobile-cv-suite)

add_subdirectory(../mobile-cv-suite/jsonl-recorder jsonl-recorder)
include_directories(../mobile-cv-suite/jsonl-recorder)

# TODO: Add your Vio implementation here
# find_package(custom-vio REQUIRED)

add_executable(main src/main.cpp)
target_link_libraries(main
  ${OpenCV_LIBS}
  # TODO: Add your Vio implementation here
  # custom-vio
  jsonl-recorder
  mobile-cv-suite::core
)
