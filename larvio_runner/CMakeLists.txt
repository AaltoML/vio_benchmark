cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

project(larvio)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/LARVIO/cmake)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(SuiteSparse REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}/LARVIO/include
    ${OpenCV_INCLUDE_DIRS}
    ${SUITESPARSE_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
)

add_subdirectory(../jsonl-recorder jsonl-recorder)
include_directories(../jsonl-recorder)

# Image processor
add_library(image_processor
    ${PROJECT_SOURCE_DIR}/LARVIO/src/image_processor.cpp
    ${PROJECT_SOURCE_DIR}/LARVIO/src/ORBDescriptor.cpp
)
target_link_libraries(image_processor
    ${OpenCV_LIBRARIES}
)

# Initializer
# -- Static Initializer
add_library(staticInitializer
    ${PROJECT_SOURCE_DIR}/LARVIO/src/StaticInitializer.cpp
)
target_link_libraries(staticInitializer
    ${SUITESPARSE_LIBRARIES}
)
# -- Dynamic Initializer
add_library(dynamicInitializer
    ${PROJECT_SOURCE_DIR}/LARVIO/src/StaticInitializer.cpp
    ${PROJECT_SOURCE_DIR}/LARVIO/src/DynamicInitializer.cpp
    ${PROJECT_SOURCE_DIR}/LARVIO/src/feature_manager.cpp
    ${PROJECT_SOURCE_DIR}/LARVIO/src/initial_alignment.cpp
    ${PROJECT_SOURCE_DIR}/LARVIO/src/initial_sfm.cpp
    ${PROJECT_SOURCE_DIR}/LARVIO/src/solve_5pts.cpp
)
target_link_libraries(dynamicInitializer
    ${SUITESPARSE_LIBRARIES}
    ${CERES_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
# -- Flexible Initializer
add_library(flexibleInitializer
    ${PROJECT_SOURCE_DIR}/LARVIO/src/FlexibleInitializer.cpp
)
target_link_libraries(flexibleInitializer
    staticInitializer
    dynamicInitializer
)

# Estimator
add_library(estimator
    ${PROJECT_SOURCE_DIR}/LARVIO/src/larvio.cpp
)
target_link_libraries(estimator
    flexibleInitializer
    ${SUITESPARSE_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

# App
# add_executable(larvio
#     LARVIO/app/larvioMain.cpp)
# target_link_libraries(larvio
#     image_processor
#     estimator
#     ${OpenCV_LIBS}
#     ${Pangolin_LIBRARIES}
# )

# Runner
add_executable(larvio_runner
    src/larvio_runner.cpp)
target_link_libraries(larvio_runner
    image_processor
    estimator
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
    jsonl-recorder
)
