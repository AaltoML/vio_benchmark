cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM3)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wno-deprecated -O3 -march=native ")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ORB_SLAM3/cmake_modules)

IF (APPLE)
   # Default install location with: brew install opencv@3
   find_package(OpenCV 3 REQUIRED PATHS "/usr/local/opt/opencv@3/")
ELSE()
   find_package(OpenCV 3)
ENDIF()

if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
ORB_SLAM3
ORB_SLAM3/include
ORB_SLAM3/include/CameraModels
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
)

IF (APPLE)
   # Default install locaiton with: brew install boost
   include_directories(/usr/local/include)
ENDIF()

#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY target/lib)

add_library(${PROJECT_NAME} SHARED
ORB_SLAM3/src/System.cc
ORB_SLAM3/src/Tracking.cc
ORB_SLAM3/src/LocalMapping.cc
ORB_SLAM3/src/LoopClosing.cc
ORB_SLAM3/src/ORBextractor.cc
ORB_SLAM3/src/ORBmatcher.cc
ORB_SLAM3/src/FrameDrawer.cc
ORB_SLAM3/src/Converter.cc
ORB_SLAM3/src/MapPoint.cc
ORB_SLAM3/src/KeyFrame.cc
ORB_SLAM3/src/Atlas.cc
ORB_SLAM3/src/Map.cc
ORB_SLAM3/src/MapDrawer.cc
ORB_SLAM3/src/Optimizer.cc
ORB_SLAM3/src/PnPsolver.cc
ORB_SLAM3/src/Frame.cc
ORB_SLAM3/src/KeyFrameDatabase.cc
ORB_SLAM3/src/Sim3Solver.cc
ORB_SLAM3/src/Initializer.cc
ORB_SLAM3/src/Viewer.cc
ORB_SLAM3/src/ImuTypes.cc
ORB_SLAM3/src/G2oTypes.cc
ORB_SLAM3/src/CameraModels/Pinhole.cpp
ORB_SLAM3/src/CameraModels/KannalaBrandt8.cpp
ORB_SLAM3/src/OptimizableTypes.cpp
ORB_SLAM3/src/MLPnPsolver.cpp
ORB_SLAM3/include/System.h
ORB_SLAM3/include/Tracking.h
ORB_SLAM3/include/LocalMapping.h
ORB_SLAM3/include/LoopClosing.h
ORB_SLAM3/include/ORBextractor.h
ORB_SLAM3/include/ORBmatcher.h
ORB_SLAM3/include/FrameDrawer.h
ORB_SLAM3/include/Converter.h
ORB_SLAM3/include/MapPoint.h
ORB_SLAM3/include/KeyFrame.h
ORB_SLAM3/include/Atlas.h
ORB_SLAM3/include/Map.h
ORB_SLAM3/include/MapDrawer.h
ORB_SLAM3/include/Optimizer.h
ORB_SLAM3/include/PnPsolver.h
ORB_SLAM3/include/Frame.h
ORB_SLAM3/include/KeyFrameDatabase.h
ORB_SLAM3/include/Sim3Solver.h
ORB_SLAM3/include/Initializer.h
ORB_SLAM3/include/Viewer.h
ORB_SLAM3/include/ImuTypes.h
ORB_SLAM3/include/G2oTypes.h
ORB_SLAM3/include/CameraModels/GeometricCamera.h
ORB_SLAM3/include/CameraModels/Pinhole.h
ORB_SLAM3/include/CameraModels/KannalaBrandt8.h
ORB_SLAM3/include/OptimizableTypes.h
ORB_SLAM3/include/MLPnPsolver.h
ORB_SLAM3/include/TwoViewReconstruction.h
ORB_SLAM3/src/TwoViewReconstruction.cc)

add_subdirectory(ORB_SLAM3/Thirdparty/g2o)
add_subdirectory(../jsonl-recorder jsonl-recorder)

include_directories(../jsonl-recorder)

find_library(DBOW2 DBoW2 HITS ORB_SLAM3/Thirdparty/DBoW2/lib)
find_library(G2O g2o HITS ORB_SLAM3/Thirdparty/g2o/lib)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${DBOW2}
${G2O}
# TODO: What were these used for?
# -lboost_serialization
# -lcrypto
)

add_executable(orbslam3_runner
src/orbslam3_runner.cc)
target_link_libraries(orbslam3_runner ${PROJECT_NAME} jsonl-recorder)

add_executable(stereo_inertial_euroc
src/stereo_inertial_euroc.cc)
target_link_libraries(stereo_inertial_euroc ${PROJECT_NAME} jsonl-recorder)
