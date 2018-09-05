cmake_minimum_required(VERSION 2.8.3)
project(gbx_imu)

#NOTE: Eigen includes are different in the snapdragon and odroid.  Snap uses Eigen includes, Odroid uses Eigen3 includes.

#list(APPEND CMAKE_INCLUDE_PATH "/opt/ros/kinetic/share/cmake_modules/cmake/Modules/")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  tf2_ros
  cmake_modules
  nav_msgs
  gbx_ros_bridge_msgs
  message_generation
)

#Check for OpenMP, workaround for Eigen library multithreading compile-time bug
find_package(OpenMP)
if(OPENMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    if(MSVC90 OR MSVC10)
        if(MSVC90)
            set(OPENMP_DLL VCOMP90)
        elseif(MSVC10)
            set(OPENMP_DLL VCOMP100)
        endif(MSVC90)
        set(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} /DELAYLOAD:${OPENMP_DLL}D.dll")
        set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /DELAYLOAD:${OPENMP_DLL}.dll")
     endif(MSVC90 OR MSVC10)
elseif(OPENMP_FOUND)
    message (STATUS "OpenMP not found")
endif()
if (MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc /O2 ${SSE_FLAGS}")
else (MSVC)
    set(CMAKE_CXX_FLAGS "-O3 ${CMAKE_CXX_FLAGS} ${SSE_FLAGS}")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -O0 -g ${SSE_FLAGS}")
    set(CMAKE_CXX_FLAGS_RELEASE " ${CMAKE_CXX_FLAGS} -O3 ${SSE_FLAGS}")
endif (MSVC)

if(NOT GBX_VERSION)
    set(GBX_VERSION 4)
endif()


set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(Eigen REQUIRED)
#find_package (Eigen3 3.3 REQUIRED NO_MODULE)

#catkin_package(
#    INCLUDE_DIRS include
#    CATKIN_DEPENDS gbx_ros_bridge_msgs
#    DEPENDS Eigen
#)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
#  ${Eigen3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS std_msgs geometry_msgs gbx_ros_bridge_msgs)

add_executable(gpsImu_node src/main.cpp src/estimationNode.cpp src/gpsCallbacks.cpp src/filterImu.cpp
    src/filterTW.cpp src/mathHelperFunctions.cpp src/gbxStreamEndpointGPSKF.cpp src/rosPlayback.cpp)

target_link_libraries(gpsImu_node 
  SharedPpEngine
  Threads::Threads
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GPSTK_LIBRARIES}
  ${Eigen_LIBRARIES}
  )
#target_link_libraries(gpsImu_node Eigen3::Eigen)

set_property(TARGET gpsImu_node PROPERTY CXX_STANDARD 11)
set_property(TARGET gpsImu_node PROPERTY CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

install(FILES
  launch/gpsGbx.launch
#   # myfile1
#   # myfile2
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

