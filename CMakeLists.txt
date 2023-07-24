cmake_minimum_required(VERSION 3.10)
project(scan_consist)

set(CMAKE_BUILD_TYPE Debug)

# set the location of FindXXX.cmake modules
# SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

# location of map-consistency
add_subdirectory(libs/map-consistency)

find_package(G2O REQUIRED)
message(STATUS "G2O Found ${G2O_FOUND}")
message(STATUS "G2O INCLUDE_DIR: ${G2O_INCLUDE_DIRS}")

find_package(Eigen3 REQUIRED)
message(STATUS "EIGEN3 Found ${EIGEN3_FOUND}")
message(STATUS "EIGEN3 INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")

include_directories(libs/map-consistency/src)
include_directories(${G2O_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

# main executable
add_executable(main src/main.cpp)

target_link_libraries(main PRIVATE 
    consist 
    ${G2O_TYPES_DATA}
    ${G2O_TYPES_SLAM2D}
    ${G2O_CORE_LIBRARY}
    ${G2O_STUFF_LIBRARY}
    ${G2O_SOLVER_CHOLMOD}
)

# Optionally, if you need to link the GUI libraries for the tester executable:
# target_link_libraries(your_executable_name PRIVATE GUI_LIBRARIES)

# Link the executable with the online package's target
# target_link_libraries(your_executable_name PRIVATE online_package_target)