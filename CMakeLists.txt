cmake_minimum_required(VERSION 3.10)
project(scan_consist)

# SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

# Add the subdirectory of the map-consistency
add_subdirectory(libs/map-consistency)

find_package(G2O REQUIRED)

message(STATUS "G2O Found ${G2O_FOUND}")
message(STATUS "G2O INCLUDE_DIR: ${G2O_INCLUDE_DIRS}")


find_package(Eigen3 REQUIRED)
message(STATUS "EIGEN3 Found ${EIGEN3_FOUND}")
message(STATUS "EIGEN3 INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")

# Add the include directories of the online package
include_directories(libs/map-consistency/src)
include_directories(${G2O_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

# Add your main executable
add_executable(main src/main.cpp)

# Link the executable with the consist's target
target_link_libraries(main PRIVATE consist)

# Optionally, if you need to link the GUI libraries for the tester executable:
# target_link_libraries(your_executable_name PRIVATE GUI_LIBRARIES)


# Link the executable with the online package's target
# target_link_libraries(your_executable_name PRIVATE online_package_target)