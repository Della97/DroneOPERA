cmake_minimum_required(VERSION 3.0)
project(drone++)

# Set the C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)

# Set the path to the NS-3 build and include directories
set(NS3_BUILD_DIR "includes/ns3/ns-allinone-3.40/ns-3.40/build/lib")
set(NS3_INCLUDE_DIR "includes/ns3/ns-allinone-3.40/ns-3.40/build/include")

# Include directories
include_directories(${NS3_INCLUDE_DIR})

# Add the .so files as libraries
link_directories(${NS3_BUILD_DIR})

# Add your source files and header files
add_executable(out 
    main2.cpp
    drone/Drone.cpp
    mobility/custom-mobility-model.cpp
    energy/energy.cpp
    parser/JsonParser.cpp
)

# Link the necessary NS-3 libraries
target_link_libraries(out
    ns3.40-core-default
    ns3.40-network-default
    ns3.40-internet-default
    ns3.40-point-to-point-default
    ns3.40-applications-default
    ns3.40-wifi-default
    ns3.40-mobility-default
    ns3.40-csma-default
    ns3.40-propagation-default
    ns3.40-netanim-default
    ns3.40-netsimulyzer-default
    ns3.40-buildings-default
    ns3.40-stats-default
    ns3.40-mpi-default
    ns3.40-energy-default
    ns3.40-energy-test-default
    ns3.40-buildings-default
)

# Use mpic++ as the C++ compiler
set(CMAKE_CXX_COMPILER mpic++)

# Add a custom target to run the executable with MPI
add_custom_target(run_mpi
    COMMAND mpiexec -n 2 ${CMAKE_CURRENT_BINARY_DIR}/out
    DEPENDS out
    WORKING_DIRECTORY ${CMAKE_PROJECT_DIR}
)

# Add a custom target to clean the build
add_custom_target(clean-all
    COMMAND ${CMAKE_COMMAND} -E echo "Cleaning build..."
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/CMakeFiles
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/CMakeCache.txt
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/Makefile
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/cmake_install.cmake
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/install_manifest.txt
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/out
)

