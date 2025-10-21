cmake_minimum_required(VERSION 3.5)
project(drone++)

# Set the C++ standard to C++17
set(CMAKE_CXX_STANDARD 20)

# Set the path to the NS-3 build and include directories
set(NS3_BUILD_DIR "includes/ns3/ns-allinone-3.44/ns-3.44/build/lib")
set(NS3_INCLUDE_DIR "includes/ns3/ns-allinone-3.44/ns-3.44/build/include")

# Include directories
include_directories(${NS3_INCLUDE_DIR})

# Add the .so files as libraries
link_directories(${NS3_BUILD_DIR})

# Add your source files and header files
add_executable(out 
    main.cpp
    drone/Drone.cpp
    mobility/custom-mobility-model.cpp
    energy/energy.cpp
    parser/JsonParser.cpp
)

# Link the necessary NS-3 libraries
target_link_libraries(out
    ns3.44-core-default
    ns3.44-network-default
    ns3.44-internet-default
    ns3.44-point-to-point-default
    ns3.44-applications-default
    ns3.44-wifi-default
    ns3.44-mobility-default
    ns3.44-csma-default
    ns3.44-propagation-default
    ns3.44-netanim-default
    #ns3.40-netsimulyzer-default
    ns3.44-buildings-default
    ns3.44-stats-default
    #ns3.44-mpi-default
    ns3.44-energy-default
    ns3.44-buildings-default
)

# Use mpic++ as the C++ compiler
set(CMAKE_CXX_COMPILER g++)

# Add a custom target to run the executable with MPI
add_custom_target(run_mpi
    COMMAND g++ -n 2 ${CMAKE_CURRENT_BINARY_DIR}/out
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

