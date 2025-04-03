# CMake generated Testfile for 
# Source directory: /home/della/drone++/src/includes/ns3/ns-allinone-3.40/ns-3.40/scratch
# Build directory: /home/della/drone++/src/includes/ns3/ns-allinone-3.40/ns-3.40/cmake-cache/scratch
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(ctest-scratch_scratch-simulator "ns3.40-scratch-simulator-default")
set_tests_properties(ctest-scratch_scratch-simulator PROPERTIES  WORKING_DIRECTORY "/home/della/drone++/src/includes/ns3/ns-allinone-3.40/ns-3.40/build/scratch/" _BACKTRACE_TRIPLES "/home/della/drone++/src/includes/ns3/ns-allinone-3.40/ns-3.40/build-support/macros-and-definitions.cmake;1655;add_test;/home/della/drone++/src/includes/ns3/ns-allinone-3.40/ns-3.40/build-support/macros-and-definitions.cmake;1730;set_runtime_outputdirectory;/home/della/drone++/src/includes/ns3/ns-allinone-3.40/ns-3.40/scratch/CMakeLists.txt;57;build_exec;/home/della/drone++/src/includes/ns3/ns-allinone-3.40/ns-3.40/scratch/CMakeLists.txt;69;create_scratch;/home/della/drone++/src/includes/ns3/ns-allinone-3.40/ns-3.40/scratch/CMakeLists.txt;0;")
add_test(ctest-scratch_subdir_scratch-subdir "ns3.40-scratch-subdir-default")
set_tests_properties(ctest-scratch_subdir_scratch-subdir PROPERTIES  WORKING_DIRECTORY "/home/della/drone++/src/includes/ns3/ns-allinone-3.40/ns-3.40/build/scratch/subdir/" _BACKTRACE_TRIPLES "/home/della/drone++/src/includes/ns3/ns-allinone-3.40/ns-3.40/build-support/macros-and-definitions.cmake;1655;add_test;/home/della/drone++/src/includes/ns3/ns-allinone-3.40/ns-3.40/build-support/macros-and-definitions.cmake;1730;set_runtime_outputdirectory;/home/della/drone++/src/includes/ns3/ns-allinone-3.40/ns-3.40/scratch/CMakeLists.txt;57;build_exec;/home/della/drone++/src/includes/ns3/ns-allinone-3.40/ns-3.40/scratch/CMakeLists.txt;99;create_scratch;/home/della/drone++/src/includes/ns3/ns-allinone-3.40/ns-3.40/scratch/CMakeLists.txt;0;")
subdirs("nested-subdir")
