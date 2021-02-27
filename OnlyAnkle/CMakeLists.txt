# Generic CMakeLists.txt for making an OpenSim-using executable.
# This shows how to use the provided OpenSimConfig.cmake to locate an OpenSim
# installation on your machine so you can use it from your own code.
# You will most likely want to copy some of these lines into your own
# CMakeLists.txt rather than use this one verbatim.

cmake_minimum_required(VERSION 3.2)
project(gcolo)
set(OpenSimMoco_DIR /home/barako/MOCOBIN/cmake)
set(OpenSim_DIR /home/barako/MOCOBIN/opensim-core/cmake)


SET(CMAKE_CXX_COMPILER /opt/rh/devtoolset-7/root/usr/bin/g++) 
SET(CMAKE_C_COMPILER /opt/rh/devtoolset-7/root/usr/bin/gcc) 

# List your source and header files here.
#set(my_source_files JumpingOptimization.cpp PrescribedController.cpp)
set(my_source_files RegisterTypes_osimMocoJumpGoal.cpp  MocoJumpGoal.cpp colo.cpp  )
set(4links_source_files obuild.cpp)


#set(THREADS_PREFER_PTHREAD_FLAG ON)
#find_package(Threads REQUIRED)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# This depends on OpenSimConfig.cmake being located somewhere predictable
# on your machine. If you have installed it somewhere that CMake won't be
# able to guess, you'll need to tell find_package where to look (with the PATHS
# or HINTS parameters), or specify the installation directory in the
# CMAKE_PREFIX_PATH variable.
find_package(OpenSim REQUIRED)
include_directories(/home/barako/MOCOBIN/sdk/include /home/barako/MOCOBIN/opensim-core/sdk/include)
# /home/barako/MOCOBIN/spdlog/include)

add_executable(ccolo ${my_source_files} ${my_header_files})
add_executable(build ${4links_source_files} ${my_header_files})

#add_executable(sliding ${sliding_source_files} ${my_header_files})
#target_link_libraries(moco1 ${OpenSim_LIBRARIES} /usr/lib64/libstdc++.so.6  Threads::Threads)
target_link_libraries(ccolo ${OpenSim_LIBRARIES} /usr/lib64/libstdc++.so.6 /home/barako/MOCOBIN/sdk/lib/libosimMoco.so /home/barako/MOCOBIN/ipopt/lib/libipopt.so ~/MOCOBIN/adol-c/lib64/libadolc.so)
target_link_libraries(build ${OpenSim_LIBRARIES} /usr/lib64/libstdc++.so.6 )

