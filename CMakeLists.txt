cmake_minimum_required(VERSION 2.8.3)
project(octree_registration)

add_definitions(-w -Wunknown-pragmas -Wunused-function -Wdeprecated-declarations)

############# DEPENDENCIES ####################
# PCL
find_package(PCL 1.10 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# OpenMP
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

################## Include and libraries #####################
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Octree)
include_directories("/usr/include/python3.8")

file(GLOB_RECURSE ALL_SRCS "Octree/*.cpp")
file(GLOB_RECURSE MAIN_SRCS "Octree/*main.cpp")
list(REMOVE_ITEM ALL_SRCS ${MAIN_SRCS})
list(REMOVE_ITEM ALL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/Octree/kitti_bin_to_pcd.cpp)

if(UNIX)
    message("current platform : Linux")
    list(REMOVE_ITEM ALL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/Octree/graph2d.cpp)
endif()

list(REMOVE_ITEM ALL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/Octree/octree_rec.cpp)

#################### Executable #########################
add_executable(octreeTest Octree/octreeTest_main.cpp ${ALL_SRCS})
target_link_libraries(octreeTest ${PCL_LIBRARIES} )

add_executable(singleTest Octree/Comparision_main.cpp ${ALL_SRCS})
target_link_libraries(singleTest ${PCL_LIBRARIES} )

add_executable(kitti_btop Octree/kitti_bin_to_pcd.cpp ${ALL_SRCS})
target_link_libraries(kitti_btop ${PCL_LIBRARIES} )

