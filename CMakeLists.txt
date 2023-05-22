cmake_minimum_required(VERSION          3.10)
cmake_policy(SET CMP0048                NEW) 

project(woa_planning_multithreads VERSION 1.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# check if you are in the top-level project or not
if (NOT CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    message(
        STATUS 
        "This project has a top-level one called [${CMAKE_PROJECT_NAME}]"
    )
else()
    message(
        STATUS 
        "This project is a top-level one"
    )
endif()


# add_subdirectory(kinetics)
add_subdirectory(utils)
add_subdirectory(openGJK)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

add_executable(
    ${PROJECT_NAME} 
    src/A_b.cpp
    src/calc_binomial.cpp
    src/calc_fitness_woa.cpp
    src/calc_p.cpp
    src/calc_q_ddot.cpp
    src/calc_q_dot.cpp
    src/calc_r.cpp
    src/delta_var.cpp
    src/forward_kin.cpp
    src/global_data.cpp
    src/J_Base2EE.cpp
    src/J_bm.cpp
    src/links_transform.cpp
    src/read_vertex_file.cpp
    src/main.cpp
)

set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC 
	${PROJECT_SOURCE_DIR}/include
)

link_directories(${PROJECT_SOURCE_DIR}/lib)

# target_link_libraries(${PROJECT_NAME} kinetics)
target_link_libraries(
    ${PROJECT_NAME} 
    utils
)

target_link_libraries(
    ${PROJECT_NAME} 
    opengjk
)

# Find pthread library
find_package(Threads REQUIRED)
# Link pthread library
target_link_libraries(${PROJECT_NAME} Threads::Threads)

# Print path to pthread library
message("PTHREAD_LIBRARY: ${CMAKE_THREAD_LIBS_INIT}")
message(STATUS "Found pthread library: ${CMAKE_THREAD_LIBS_INIT}")

# # Find pthread library
# find_library(PTHREAD_LIB pthread)
# # Print full path to pthread library
# message(STATUS "Found pthread library: ${PTHREAD_LIB}")

# gdb调试
SET(CMAKE_BUILD_TYPE "Debug")  
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")  
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall") 


                           
                         







