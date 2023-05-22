
cmake_minimum_required(VERSION          3.10)
cmake_policy(SET CMP0048                NEW) 

project(utils VERSION 1.0)

# # specify the C++ standard
# set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_STANDARD_REQUIRED True)

# check if you are in the top-level project or not
if (NOT CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    message(STATUS 
    "This project has a top-level one called 
    [${CMAKE_PROJECT_NAME}]")
else()
    message(STATUS 
    "This project is a top-level one")
endif()

aux_source_directory(${PROJECT_SOURCE_DIR}/src DIR_LIB_SRCS)

# 生成链接库
add_library (utils
STATIC    
     ${DIR_LIB_SRCS}
)

target_include_directories(utils 
PUBLIC 
    ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(utils PRIVATE m)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../lib)
