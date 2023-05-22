cmake_minimum_required(VERSION          3.10)
cmake_policy(SET CMP0048                NEW) 


project(opengjk VERSION 1.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# check if you are in the top-level project or not
if (NOT CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    message(STATUS 
    "This project has a top-level one called 
    [${CMAKE_PROJECT_NAME}]")
else()
    message(STATUS 
    "This project is a top-level one")
endif()

add_library(opengjk 
STATIC
    ${PROJECT_SOURCE_DIR}/openGJK.c
)

target_include_directories(opengjk 
PUBLIC 
	${PROJECT_SOURCE_DIR}/include
	${CMAKE_BINARY_DIR}
)

target_link_libraries(opengjk PRIVATE m)

set (LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../lib)
