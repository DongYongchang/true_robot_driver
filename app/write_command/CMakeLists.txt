# use cmake 3 which supports targets
cmake_minimum_required(VERSION 3.1.3)

# ===== PROJECT CONFIG =====
project(write_cmd)

set(PROJECT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${PROJECT_DIR}/install/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${PROJECT_DIR}/install/bin")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CONFIGURATION_TYPES "Release")

message(${CMAKE_PREFIX_PATH})


add_definitions(-DCODEIT_1_3_0)
add_definitions(-DCPP_RPC_LIB)

#find_package(truerobot REQUIRED PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../../install NO_DEFAULT_PATH)
find_package(truerobot_cmd REQUIRED)


# ===== SETUP EXECUTABLE =====

# ===== CREATE EXECUTABLE FROM SOURCE =====
# Include app and library headers
include_directories(
	
)

file(GLOB_RECURSE  SRC_FILES 
	${CMAKE_CURRENT_SOURCE_DIR}/*.cc
	${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc
)

add_executable(${PROJECT_NAME} ${SRC_FILES} )

# ===== LINK DEPENDENCIES ======
target_link_libraries(${PROJECT_NAME} 
	truerobot_cmd
	
	pthread
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "CO_SINGLE_THREAD")

# set cmake compile features and options, this replaces the CMAKE_CXX_FLAGS
# for c++ version: cxx_std_11, cxx_std_14, cxx_std_17
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)

#target_compile_options(${PROJECT_NAME} PRIVATE -Werror)
set(CMAKE_VERBOSE_MAKEFILE ON)

# visual studio 19
add_compile_definitions(__STDC_LIMIT_MACROS)

