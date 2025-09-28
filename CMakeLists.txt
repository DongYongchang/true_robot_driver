
cmake_minimum_required(VERSION 3.12)

project(truerobot)

set(TrueRbotState "truerobot_state")

set(CMAKE_CONFIGURATION_TYPES Release)
set(CMAKE_CXX_STANDARD 17)
add_definitions(-DCODEIT_1_3_0)

link_directories(${CMAKE_SOURCE_DIR}/lib/linux/Release)

# Add source files
file(GLOB_RECURSE  STATE_SRC_FILES 
	${CMAKE_CURRENT_SOURCE_DIR}/src/state/*.cc
	${CMAKE_CURRENT_SOURCE_DIR}/src/state/*.cpp
)

add_library(${TrueRbotState} STATIC ${STATE_SRC_FILES} )

target_include_directories(${TrueRbotState}
	PRIVATE
	${CMAKE_SOURCE_DIR}/include
	${CMAKE_SOURCE_DIR}/include/state
)

target_include_directories(${TrueRbotState}
	PUBLIC
	${CMAKE_INSTALL_PREFIX}/include
	${CMAKE_INSTALL_PREFIX}/include/state
)

# ===== LINK DEPENDENCIES ======
target_link_directories(${TrueRbotState}
	PUBLIC 
	${CMAKE_INSTALL_PREFIX}/lib
)

target_link_libraries(${TrueRbotState} ${CMAKE_SOURCE_DIR}/lib/linux/Release/libprotobuf.so)
target_link_libraries(${TrueRbotState} ${CMAKE_SOURCE_DIR}/lib/linux/Release/libzmq.so)
target_link_libraries(${TrueRbotState} ${CMAKE_SOURCE_DIR}/lib/linux/Release/libmessage.so)
target_link_libraries(${TrueRbotState} pthread)


# ===== LINK DEPENDENCIES ======
#### Head file install ####
install(DIRECTORY 
	${CMAKE_CURRENT_SOURCE_DIR}/include/state
	DESTINATION ${CMAKE_INSTALL_PREFIX}/include
	FILES_MATCHING 
	PATTERN "*.h"
	PATTERN "*.hpp"
	PATTERN "*.inc"
)

#### Cmake package install #### 
install(TARGETS ${TrueRbotState}
EXPORT "${TrueRbotState}-targets"
RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# copy the *-targets.cmake file to the CMAKE_INSTALL_PREFIX directory
install(EXPORT "${TrueRbotState}-targets"
    FILE "${TrueRbotState}-targets.cmake"
    DESTINATION "lib/cmake/${TrueRbotState}"
    )


install(DIRECTORY "${CMAKE_SOURCE_DIR}/lib/linux/Release/" 
	DESTINATION "bin"
	FILES_MATCHING 
	PATTERN "*"
)

# copy the *.-config file to the CMAKE_INSTALL_PREFIX directory. This will specify the dependencies.
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/${TrueRbotState}-config.cmake.in" 
			 "${TrueRbotState}-config.cmake" @ONLY)
install(FILES 
	"${CMAKE_CURRENT_BINARY_DIR}/${TrueRbotState}-config.cmake"
    DESTINATION 
	"lib/cmake/${TrueRbotState}"
)

# we supply a package wrapper ourselves: cmake/Modules/FindXXX.cmake
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/Modules")


# set cmake compile features and options, this replaces the CMAKE_CXX_FLAGS
# for c++ version: cxx_std_11, cxx_std_14, cxx_std_17
target_compile_features(${TrueRbotState} PRIVATE cxx_std_17)
set_target_properties(${TrueRbotState} PROPERTIES CXX_EXTENSIONS OFF)
#target_compile_options(${PROJECT_NAME} PRIVATE -Werror)
set(CMAKE_VERBOSE_MAKEFILE ON)

#######################################################################################################################3


set(TrueRbotoCmd "truerobot_cmd")

set(CMAKE_CONFIGURATION_TYPES Release)
set(CMAKE_CXX_STANDARD 17)
add_definitions(-DCODEIT_1_3_0)


link_directories(${CMAKE_SOURCE_DIR}/lib/linux/Release)

# Add source files
file(GLOB_RECURSE  STATE_SRC_FILES 
	${CMAKE_CURRENT_SOURCE_DIR}/src/cmd/*.cc
	${CMAKE_CURRENT_SOURCE_DIR}/src/cmd/*.cpp
)

include_directories(${CMAKE_SOURCE_DIR}/include/cmd)

add_library(${TrueRbotoCmd} STATIC ${STATE_SRC_FILES} )


target_include_directories(${TrueRbotoCmd}
	PRIVATE
	${CMAKE_SOURCE_DIR}/include
	${CMAKE_SOURCE_DIR}/include/cmd
)

target_include_directories(${TrueRbotoCmd}
	PUBLIC
	${CMAKE_INSTALL_PREFIX}/include
	${CMAKE_INSTALL_PREFIX}/include/cmd
)


# ===== LINK DEPENDENCIES ======
target_link_directories(${TrueRbotoCmd}
	PUBLIC 
	${CMAKE_INSTALL_PREFIX}/lib
)

target_link_libraries(${TrueRbotoCmd} ${CMAKE_SOURCE_DIR}/lib/linux/Release/libcpp_rpc.so)
target_link_libraries(${TrueRbotoCmd} pthread)


# ===== LINK DEPENDENCIES ======
#### Head file install ####
install(DIRECTORY 
	${CMAKE_CURRENT_SOURCE_DIR}/include/cmd
	DESTINATION ${CMAKE_INSTALL_PREFIX}/include
	FILES_MATCHING 
	PATTERN "*.h"
	PATTERN "*.hpp"
	PATTERN "*.inc"
)

#### Cmake package install #### 
install(TARGETS ${TrueRbotoCmd}
EXPORT "${TrueRbotoCmd}-targets"
RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# copy the *-targets.cmake file to the CMAKE_INSTALL_PREFIX directory
install(EXPORT "${TrueRbotoCmd}-targets"
    FILE "${TrueRbotoCmd}-targets.cmake"
    DESTINATION "lib/cmake/${TrueRbotoCmd}"
    )


install(DIRECTORY "${CMAKE_SOURCE_DIR}/lib/linux/Release/" 
	DESTINATION "bin"
	FILES_MATCHING 
	PATTERN "*"
)

# copy the *.-config file to the CMAKE_INSTALL_PREFIX directory. This will specify the dependencies.
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/${TrueRbotoCmd}-config.cmake.in" 
			 "${TrueRbotoCmd}-config.cmake" @ONLY)
install(FILES 
	"${CMAKE_CURRENT_BINARY_DIR}/${TrueRbotoCmd}-config.cmake"
    DESTINATION 
	"lib/cmake/${TrueRbotoCmd}"
)


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/Modules")


# set cmake compile features and options, this replaces the CMAKE_CXX_FLAGS
# for c++ version: cxx_std_11, cxx_std_14, cxx_std_17
target_compile_features(${TrueRbotoCmd} PRIVATE cxx_std_17)
set_target_properties(${TrueRbotoCmd} PROPERTIES CXX_EXTENSIONS OFF)
#target_compile_options(${PROJECT_NAME} PRIVATE -Werror)
set(CMAKE_VERBOSE_MAKEFILE ON)


# we supply a package wrapper ourselves: cmake/Modules/FindXXX.cmake
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/Modules")


# set cmake compile features and options, this replaces the CMAKE_CXX_FLAGS
# for c++ version: cxx_std_11, cxx_std_14, cxx_std_17
target_compile_features(${TrueRbotoCmd} PRIVATE cxx_std_17)
set_target_properties(${TrueRbotoCmd} PROPERTIES CXX_EXTENSIONS OFF)
#target_compile_options(${PROJECT_NAME} PRIVATE -Werror)
set(CMAKE_VERBOSE_MAKEFILE ON)