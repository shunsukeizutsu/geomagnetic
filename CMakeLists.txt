cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(extractData)

include(CheckCXXCompilerFlag)
#CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
#	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
	message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Use a different C++ compiler.")
endif()

find_package(Eigen3)
IF(NOT EIGEN3_INCLUDE_DIR)
	set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_ROOT_DIR})
ENDIF()

include_directories( ./src ../izutu/utility/src ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} )

# set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# set the build type
set(CMAKE_BUILD_TYPE Release)
# confirmation messages
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
message(STATUS "CMAKE_C_FLAGS: ${CMAKE_C_FLAGS}")
message(STATUS "CMAKE_C_FLAGS_RELEASE: ${CMAKE_C_FLAGS_RELEASE}")
message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
message(STATUS "CMAKE_CXX_FLAGS_RELEASE: ${CMAKE_CXX_FLAGS_RELEASE}")

add_executable(extra ./src/extradata.cpp ../izutu/utility/src/utility.cpp ./src/GnsstoMag.cpp)
target_link_libraries(extra m ssm)

add_executable(sensor ./src/sensordata.cpp ../izutu/utility/src/utility.cpp )
target_link_libraries(sensor m ssm)

add_executable(localizer ./src/agndata.cpp ../izutu/utility/src/utility.cpp )
target_link_libraries(localizer m ssm)

add_executable(GtoM ./src/Magcal.cpp ./src/GnsstoMag.cpp)
target_link_libraries(GtoM m ssm)

add_executable(eigen ./src/PracEigen.cpp ./src/GnsstoMag.cpp)
target_link_libraries(eigen m ssm)