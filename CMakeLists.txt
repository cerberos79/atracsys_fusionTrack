cmake_minimum_required(VERSION 2.8.3)
project(atracsystracking)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_COLOR_MAKEFILE ON)
Set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(THREADS_PREFER_PTHREAD_FLAG OFF)
if(CMAKE_COMPILER_IS_GNUCC)
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -W -o3")
endif(CMAKE_COMPILER_IS_GNUCC)

set(PACKAGE_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread filesystem log timer)
set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME ON)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs rospy message_generation)

set(Atracsys_include ${PACKAGE_ROOT}/AtracsysLibs/include)

IF( "${CMAKE_SIZEOF_VOID_P}" EQUAL "8" )
    SET( ARCH 64 )
ELSE( "${CMAKE_SIZEOF_VOID_P}" EQUAL "8" )
    SET( ARCH 32 )
ENDIF( "${CMAKE_SIZEOF_VOID_P}" EQUAL "8" )

find_library(AtracsysLib REQUIRED fusionTrack${ARCH} PATHS ${PACKAGE_ROOT}/AtracsysLibs/lib)
add_definitions( -DATR_FTK -DBOOST_LOG_DYN_LINK )
include_directories( include  ${Atracsys_include}  ${Boost_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR}  ${catkin_INCLUDE_DIRS})
add_service_files(FILES   AtracsysService.srv )
generate_messages(DEPENDENCIES std_msgs) 
catkin_package(INCLUDE_DIRS include  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs)

LINK_DIRECTORIES(${PACKAGE_ROOT}/AtracsysLibs/lib)
add_executable(atracsystracking ${PACKAGE_ROOT}/src/atracsys_node.cpp ${PACKAGE_ROOT}/src/atracsys.cpp ${PACKAGE_ROOT}/include/atracsys.hpp)
target_link_libraries(atracsystracking ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${AtracsysLib})
add_dependencies( atracsystracking ${PROJECT_NAME}_generate_messages_cpp )

install(TARGETS ${PROJECT_NAME} atracsystracking
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
