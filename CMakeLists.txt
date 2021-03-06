cmake_minimum_required(VERSION 3.5)
project(b5l_gateway_viewer)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/build)
     file(REMOVE_RECURSE ${CMAKE_CURRENT_SOURCE_DIR}/build )
endif()

if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../build/${PROJECT_NAME}/b5l_gateway_viewer)
     file(REMOVE_RECURSE ${CMAKE_CURRENT_SOURCE_DIR}/../build/${PROJECT_NAME}/b5l_gateway_viewer )
endif()

if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../install/${PROJECT_NAME}/lib/${PROJECT_NAME}/b5l_gateway_viewer)
     file(REMOVE_RECURSE ${CMAKE_CURRENT_SOURCE_DIR}/../install/${PROJECT_NAME}/lib/${PROJECT_NAME}/b5l_gateway_viewer )
endif()

if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../../install/${PROJECT_NAME}/lib/${PROJECT_NAME}/b5l_gateway_viewer)
     file(REMOVE_RECURSE ${CMAKE_CURRENT_SOURCE_DIR}/../../install/${PROJECT_NAME}/lib/${PROJECT_NAME}/b5l_gateway_viewer )
endif()

if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../../build/${PROJECT_NAME}/b5l_gateway_viewer)
     file(REMOVE_RECURSE ${CMAKE_CURRENT_SOURCE_DIR}/../../build/${PROJECT_NAME}/b5l_gateway_viewer )
endif()

# Set the path to the support repo: https://github.com/ZeroxCorbin/CSupportLibrary
if(WIN32)
     set(SUPPORT_LIB ../../Cpp/CSupportLibrary/)
else()
     set(SUPPORT_LIB ../CSupportLibrary/)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/support/)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/)
include_directories(${SUPPORT_LIB}/include/)

add_definitions(-DPI) #This is a Raspberry PI
add_definitions(-DSOURCE_DIR_PREFIX="${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(${PCL_DEFINITIONS})

find_package(PCL 1.3 REQUIRED)
find_package(Boost COMPONENTS)

# if(WIN32)
#      set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /STACK:10000000")
# endif()

set(CMAKE_THREAD_LIBS_INIT "-lpthread")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_WIN32_THREADS_INIT 0)
set(CMAKE_USE_PTHREADS_INIT 1)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(
     b5l_gateway_viewer
     src/b5l_gateway_viewer.cpp
     src/CloudProcessing.cpp
     ${SUPPORT_LIB}/src/clsTCPSocket.cpp
     ${SUPPORT_LIB}/src/libException.cpp
     ${SUPPORT_LIB}/src/clsTimer.cpp
     )

target_link_libraries(
     b5l_gateway_viewer
     Threads::Threads
     ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES} ${PCL_FILTERS_LIBRARIES}
     
     )

install(TARGETS
  b5l_gateway_viewer
  DESTINATION lib/${PROJECT_NAME})
