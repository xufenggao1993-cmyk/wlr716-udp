cmake_minimum_required(VERSION 3.5)
project(wlr716_udp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set (MIN_BOOST_VERSION 1.71.0 CACHE INTERNAL "Boost min version requirement" FORCE)

########################################
# Find Boost
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED)

if (NOT Boost_FOUND)
  BUILD_ERROR ("Boost not found. Please install boost version ${MIN_BOOST_VERSION} or higher.")
endif()

  # set boost pkgconfig cflags
  set (Boost_PKGCONFIG_CFLAGS ${Boost_INCLUDE_DIRS})
  if (NOT "${Boost_PKGCONFIG_CFLAGS}" STREQUAL "")
    set (Boost_PKGCONFIG_CFLAGS "-I${Boost_PKGCONFIG_CFLAGS}")
  endif (NOT "${Boost_PKGCONFIG_CFLAGS}" STREQUAL "")
  string (REPLACE ";" " -I" Boost_PKGCONFIG_CFLAGS "${Boost_PKGCONFIG_CFLAGS}")

  # set boost pkgconfig libs
  set (Boost_PKGCONFIG_LIBS ${Boost_LIBRARY_DIRS})
  if (NOT "${Boost_PKGCONFIG_LIBS}" STREQUAL "")
    set (Boost_PKGCONFIG_LIBS "-L${Boost_PKGCONFIG_LIBS}")
  endif(NOT "${Boost_PKGCONFIG_LIBS}" STREQUAL "")
  string (REPLACE ";" " -L" Boost_PKGCONFIG_LIBS "${Boost_PKGCONFIG_LIBS}")

  foreach (b ${Boost_LIBRARIES})
    get_filename_component(bname ${b} NAME_WE)
    # Prefix -l if not already prefixed
    string(FIND ${bname} "-l" lprefix)
    if (NOT ${lprefix} EQUAL 0 )
      set (bname "-l${bname}")
    endif()
    # Remove the prefix lib (not always present, like in pthread)
    string (REPLACE "lib" "" bname "${bname}")
    # Some boost versions add the Boost:: component to cmake, need to change that to boost_ prefix
    string (REPLACE "Boost::" "boost_" bname "${bname}")
    set (Boost_PKGCONFIG_LIBS "${Boost_PKGCONFIG_LIBS} ${bname}")
  endforeach(b)
########################################

include_directories(
  include
)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/protocol.cpp
  src/udp.cpp
  src/utils.cpp
)

target_link_libraries(${PROJECT_NAME}
  boost_thread
)

ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
endif()

ament_package()
