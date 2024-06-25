cmake_minimum_required(VERSION 3.5)
project(navigation_potential_function)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

set(PROJECT_INCLUDE_DIR "include/${PROJECT_NAME}")
include_directories(${PROJECT_INCLUDE_DIR}
                    ${BOOST_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS})

# Generate navigation potential function static library
add_library(navigation_potential_function_lib src/navigation_potential_function.cpp)

add_executable(simulation src/ros_simulation.cpp)
ament_target_dependencies(simulation rclcpp geometry_msgs turtlesim)
target_link_libraries(simulation navigation_potential_function_lib ${catkin_LIBRARIES})

install(TARGETS
    simulation
    navigation_potential_function_lib
    DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY
    config
    launch
    DESTINATION share/${PROJECT_NAME}
  )

ament_package()