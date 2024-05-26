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

set(PROJECT_INCLUDE_DIR "include/${PROJECT_NAME}")
include_directories(${PROJECT_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS})

find_package(ament_cmake REQUIRED)

add_executable(main src/main.cpp src/navigation_potential_function.cpp)
target_link_libraries(main mpfr ${catkin_LIBRARIES})

#target_include_directories(main PUBLIC include/${PROJECT_NAME})

ament_package()