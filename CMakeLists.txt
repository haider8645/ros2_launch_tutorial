cmake_minimum_required(VERSION 3.5)
project(ros2_launch_tutorial)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

#Make sure you install all the folders used in the launch file
install(DIRECTORY 
         launch
         urdf
         rviz
         config
        DESTINATION 
         share/${PROJECT_NAME})

ament_package()
