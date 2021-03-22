cmake_minimum_required(VERSION 3.5)
project(rov_io)

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


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rov_msgs REQUIRED)


include_directories(include
  ${rclcpp_INCLUDE_DIRS}
  #${EIGEN3_INCLUDE_DIR} 
)
ament_python_install_package(scripts/)




#Install C++ targets
install(TARGETS
  #${PROJECT_NAME}_dvl_50
  DESTINATION lib/${PROJECT_NAME}
)

#Install Python scripts
#file(GLOB SCRIPT_FILES scripts/*)
install(PROGRAMS
  scripts/dvl-A50.py
  DESTINATION lib/${PROJECT_NAME}
)

#Install Launch file
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
