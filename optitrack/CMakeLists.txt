cmake_minimum_required(VERSION 3.5)
project(optitrack)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system program_options filesystem)
find_package(Eigen3 3.3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(optitrack_msgs REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  nav_msgs
  tf2
  tf2_ros
  Boost
  Eigen3
  geometry_msgs
  optitrack_msgs
)

rosidl_generate_interfaces(${PROJECT_NAME}_msgs
  "msg/Marker.msg"
  "msg/RigidBody.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${std_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${PROJECT_NAME}_msgs
  ${optitrack_msgs}
)

set(library_name ${PROJECT_NAME})
add_library(${library_name} SHARED src/Mocap.cpp )
ament_target_dependencies(${library_name} ${dependencies})


add_executable(publisher_main src/optitrack_node.cpp)
target_link_libraries(publisher_main ${library_name})

ament_target_dependencies(publisher_main ${dependencies})



install(TARGETS publisher_main ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()



# cmake_minimum_required(VERSION 3.5)
# project(optitrack)



# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake_auto REQUIRED)
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(Boost REQUIRED COMPONENTS thread system program_options filesystem)
# find_package(Eigen3 REQUIRED)

# ## Specify include directories
# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
#   ${Boost_INCLUDE_DIRS}
#   ${EIGEN3_INCLUDE_DIR}
#   ${std_msgs_INCLUDE_DIRS}
# )


# ## Create an executable
# add_executable(optitrack_node_executable
#   ./src/Mocap.cpp
#   ./src/optitrack_node.cpp
# )

# ## Link the libraries
# ament_target_dependencies(optitrack_node_executable
#   rclcpp
#   std_msgs
#   geometry_msgs
#   Boost
#   Eigen3
# )

# ## Add message files
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/Marker.msg"
#   "msg/RigidBody.msg"
#   DEPENDENCIES std_msgs geometry_msgs
# )

# target_link_libraries(optitrack_node_executable
#   ${Boost_LIBRARIES}
#   ${catkin_LIBRARIES}
# )

# ## Install the executable
# install(TARGETS optitrack_node_executable
#   DESTINATION lib/${PROJECT_NAME}
# )

# ## Install the launch file
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

# ament_export_include_directories(include)
# ament_export_dependencies(ament_cmake)
# ament_export_dependencies(rclcpp)
# ament_export_dependencies(std_msgs)
# ament_export_dependencies(geometry_msgs)
# ament_export_dependencies(Boost)
# ament_export_dependencies(Eigen3)
# ament_export_libraries(optitrack_node_executable)


# ament_package()


