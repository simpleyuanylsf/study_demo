cmake_minimum_required(VERSION 3.0.2)
project(nav_grid_map)

## 1. 寻找 ROS 依赖库
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  visualization_msgs
  geometry_msgs  # 新增
  pcl_ros
  pcl_conversions
)

## 2. 寻找系统 PCL 库
find_package(PCL REQUIRED)

## 3. 寻找 Eigen3 数学库 (新增)
find_package(Eigen3 REQUIRED)

## 4. 声明 catkin 包
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nav_grid_map
#  CATKIN_DEPENDS roscpp std_msgs sensor_msgs visualization_msgs geometry_msgs pcl_ros pcl_conversions
#  DEPENDS system_lib
)

## 5. 包含头文件目录 (注意加上 Eigen 的目录)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR} 
)

## 6. PCL 定义
add_definitions(${PCL_DEFINITIONS})

## 7. 编译你的节点
# 我们把你刚才的代码保存为 nav_map_node.cpp
add_executable(nav_map_node src/nav_map_node.cpp)
add_executable(grid_map_node src/grid_map_node.cpp)

## 8. 链接库文件
target_link_libraries(nav_map_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  # Eigen 是纯头文件库，通常不需要显式 link 库文件，但如果有报错可以加上
)
target_link_libraries(grid_map_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)