cmake_minimum_required(VERSION 2.8.3)
project(jog_teleop)

add_compile_options(-std=c++11)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  jog_msgs
  tf
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
  roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(spacenav_teleop nodes/spacenav_teleop.cpp)
add_dependencies(spacenav_teleop ${catkin_EXPORTED_TARGETS})
target_link_libraries(spacenav_teleop ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(xbox_teleop nodes/xbox_teleop.cpp)
add_dependencies(xbox_teleop ${catkin_EXPORTED_TARGETS})
target_link_libraries(xbox_teleop ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(dragonrise_teleop nodes/dragonrise_teleop.cpp)
add_dependencies(dragonrise_teleop ${catkin_EXPORTED_TARGETS})
target_link_libraries(dragonrise_teleop ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

install(TARGETS spacenav_teleop xbox_teleop dragonrise_teleop
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS nodes/key_teleop.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
