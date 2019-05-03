cmake_minimum_required(VERSION 2.8.3)
project(magellan_proflex500_driver)

find_package(catkin REQUIRED COMPONENTS roscpp nmea_msgs)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  )

add_executable(magellan_proflex500_driver
  src/magellan_proflex500_driver.cpp
  )

target_link_libraries(magellan_proflex500_driver
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )
