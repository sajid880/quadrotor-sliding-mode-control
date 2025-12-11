cmake_minimum_required(VERSION 3.5)
project(${PROJECT_NAME})

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  PROGRAMS
    ${PROJECT_NAME}/${PROJECT_NAME}_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
