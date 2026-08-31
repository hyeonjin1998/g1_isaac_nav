cmake_minimum_required(VERSION 3.8)
project(g1_localization)

find_package(ament_cmake REQUIRED)

foreach(dir launch params maps config rviz)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
    install(DIRECTORY ${dir}/ DESTINATION share/${PROJECT_NAME}/${dir})
  endif()
endforeach()

ament_package()
