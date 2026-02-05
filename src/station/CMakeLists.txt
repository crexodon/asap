cmake_minimum_required(VERSION 3.8)
project(station)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(interfaces REQUIRED) # Die eigenen Messages laden

# Da wir hier keine Interfaces mehr generieren, gibt es keinen Namenskonflikt!
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  station/job_handler_node.py
  station/station_node.py
  station/job_spawner_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()