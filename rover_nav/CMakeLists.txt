cmake_minimum_required(VERSION 3.8)
project(rover_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/pointcloud_processor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pointcloud_processor  
)

install(PROGRAMS
  scripts/my_octomap_server.py  #
  DESTINATION lib/${PROJECT_NAME}
  RENAME my_octomap_server  
)

# Install shared files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()