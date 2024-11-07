cmake_minimum_required(VERSION 3.8)
project(leo_gz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)

find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)

add_library(leo_gz_differential_plugin SHARED
  src/differential_system.cpp
)

target_link_libraries(leo_gz_differential_plugin
  gz-sim::core
  gz-plugin::core
)

install(
  TARGETS
    leo_gz_differential_plugin
  DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_system_plugin_path.sh")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_system_plugin_path.dsv")

ament_package()
