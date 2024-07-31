cmake_minimum_required(VERSION 3.8)
project(calc_ur5ik_topic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

add_executable(set_target src/set_target.cpp)
ament_target_dependencies(
  set_target
  "rclcpp"
  "std_msgs"
)
target_include_directories(set_target PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(set_target PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS set_target
  DESTINATION lib/${PROJECT_NAME})

add_executable(ur5ik src/ur5ik.cpp)
ament_target_dependencies(
  ur5ik
  "rclcpp"
  "std_msgs"
)
target_include_directories(ur5ik PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ur5ik PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS ur5ik set_target
  DESTINATION lib/${PROJECT_NAME})

#python module
ament_python_install_package(${PROJECT_NAME})
install (PROGRAMS
  scripts/set_target_py.py
  scripts/ur5ik_py.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
