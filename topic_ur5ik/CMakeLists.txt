cmake_minimum_required(VERSION 3.8)
project(topic_ur5ik)

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

add_executable(target_pub src/target_pub.cpp)
ament_target_dependencies(
  target_pub
  "rclcpp"
  "std_msgs"
)
target_include_directories(target_pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(target_pub PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


add_executable(calc_sub src/calc_sub.cpp)
ament_target_dependencies(
  calc_sub
  "rclcpp"
  "std_msgs"
)
target_include_directories(calc_sub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(calc_sub PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS
  calc_sub
  target_pub
  DESTINATION lib/${PROJECT_NAME})

#python module
ament_python_install_package(${PROJECT_NAME})
install (PROGRAMS
  scripts/target_pub_py.py
  scripts/calc_sub_py.py
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
