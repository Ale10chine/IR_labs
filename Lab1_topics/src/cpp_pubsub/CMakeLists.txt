cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

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

# Adding executable for PUBLISHER NODE and naming it 'talker', in this way we can run the node using 'ros2 run'
add_executable(talker src/publisher_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
# Adding executable for SUBSCRIBER NODE
add_executable(listener src/subscriber_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

# Adding the tracking for the executable
install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})
 

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()




# Clean up with the essentials parts of the CmMakeLists.txt

#cmake_minimum_required(VERSION 3.5)
#project(cpp_pubsub)

## Default to C++14
#if(NOT CMAKE_CXX_STANDARD)
  #set(CMAKE_CXX_STANDARD 14)
#endif()

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

#find_package(ament_cmake REQUIRED)
#find_package(rclcpp REQUIRED)
#find_package(std_msgs REQUIRED)

#add_executable(talker src/publisher_lambda_function.cpp)
#ament_target_dependencies(talker rclcpp std_msgs)

#install(TARGETS
  #talker
  #DESTINATION lib/${PROJECT_NAME})

#ament_package()
