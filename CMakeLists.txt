cmake_minimum_required(VERSION 3.5)
project(pi_hw_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


#generate messages
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "messages/msg/Test.msg"
  #DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)


# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
#find_package(gpiod REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tinyxml_vendor REQUIRED)
find_package(TinyXML REQUIRED)


add_executable(hw_interface src/interface.cpp)
add_executable(test_ex src/main.cpp)

target_link_libraries(hw_interface Threads::Threads)
target_link_libraries(hw_interface -lgpiodcxx)
target_link_libraries(hw_interface -lstdc++fs)
target_link_libraries(hw_interface ${TinyXML_LIBRARIES})
ament_target_dependencies(hw_interface rclcpp)

target_link_libraries(test_ex Threads::Threads)
target_link_libraries(test_ex -lgpiodcxx)

rosidl_target_interfaces(hw_interface
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS hw_interface DESTINATION lib/${PROJECT_NAME})
install(TARGETS test_ex DESTINATION lib/${PROJECT_NAME})


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
