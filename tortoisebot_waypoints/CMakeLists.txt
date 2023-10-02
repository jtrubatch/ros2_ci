cmake_minimum_required(VERSION 3.8)
project(tortoisebot_waypoints)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tortoisebot_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

# SERVER 
add_executable(action_server src/tortoisebot_as.cpp)
ament_target_dependencies(action_server
	"rclcpp"
	"rclcpp_action"
	"tortoisebot_action"
	"geometry_msgs"
	"nav_msgs"
	"std_msgs"
	"tf2"
)
install(TARGETS
	action_server
	DESTINATION lib/${PROJECT_NAME}
)

# CLIENT
include_directories(include)
add_library(action_client_lib src/tortoisebot_ac.cpp include/tortoisebot_waypoints/tortoisebot_ac.hpp)
ament_target_dependencies(action_client_lib
	"rclcpp"
	"rclcpp_action"
	"tortoisebot_action"
	"std_msgs"
	"geometry_msgs"
)

ament_export_targets(action_client_lib HAS_LIBRARY_TARGET)

install(
	DIRECTORY include/tortoisebot_waypoints
	DESTINATION include
)

add_executable(action_client src/main.cpp)
target_link_libraries(action_client action_client_lib)

install(
	TARGETS action_client_lib
	DESTINATION lib/action_client_lib
)

install(
	TARGETS action_client
	DESTINATION lib/${PROJECT_NAME}
)

install(
	TARGETS action_client_lib
	EXPORT action_client_lib
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)
# LAUNCH FILE
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
# TESTING
if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
  	find_package(ament_cmake_gtest REQUIRED)

	set(TESTFILES
		test/main.cpp
		test/tortoisebot_test.cpp
	)
	ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
	ament_target_dependencies(${PROJECT_NAME}_test
		"rclcpp"
		"rclcpp_action"
		"tortoisebot_action"
		"geometry_msgs"
		"nav_msgs"
		"std_msgs"
		"tf2"
	)
  	include_directories(include)
	target_link_libraries(${PROJECT_NAME}_test action_client_lib)
	
	install(TARGETS
		${PROJECT_NAME}_test
		DESTINATION lib/${PROJECT_NAME}
	)
endif()


ament_package()
