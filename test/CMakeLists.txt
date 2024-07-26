# tests for VP
ament_add_gtest(test_vp
  test_vector_pursuit.cpp
  path_utils/path_utils.cpp
)
ament_target_dependencies(test_vp
  ${dependencies}
  nav2_controller
)

target_link_libraries(test_vp
  ${library_name}
)

# Path utils test
ament_add_gtest(test_path_utils path_utils/test_path_utils.cpp path_utils/path_utils.cpp)
ament_target_dependencies(test_path_utils nav_msgs geometry_msgs tf2_geometry_msgs)
