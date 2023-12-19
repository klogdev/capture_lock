link_directories("../feature")
link_directories("../calib_loader")
link_directories("/usr/local/lib/colmap/")
link_directories("../")
include_directories(${WORKSPACE_DIR})

add_library(relative_pose STATIC
relative_pose.h
relative_pose.cpp
)
target_include_directories(
  relative_pose PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  relative_pose PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES})