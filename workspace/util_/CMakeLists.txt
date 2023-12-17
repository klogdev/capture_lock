link_directories("../feature")
link_directories("../calib_loader")
link_directories("/usr/local/lib/colmap/")
link_directories("../")
include_directories(${WORKSPACE_DIR})

add_library(reprojection STATIC
reprojection.h
reprojection.cpp
)
target_include_directories(
  reprojection PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  reprojection PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES})