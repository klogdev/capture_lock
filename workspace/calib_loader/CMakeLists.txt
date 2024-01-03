link_directories("../")
link_directories("../feature")
link_directories("../file_reader")
link_directories("/usr/local/lib/colmap/")

include_directories(${WORKSPACE_DIR})

add_library(colmap_calib STATIC
colmap_calib.h
)
target_include_directories(
  colmap_calib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  colmap_calib PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES})

add_library(kitti_calib STATIC
kitti_calib.h
)
target_include_directories(
kitti_calib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
kitti_calib PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} kitti_odo)
