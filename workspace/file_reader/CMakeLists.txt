link_directories("../feature")
link_directories("../calib_loader")
link_directories("/usr/local/lib/colmap/")
link_directories("../")
include_directories(${WORKSPACE_DIR})

add_library(stream STATIC
file_stream.h
file_stream.cpp
)
target_include_directories(
  stream PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  stream PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} ${absl_LIBRARIES} img sift)

add_library(gyro_load STATIC
gyro_raw.h
gyro_raw.cpp
)
target_include_directories(
gyro_load PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
gyro_load PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} img sift)

add_library(kitti_odo STATIC
kitti_odo_helper.h
kitti_odo_helper.cpp
)
target_include_directories(
  kitti_odo PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  kitti_odo PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} img sift)