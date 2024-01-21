link_directories("../feature")
link_directories("../calib_loader")
link_directories("../file_reader")
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

add_library(triangulate STATIC
triangulate.h
triangulate.cpp
)
target_include_directories(
  triangulate PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  triangulate PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES})

add_library(save_pose STATIC
pose_save.h
pose_save.cpp
)
target_include_directories(
  save_pose PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  save_pose PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} img sift)

add_library(sift_colmap STATIC
sift_colmap.h
sift_colmap.cpp
)
target_include_directories(
  sift_colmap PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  sift_colmap PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} img sift)

add_library(gyro STATIC
gyro_calc.h
)
target_include_directories(
  gyro PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  gyro PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} img sift)

add_library(kitti_pose STATIC
kitti_pose.h
kitti_pose.cpp
)
target_include_directories(
  kitti_pose PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  kitti_pose PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES})

add_library(gt_map STATIC
gt_map.h
gt_map.cpp
)
target_include_directories(
  gt_map PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  gt_map PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} kitti_pose)