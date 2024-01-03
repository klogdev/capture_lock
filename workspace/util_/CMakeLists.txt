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

  add_library(gyro STATIC
  gyro_calc.h
  )
  target_include_directories(
    gyro PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
  target_link_libraries(
    gyro PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} img sift)