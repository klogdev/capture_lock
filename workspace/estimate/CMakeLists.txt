link_directories("../feature")
link_directories("../calib_loader")
link_directories("../util_")
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

add_library(lhm STATIC
lhm.h
lhm.cpp
)
target_include_directories(
  lhm PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  lhm PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} adj_quat math_help lsq)

add_library(adj_quat STATIC
adj_quat.h
adj_quat.cpp
)
target_include_directories(
  adj_quat PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  adj_quat PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES})

add_library(lsq STATIC
least_sqr_pnp.h
least_sqr_pnp.cpp
)
target_include_directories(
  lsq PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  lsq PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES})

# add_library(epipole_estimate STATIC
# epipole_estimator.h
# epipole_estimator.cpp
# )
# target_include_directories(
#   epipole_estimate PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
# target_link_libraries(
#   epipole_estimate PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES})