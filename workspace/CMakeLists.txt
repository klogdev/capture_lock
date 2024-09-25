link_directories("/usr/local/lib/colmap/")


add_library(cost STATIC
cost_fxn.h
)
target_include_directories(
  cost PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  cost PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} img sift)

add_library(ba STATIC
bundle_adjuster.h
bundle_adjuster.cpp
)
target_include_directories(
  ba PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  ba PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} img sift data_type)

add_library(ba_run STATIC
global_bundle.h
global_bundle.cpp
)
target_include_directories(
  ba_run PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  ba_run PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} img sift ba)

add_library(increment STATIC
incremental_construct.h
incremental_construct.cpp
)
target_include_directories(
  increment PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  increment PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} img sift 
  reprojection triangulate sift_colmap pnp_lone)

add_library(init STATIC
init_first_pair.h
init_first_pair.cpp
)
target_include_directories(
  init PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  init PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} img sift relative_pose 
  reprojection triangulate sift_colmap kitti_pose pnp_lone)


add_executable(mapper mapper_main.cpp)
target_include_directories(
  mapper PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  mapper PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} ${absl_LIBRARIES}
  img sift stream init increment ba ba_run cost 
  save_pose file_option calib_base data_type kitti_pose gt_map colmap_intr)


set(WORKSPACE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(feature) 
add_subdirectory(test)
add_subdirectory(calib_loader)
add_subdirectory(estimate)
add_subdirectory(util_)
add_subdirectory(file_reader)
add_subdirectory(pnp)