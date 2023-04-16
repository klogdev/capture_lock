add_subdirectory(feature) 
add_subdirectory(polymorphism_test)
add_subdirectory(test)

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
  ba PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} img sift)

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
  increment PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} img sift)

add_library(init STATIC
init_first_pair.h
init_first_pair.cpp
)
target_include_directories(
  init PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  init PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} img sift)

add_library(stream STATIC
file_stream.h
file_stream.cpp
)
target_include_directories(
  stream PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  stream PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} ${absl_LIBRARIES} img sift)

add_library(gyro STATIC
gyro_calc.h
)
target_include_directories(
  gyro PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  gyro PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} img sift)

add_library(kitti STATIC
kitti_helper.h
)
target_include_directories(
  kitti PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  kitti PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} img sift)

add_executable(mapper mapper_main.cpp)
target_include_directories(
  mapper PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  mapper PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} ${absl_LIBRARIES}
  img sift stream init increment ba ba_run cost)

add_executable(main_cli main.cpp)
target_include_directories(
  main_cli PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  main_cli PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES})
