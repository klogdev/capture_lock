add_subdirectory(feature)

link_directories("/usr/local/lib/colmap/")

add_executable(feature_sift feature_sift_test.cpp)
target_include_directories(
  feature_sift PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  feature_sift PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES})

add_executable(essential_matrix essential_matrix_test.cpp)
target_include_directories(
  essential_matrix PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  essential_matrix PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES})

add_executable(ransac ransac_test.cpp)
target_include_directories(
  ransac PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  ransac PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES})

add_executable(main_cli main.cpp)
target_include_directories(
  main_cli PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  main_cli PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES})
