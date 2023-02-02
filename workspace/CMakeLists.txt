add_subdirectory(feature)

link_directories("/usr/local/lib/colmap/")

add_library(test_util STATIC
test_util.h
test_util.cpp
)
target_link_libraries(
  test_util PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} img sift)


add_executable(feature_sift feature_sift_test.cpp)
target_include_directories(
  feature_sift PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  feature_sift PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} img sift test_util)

add_executable(essential_matrix essential_matrix_test.cpp)
target_include_directories(
  essential_matrix PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  essential_matrix PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} img sift test_util)

add_executable(ransac ransac_test.cpp)
target_include_directories(
  ransac PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  ransac PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} img sift test_util)

add_executable(main_cli main.cpp)
target_include_directories(
  main_cli PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  main_cli PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES})

add_executable(library_test library_test.cpp)
target_include_directories(
  library_test PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}" 
  "${OpenCV_INCLUDE_DIRS}")
target_link_libraries(
  library_test PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} ${OpenCV_LIBRARIES}
  ${absl_LIBRARIES})

add_executable(opencv_test open_cv_test.cpp)
target_include_directories(
  opencv_test PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}" 
  "${OpenCV_INCLUDE_DIRS}")
target_link_libraries(
  opencv_test PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} ${OpenCV_LIBRARIES}
  ${absl_LIBRARIES} test_util)