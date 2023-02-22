add_subdirectory(feature)

link_directories("/usr/local/lib/colmap/")

add_library(test_util STATIC
test_util.h
test_util.cpp
)
target_include_directories(
  test_util PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  test_util PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} img sift)

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
  stream PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} img sift)

add_executable(mapper mapper_main.cpp)
target_include_directories(
  mapper PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  mapper PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} img sift test_util)

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