link_directories("/usr/local/lib/colmap/")

add_executable(residual residual_factory.cpp)
target_include_directories(
  residual PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  residual PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} ${absl_LIBRARIES})