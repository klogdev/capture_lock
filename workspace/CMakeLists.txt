add_subdirectory(feature)

link_directories("/usr/local/lib/colmap/")

# add_library(read_data STATIC read_data.cpp)
# target_include_directories(
#     read_data PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
# target_link_libraries(read_data Eigen3::Eigen glog ${COLMAP_LIBRARIES})

add_executable(main_cli main.cpp)
target_include_directories(
  main_cli PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  main_cli PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} read_data)
