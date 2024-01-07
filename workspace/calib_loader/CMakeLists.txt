link_directories("../")
link_directories("../feature")
link_directories("../file_reader")
link_directories("/usr/local/lib/colmap/")

include_directories(${WORKSPACE_DIR})

# add_library(colmap_calib_loader STATIC
# colmap_calib.h
# colmap_calib.cpp
# )

add_library(calib_base STATIC
calib_base.h
calib_base.cpp
)
target_include_directories(
  calib_base PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  calib_base PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} kitti_odo data_type)

