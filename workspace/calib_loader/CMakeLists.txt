link_directories("../")
link_directories("../feature")
link_directories("../file_reader")
link_directories("/usr/local/lib/colmap/")

include_directories(${WORKSPACE_DIR})

add_library(calib_loader STATIC
calib_base.h
calib_base.cpp
colmap_calib.h
kitti_calib.h
)
target_include_directories(
  calib_loader PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  calib_loader PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} kitti_odo)

