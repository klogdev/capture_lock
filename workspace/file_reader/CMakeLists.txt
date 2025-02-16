link_directories("../feature")
link_directories("../calib_loader")
link_directories("../util_")
link_directories("/usr/local/lib/colmap/")
link_directories("../")
include_directories(${WORKSPACE_DIR})

add_library(stream STATIC
file_stream.h
file_stream.cpp
)
target_include_directories(
  stream PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  stream PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} ${absl_LIBRARIES} img sift)

add_library(colmap_pair STATIC
colmap_pairs.h
colmap_pairs.cpp
)
target_include_directories(
  colmap_pair PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" 
  "${OpenCV_INCLUDE_DIRS}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  colmap_pair PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} ${absl_LIBRARIES} 
  ${OpenCV_LIBRARIES} file_save) 

add_library(orb_load STATIC
orb_loaded.h
orb_loaded.cpp
)
target_include_directories(
  orb_load PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" 
  "${OpenCV_INCLUDE_DIRS}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  orb_load PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} ${absl_LIBRARIES} 
  ${OpenCV_LIBRARIES} file_save tum) 

add_library(tum STATIC
tum_rgbd.h
tum_rgbd.cpp
)
target_include_directories(
  tum PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" 
  "${OpenCV_INCLUDE_DIRS}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  tum PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} ${absl_LIBRARIES} ${OpenCV_LIBRARIES} 
  sift_colmap data_type ba_run sift img cost tum_test relative_pose) 

add_library(file_option STATIC
file_options.h
)
target_include_directories(
file_option PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
file_option PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} ${absl_LIBRARIES} calib_base)

add_library(data_type STATIC
data_types.h
)
target_include_directories(
data_type PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
data_type PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} ${absl_LIBRARIES})

# add_library(gyro_load STATIC
# gyro_raw.h
# gyro_raw.cpp
# )
# target_include_directories(
# gyro_load PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
# target_link_libraries(
# gyro_load PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} img sift)

add_library(kitti_odo STATIC
kitti_odo_helper.h
kitti_odo_helper.cpp
)
target_include_directories(
  kitti_odo PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  kitti_odo PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES})

add_library(colmap_intr STATIC
colmap_pose.h
colmap_pose.cpp
)
target_include_directories(
  colmap_intr PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  colmap_intr PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES})