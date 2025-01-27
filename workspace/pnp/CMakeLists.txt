link_directories("/usr/local/lib/colmap/")
link_directories("../file_reader")
link_directories("../estimate")
link_directories("../")
include_directories(${WORKSPACE_DIR})

add_library(pnp_template STATIC
pnp_test_template.h
pnp_test_template.cpp
)
target_include_directories(
  pnp_template PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  pnp_template PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} lhm lsq dls epnp upnp sqpnp posit)

add_library(pnp_help STATIC
pnp_helper.h
pnp_helper.cpp
)
target_include_directories(
  pnp_help PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  pnp_help PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} lhm math_help)
  

add_library(pnp_data STATIC
pnp_test_data.h
pnp_test_data.cpp
)
target_include_directories(
  pnp_data PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  pnp_data PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} lhm math_help 
  tum colmap_pair pnp_help orb_load)

add_library(pnp_runner STATIC
pnp_runner.h
pnp_runner.cpp
)
target_include_directories(
  pnp_runner PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
  pnp_runner PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} pnp_data pnp_template lhm file_save)

add_executable(pnp pnp_test.cpp)
target_include_directories(
pnp PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
pnp PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} ${absl_LIBRARIES}
img test_util lhm pnp_runner pnp_data pnp_template)

add_executable(tum_cloud tum_cloud_test.cpp)
target_include_directories(
tum_cloud PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
tum_cloud PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} ${absl_LIBRARIES}
tum file_save)

add_executable(pnp_iter pnp_iteration.cpp)
target_include_directories(
pnp_iter PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_INCLUDE_DIRS}")
target_link_libraries(
pnp_iter PUBLIC Eigen3::Eigen ${COLMAP_LIBRARIES} ${absl_LIBRARIES}
img test_util lhm pnp_runner pnp_data pnp_template)
