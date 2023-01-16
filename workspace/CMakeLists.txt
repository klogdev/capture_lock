add_subdirectory(feature)

get_filename_component(PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
set(COLMAP_SRC_DIR "${PARENT_DIR}/colmap/src")
message("COLMAP_SRC_DIR is ${COLMAP_SRC_DIR}")

add_library(workspace_lib STATIC read_data.cpp)
target_include_directories(
    workspace_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${COLMAP_SRC_DIR}")
target_link_libraries(workspace_lib Eigen3::Eigen glog)

# TODO: add main.cpp into an executable.

#add_executable(main main.cpp)
#target_include_directories(main PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
#target_link_libraries(main PRIVATE
#                    read_data
#                    image
#                    camera
#                    point2d
#                    point3d
#                    Eigen3::Eigen)
#target_include_directories(colmap PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
