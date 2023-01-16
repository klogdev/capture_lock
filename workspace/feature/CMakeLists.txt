add_library(stb_image STATIC
            stb_image.h
            stb_image_write.h
)

set_target_properties(stb_image PROPERTIES LINKER_LANGUAGE CXX)

add_library(img STATIC
            image_sift.h
            image_sift.cpp
)

target_link_libraries(img PRIVATE stb_image)

add_library(sift STATIC
            sift.cpp
            sift.h
)

target_link_libraries(sift PRIVATE
                      img
)

add_executable(find_keypoints find_keypoints.cpp)

#target_include_directories(find_keypoints PRIVATE ../src)

target_link_libraries(find_keypoints PRIVATE
                      img
                      sift
)

add_executable(match_features match_features.cpp)

#target_include_directories(match_features PRIVATE ../src)

target_link_libraries(match_features PRIVATE
                      img
                      sift
)
