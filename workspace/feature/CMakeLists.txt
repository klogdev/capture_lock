add_library(img STATIC
            image_sift.h
            image_sift.cpp
)

target_link_libraries(img PRIVATE PUBLIC ${OpenCV_LIBRARIES})

add_library(sift STATIC
            sift.cpp
            sift.h
)

target_link_libraries(sift PRIVATE
                      img
)

add_executable(find_keypoints find_keypoints.cpp)


target_link_libraries(find_keypoints PRIVATE
                      img
                      sift
)

add_executable(match_features match_features.cpp)


target_link_libraries(match_features PRIVATE
                      img
                      sift
)
