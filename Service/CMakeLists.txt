add_subdirectory(Btb)
add_subdirectory(Fusion_Service)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE vofa.cpp PowerLimit/PowerLimit.cpp)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
