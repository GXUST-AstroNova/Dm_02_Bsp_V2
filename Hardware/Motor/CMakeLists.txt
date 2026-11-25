target_sources(${CMAKE_PROJECT_NAME} PRIVATE motor.cpp DJI/m3508.cpp DM/dm_motor.cpp)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} DJI DM)
