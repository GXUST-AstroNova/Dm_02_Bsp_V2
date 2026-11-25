target_sources(${CMAKE_PROJECT_NAME} PRIVATE imu.cpp BMI088/bmi088.cpp)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} BMI088)
