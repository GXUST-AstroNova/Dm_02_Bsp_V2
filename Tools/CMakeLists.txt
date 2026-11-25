add_subdirectory(ThirdParty/Fusion/Fusion)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Controller/pid.cpp Controller/smc.cpp Controller/trajectory_smc.cpp)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR} Controller)
