target_sources(${CMAKE_PROJECT_NAME} PRIVATE btb_service.cpp btb_transport.cpp)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/Tools/ThirdParty/BoostPFR/include)
