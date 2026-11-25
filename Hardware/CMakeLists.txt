add_subdirectory(Motor)
add_subdirectory(Imu)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    bsp/hfdcan_bsp.cpp bsp/uart_bsp.cpp
    Controller/base_rc_receiver.cpp Controller/FlySky/sbus.cpp
    Flash/w25q.cpp
    Led/ws2812.cpp Led/ws2812_codec.cpp
    Buzzer/base_buzzer.cpp Buzzer/pwm_buzzer.cpp)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    bsp Controller Controller/FlySky Flash Led Buzzer)
