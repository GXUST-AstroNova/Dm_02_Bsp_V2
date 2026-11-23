# Tools：仅头文件的通用组件（irq_guard.hpp 等）与第三方库。
# 让其它模块可以直接 #include "irq_guard.hpp"。
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR})
