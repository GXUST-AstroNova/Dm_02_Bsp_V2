#include "Application.h"
#include "fdcan.h"
#include "hfdcan_bsp.h"
#include "vofa.h"
#include "dm_models.hpp"
#include "m3508.hpp"
#include "pid.hpp"
namespace Application {
    vofa debug(&huart1);
    Dm4340 test(&hfdcan2,0x10,0x20);
    M3508 test2(&hfdcan1,1);
    PositionalPID pid(2.70633115413388f, 23.543395974493f, 0.0317760653810761f, 6000.0f, 0.001f, 16384.0f);
    uint32_t loop_count = 0;
}

void Application_Init() {
    //can总线初始化
    can_bus::start(&hfdcan1);
    can_bus::start(&hfdcan2);
    can_bus::start(&hfdcan3);
    Application::test.init(DmCanMode::BusDefault);
    Application::test.enable();
    if (!Application::test2.init()) Error_Handler();
    Application::debug.add(&Application::pid.measurement_);
    Application::debug.add(&Application::pid.target_);
    Application::debug.add(&Application::pid.output_);
}

void Application_Loop() {
    if(Application::loop_count<5000){
        
    }else if(Application::loop_count==5000){
        Application::pid.Calculate(5000.0f, Application::test2.feedback().speed_rpm);
        Application::loop_count+=10;
    }
}

void Application_Timer_Callback(TIM_HandleTypeDef *htim) {
    (void)htim;
    if(htim->Instance == TIM7) {
        //1khz
        const auto feedback = Application::test2.feedback();
        if (feedback.valid) {
            Application::test2.setCurrentRaw(Application::pid.Calculate( feedback.speed_rpm));
        }
        Application::debug.send();
        DmMotor::sendAll();
        DjiMotorGroup::sendAll();
        if(Application::loop_count < 5000) {
            Application::loop_count++;
        }
    }
}
