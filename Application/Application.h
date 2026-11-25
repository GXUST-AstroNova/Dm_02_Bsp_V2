#pragma once

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void Application_Init(void);
void Application_Loop(void);
void Application_Timer_Callback(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}


#endif
