#ifndef APP_H
#define APP_H

#include <stdint.h>

void App_Init(void);          // 액추에이터 초기화 (PWM start)
void App_CreateTasks(void);   // FreeRTOS 태스크 생성
void App_SetDrive(int16_t speed, uint16_t angle);  // 외부(CAN)에서 목표값 갱신

#endif
