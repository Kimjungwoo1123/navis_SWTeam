#ifndef ACTUATOR_H       //  이미 포함됐으면 건너뛰기 (중복 방지 시작)

#define ACTUATOR_H       // "이 헤더 포함됨" 표시
#include <stdint.h>      // uint16_t 같은 타입 쓰려고

void Motor_Init(void);
void Servo_Init(void);
void Motor_SetSpeed(int percent);
void Servo_Control(uint16_t angle);   // 20ms 주기로 호출: 목표 각도 이동 + 버징 억제

#endif
