#ifndef CAN_COMM_H
#define CAN_COMM_H

#include <stdint.h>

/* 통신 규약 (라즈베리파이와 동일하게 맞출 것) */
#define CAN_ID_DRIVE_CMD   0x100   // Pi -> STM32 : 주행 명령
#define CAN_ID_STATUS      0x200   // STM32 -> Pi : 상태 보고

void CAN_Comm_Init(void);                          // 필터 설정 + CAN 시작
void CAN_Comm_Poll(void);                          // 수신 메시지 확인/처리 (주기 호출)
void CAN_Comm_SendStatus(int16_t speed, uint16_t angle);  // 상태 송신

#endif
