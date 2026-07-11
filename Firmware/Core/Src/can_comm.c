#include "can_comm.h"
#include "app.h"
#include "main.h"      // hcan 타입, HAL 함수

extern CAN_HandleTypeDef hcan;   // main.c에 정의된 CAN 핸들

/* ============================================================
 *  ① 초기화 : 필터 설정 + CAN 시작
 * ============================================================ */
void CAN_Comm_Init(void){
    CAN_FilterTypeDef filter = {0};

    /* 필터: 일단 "모든 메시지 수신" (마스크 0 = 무조건 통과)
       나중에 특정 ID만 받으려면 IdHigh/MaskIdHigh를 조정 */
    filter.FilterBank           = 0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0x0000;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x0000;   // 0 = 모든 비트 무시
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14;

    /* CAN 하드웨어(트랜시버/버스)가 없어도 시스템 전체가 멈추지 않도록
       실패해도 Error_Handler()로 트랩하지 않는다. (실패 시 CAN만 비활성) */
    HAL_CAN_ConfigFilter(&hcan, &filter);
    HAL_CAN_Start(&hcan);
}

/* ============================================================
 *  ② 수신 : FIFO에 메시지 있으면 읽어서 목표값 갱신
 *      (주기 태스크에서 계속 호출)
 * ============================================================ */
void CAN_Comm_Poll(void){
    CAN_RxHeaderTypeDef rx;
    uint8_t data[8];

    /* FIFO에 쌓인 메시지를 전부 비울 때까지 처리 (최대 3개) */
    while(HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0){
        if(HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rx, data) != HAL_OK){
            return;
        }

        /* 주행 명령 ID면 파싱 */
        if(rx.StdId == CAN_ID_DRIVE_CMD && rx.DLC >= 4){
            int16_t  speed = (int16_t)( data[0] | (data[1] << 8) );  // 바이트 2개 -> int16
            uint16_t angle = (uint16_t)( data[2] | (data[3] << 8) );
            App_SetDrive(speed, angle);   // 공유 목표값 갱신 (임계구역으로 보호됨)
        }
    }
}

/* ============================================================
 *  ③ 송신 : 현재 상태를 Pi로 전송
 * ============================================================ */
void CAN_Comm_SendStatus(int16_t speed, uint16_t angle){
    CAN_TxHeaderTypeDef tx = {0};
    uint8_t data[8];
    uint32_t mailbox;

    tx.StdId = CAN_ID_STATUS;
    tx.IDE   = CAN_ID_STD;    // 표준 11비트 ID
    tx.RTR   = CAN_RTR_DATA;  // 데이터 프레임
    tx.DLC   = 4;             // 4바이트

    data[0] = (uint8_t)( speed & 0xFF);
    data[1] = (uint8_t)((speed >> 8) & 0xFF);
    data[2] = (uint8_t)( angle & 0xFF);
    data[3] = (uint8_t)((angle >> 8) & 0xFF);

    /* 빈 우편함(mailbox) 있으면 전송 */
    if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0){
        HAL_CAN_AddTxMessage(&hcan, &tx, data, &mailbox);
    }
}
