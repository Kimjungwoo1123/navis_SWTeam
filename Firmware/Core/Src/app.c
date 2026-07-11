#include "FreeRTOS.h"
#include "task.h"
#include "app.h"
#include "actuator.h"
#include "can_comm.h"

#define TEST

static volatile int16_t  g_speed = 0;    // -100 ~ +100
static volatile uint16_t g_angle = 90;   // 30 ~ 150

/* 외부(예: CAN 수신)에서 목표값을 갱신할 때 호출 */
void App_SetDrive(int16_t speed, uint16_t angle){
    g_speed = speed;
    g_angle = angle;
}

/* 10ms 주기 : CAN 수신 확인 + 모터 제어 */
static void Task_10ms(void *pv){
    TickType_t t = xTaskGetTickCount();
    for(;;){
        CAN_Comm_Poll();            // 수신 메시지 있으면 App_SetDrive로 목표값 갱신
        Motor_SetSpeed(g_speed);
        vTaskDelayUntil(&t, pdMS_TO_TICKS(10));
    }
}

/* 20ms 주기 : 조향(서보) 제어
   목표 각도만 넘기면 이동/버징 억제는 actuator(Servo_Control)가 처리 */
static void Task_20ms(void *pv){
    TickType_t t = xTaskGetTickCount();
    for(;;){
        Servo_Control(g_angle);
        vTaskDelayUntil(&t, pdMS_TO_TICKS(20));
    }
}

/* 100ms 주기 : 현재 상태를 Pi로 송신 */
static void Task_100ms(void *pv){
    TickType_t t = xTaskGetTickCount();
    for(;;){
        CAN_Comm_SendStatus(g_speed, g_angle);
        vTaskDelayUntil(&t, pdMS_TO_TICKS(100));
    }
}

static void TestTask(void *pv){
//    for(;;){
        g_angle = 30;   vTaskDelay(pdMS_TO_TICKS(1000));
        g_angle = 80;   vTaskDelay(pdMS_TO_TICKS(1000));
        g_angle = 130;  vTaskDelay(pdMS_TO_TICKS(1000));
        g_angle = 150;  vTaskDelay(pdMS_TO_TICKS(1000));
//    }
}

void App_Init(void){
    Motor_Init();
    Servo_Init();
    CAN_Comm_Init();     // 필터 설정 + CAN 시작
}

void App_CreateTasks(void){
    /* Rate Monotonic: 짧은 주기 = 높은 우선순위 (10ms=10 > 20ms=9 > 100ms=8) */
    xTaskCreate(Task_10ms,  "10ms",  128, NULL, tskIDLE_PRIORITY + 10, NULL);
    xTaskCreate(Task_20ms,  "20ms",  128, NULL, tskIDLE_PRIORITY + 9,  NULL);
    xTaskCreate(Task_100ms, "100ms", 128, NULL, tskIDLE_PRIORITY + 8,  NULL);
#ifdef TEST
    xTaskCreate(TestTask, "1000ms", 128, NULL, 11, NULL);
#endif
}

