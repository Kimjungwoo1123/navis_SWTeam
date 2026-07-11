#include "main.h"

/* --- 튜닝 상수 (매직넘버 대신 의미 부여) --- */
#define MOTOR_MAX_PERCENT   100          // 속도 입력 범위 ±100
#define MOTOR_MAX_DUTY      3600         // TIM1 Period(3599)+1 -> 100%
#define DUTY_PER_PERCENT    (MOTOR_MAX_DUTY / MOTOR_MAX_PERCENT)  // 1% = 36

#define SERVO_MIN_ANGLE     30           // 조향 최소 각도
#define SERVO_MAX_ANGLE     150          // 조향 최대 각도
#define SERVO_MIN_PULSE_US  1000         // 최소 펄스폭(us)
#define SERVO_PULSE_SPAN_US 1000         // 1000~2000us 범위 폭

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

void Motor_Init(void){
	//Dc
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

void Servo_Init(void){
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

//-100 ~ + 100
void Motor_SetSpeed(int percent){
	if(percent >  MOTOR_MAX_PERCENT) percent =  MOTOR_MAX_PERCENT;
	if(percent < -MOTOR_MAX_PERCENT) percent = -MOTOR_MAX_PERCENT;
	uint16_t duty = (percent < 0 ? -percent : percent) * DUTY_PER_PERCENT;
	if(percent == 0){
		//자연정지
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,0);
	}
	else if(percent >0){
		//정회전
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,duty);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,duty);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,0);
	}
	else if(percent<0){
		//역회전
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,duty);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,duty);
	}
}


/* --- 서보 버징 억제용 상태 (Servo_Control 전용) --- */
#define SERVO_DRIVE_MS        700                                   // 각도 변경 후 펄스 유지 시간(ms)
#define SERVO_CTRL_PERIOD_MS  20                                    // Servo_Control 호출 주기(ms)
#define SERVO_DRIVE_TICKS     (SERVO_DRIVE_MS / SERVO_CTRL_PERIOD_MS)

static uint16_t servo_last_angle  = 0xFFFF;   // 직전 목표 각도(초기값은 불가능한 값)
static int      servo_drive_ticks = 0;        // 남은 구동 틱 수

/* 내부: 각도(30~150) -> 펄스폭(us) 변환 후 CCR 설정 */
static void Servo_WritePulse(uint16_t angle){
	if(angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;
	if(angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
	uint16_t ccr = SERVO_MIN_PULSE_US + (angle * SERVO_PULSE_SPAN_US / 180);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr);
}

/* 내부: 펄스 폭을 0으로 만들어 신호를 끊는다.
   아날로그 서보는 펄스가 없으면 힘이 풀려서 유지 토크로 인한 버징이 사라짐 */
static void Servo_Release(void){
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
}

/* 공개: SERVO_CTRL_PERIOD_MS(=20ms) 주기로 호출할 것.
   목표 각도가 바뀌면 일정 시간(SERVO_DRIVE_MS) 펄스를 보내 이동시키고,
   이동이 끝나면 신호를 끊어 유지 토크 버징을 없앤다. */
void Servo_Control(uint16_t angle){
	if(angle != servo_last_angle){          // 목표가 바뀜 -> 다시 구동 시작
		servo_last_angle  = angle;
		servo_drive_ticks = SERVO_DRIVE_TICKS;
	}
	if(servo_drive_ticks > 0){
		Servo_WritePulse(angle);            // 이동/유지용 펄스 전송
		servo_drive_ticks--;
	}else{
		Servo_Release();                    // 신호 차단 -> 힘 풀림 -> 버징 제거
	}
}
