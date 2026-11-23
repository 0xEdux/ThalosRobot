/*
 * motor.h
 *
 *  Created on: 22 nov 2026
 *      Author: juanb
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "main.h"
#include <string.h>

#include "encoder.h"

#define MIN_RPM     80.0f
#define MAX_RPM     120.0f

#define MIN_PWM     20.0f
#define MAX_PWM     240.0f  

#define FF_TIME_MS  250

#define RAMP      100.0f
#define TIM_PERIOD    255.0f     /* = htim2.Init.Period */

const float RPM_TABLE[] = {80.4f, 92.7f, 101.7f, 114.4f};
const float PWM_TABLE[] = {127.0f, 160.0f, 200.0f, 255.0f};

typedef enum { 
    MOTOR_FORWARD = 0,
    MOTOR_BACKWARD,
    MOTOR_BRAKE
} Motor_Direction_t;

typedef struct {
    GPIO_TypeDef *in1_port; uint16_t in1_pin;
    GPIO_TypeDef *in2_port; uint16_t in2_pin;
    uint32_t pwm_ch;
    TIM_HandleTypeDef *timer;
    Encoder_Handler_t *encoder;
    float kp, ki;
} Motor_Config_t;

typedef struct {

    GPIO_TypeDef *IN1_Port;
    GPIO_TypeDef *IN2_Port;
    uint16_t IN1_Pin;
    uint16_t IN2_Pin;

    Motor_Direction_t direction;

    uint32_t pwm_ch;

    Encoder_Handler_t *encoder;
    TIM_HandleTypeDef *timer;

    float Kp, Ki;
    float integral;
    float speed, refSpeed;
    
    uint32_t startTime;

    float rpm, u;

} Motor_Handler_t;

void Motor_Init(Motor_Handler_t *m, const Motor_Config_t *cfg);
static float ffPWM (float rpm);
static float constrainf(float value, float low, float high);
void Motor_SetRef(Motor_Handler_t *motor, float rpm);
void Motor_SetDirection( Motor_Handler_t *motor, Motor_Direction_t direction);
void Motor_SetVelocity(Motor_Handler_t *motor, float rpm);
static void Motor_ApplyDir(Motor_Handler_t *motor);
void Motor_Step(Motor_Handler_t *motor, float dt, uint32_t now_ms);
void Motor_SetGains(Motor_Handler_t *motor, float Kp, float Ki);


#endif /* INC_MOTOR_H_ */
