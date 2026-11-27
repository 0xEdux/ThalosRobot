/*
 * motor.c
 *
 *  Created on: 22 nov 2026
 *      Author: juanb
 */

#include "motor.h"


const float RPM_TABLE[] = {85.0f, 104.7f, 116.2f, 129.9f};
const float PWM_TABLE[] = {80.0f, 100.0f, 120.0f, 150.0f};

void Motor_Init(Motor_Handler_t *m, const Motor_Config_t *cfg) {
    if (m == NULL || cfg == NULL || cfg->timer == NULL || cfg->encoder == NULL)
        return;

    memset(m, 0, sizeof(*m));
    m->IN1_Port = cfg->in1_port;  
    m->IN1_Pin = cfg->in1_pin;
    m->IN2_Port = cfg->in2_port;  
    m->IN2_Pin = cfg->in2_pin;
    m->pwm_ch = cfg->pwm_ch;
    m->timer = cfg->timer;
    m->encoder = cfg->encoder;
    m->Kp = cfg->kp;  
    m->Ki = cfg->ki;

    __HAL_TIM_SET_COMPARE(m->timer, m->pwm_ch, 0);
    HAL_TIM_PWM_Start(m->timer, m->pwm_ch);
    Motor_SetDirection(m, MOTOR_BRAKE);
}

float ffPWM (float rpm) {
    // RPM table starts at 80 rpm
    if (rpm <= RPM_TABLE[0]) return PWM_TABLE[0];
    for (int k = 0; k < TABLE_SIZE - 1; k++) {
        if(rpm <= RPM_TABLE[k + 1]) {
            return PWM_TABLE[k] + (rpm - RPM_TABLE[k]) * (PWM_TABLE[k + 1] - PWM_TABLE[k]) / (RPM_TABLE[k + 1] - RPM_TABLE[k]);
        }
    }
    return PWM_TABLE[3];
}

float constrainf(float value, float low, float high) {
    if(value < low) {
        return low;
    } else if(value > high) {
        return high;
    } else {
        return value;
    }
}

void Motor_SetRef(Motor_Handler_t *motor, float rpm) {
    if(motor == NULL) return;
    motor->refSpeed = (rpm <= 0.0f) ? 0.0f : constrainf(rpm, MIN_RPM, MAX_RPM);
}

void Motor_SetDirection( Motor_Handler_t *motor, Motor_Direction_t direction) {
    if (motor == NULL) return;

    motor->direction = direction;
}

void Motor_SetVelocity(Motor_Handler_t *motor, float rpm) {
    if (motor == NULL) return;

    if (rpm < 0.0f) {
        motor->direction = MOTOR_BACKWARD;
        motor->refSpeed = constrainf(-rpm, MIN_RPM, MAX_RPM);
    }
    else if (rpm > 0.0f) {
        motor->direction = MOTOR_FORWARD;
        motor->refSpeed = constrainf(rpm, MIN_RPM, MAX_RPM);
    }
    else {
        motor->refSpeed = 0.0f;
        motor->direction = MOTOR_BRAKE;
    }
}

void Motor_ApplyDir(Motor_Handler_t *motor) {
    if(motor == NULL) return;

    switch (motor->direction) {
		case MOTOR_FORWARD:
            HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_RESET);
            break;

		case MOTOR_BACKWARD:
            HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_SET);
			break;

		case MOTOR_BRAKE:
            HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_SET);

			break;
		default:
			break;
	}
}

void Motor_Step(Motor_Handler_t *motor, float dt, uint32_t now_ms) {
    if(motor == NULL) return;

    motor->rpm = Encoder_GetRPM(motor->encoder);

    // Set reference
    float ref = motor->refSpeed;

    if(ref == 0.0f) {
        motor->speed = 0.0f;
    } else if(motor->speed == 0.0f) {
        motor->speed = ref;
        motor->startTime = now_ms;
        motor->integral = 0.0f;
    } else {
        // Ramp, avoid abrupt changes
        motor->speed += constrainf(ref - motor->speed, -RAMP * dt, RAMP * dt);
    }

    // FeedForard + PI
    float u = 0.0f;
    if(motor->speed > 0.0f) {

        // Feedforward for FF_TIME_MS
        float ff = ffPWM(motor->speed);
        if(now_ms - motor->startTime < FF_TIME_MS) {
            u = ff;
        } else {
            // PI
            float e = motor->speed - motor->rpm;
            float u_pi = ff + motor->Kp * e + motor->integral;
            uint8_t saturation = (u_pi > MAX_PWM && e > 0) || (u_pi < MIN_PWM && e < 0);
            
            if(!saturation && motor->rpm > 0.5f * motor->speed) {
            	motor->integral += motor->Ki * e * dt;
            }
            
            motor->integral = constrainf(motor->integral, -60.0f, 60.0f);
            u = ff + motor->Kp * e + motor->integral;
        }

        u = constrainf(u, MIN_PWM, MAX_PWM);
    } else {
        motor->integral = 0.0f;
    }

    motor->u = u;

    Motor_ApplyDir(motor);
    __HAL_TIM_SET_COMPARE(motor->timer, motor->pwm_ch, (uint32_t)(u * TIM_PERIOD / 255.0f + 0.5f));
}

void Motor_SetGains(Motor_Handler_t *motor, float Kp, float Ki) {
    if(motor == NULL) return;

    motor->Kp = Kp;
    motor->Ki = Ki;
}
