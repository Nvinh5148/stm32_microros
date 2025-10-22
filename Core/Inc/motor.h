/*
 * motor.h
 *
 *  Created on: Oct 10, 2025
 *      Author: vinh
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#define IN1_Pin        GPIO_PIN_14
#define IN1_GPIO_Port  GPIOB

#define IN2_Pin        GPIO_PIN_15
#define IN2_GPIO_Port  GPIOB

#define IN3_Pin        GPIO_PIN_12
#define IN3_GPIO_Port  GPIOB

#define IN4_Pin        GPIO_PIN_13
#define IN4_GPIO_Port  GPIOB


#define IN5_Pin        GPIO_PIN_3
#define IN5_GPIO_Port  GPIOB

#define IN6_Pin        GPIO_PIN_4
#define IN6_GPIO_Port  GPIOB



#define PPR 4436.0f


#define SAMPLE_TIME 0.01f
#define ENC_LEFT   46039   // giá trị encoder max bên trái
#define ENC_RIGHT  15703   // giá trị encoder max bên phải
#define ANGLE_LEFT  -30.0f
#define ANGLE_RIGHT  30.0f

#include <stdint.h>
#include "pid.h"
#include "main.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;



typedef struct
{
	int32_t counter;
	float position;
	float velocity;
	float setPoint;
	int32_t last_count;
} Motor_t;

typedef struct
{
	int32_t cnter;
	float POS;
	float Taget_angle;
	int32_t last_cnt;
} MOTOR_t;


void MotorReset(Motor_t *tmotor);
void MotorAngleInit(MOTOR_t *tMOTOR);
void ReadEncoder(Motor_t *tmotor, TIM_HandleTypeDef *htim);
void ReadEncoder_angular(MOTOR_t *tMOTOR, TIM_HandleTypeDef *htim);
float EncoderToAngle(int32_t cnt);
float MotorPIDVelocity(PID_CONTROL_t *PID_Ctrl, Motor_t *tmotor, float vel_set);
float MotorPIDPosition(PID_CONTROL_t *PID_Ctrl, MOTOR_t *tMOTOR, float pos_set);
void MotorSetDuty1( int duty);
void MotorSetDuty2( int duty);
void MotorSetDuty3( int g_duty);


#endif /* INC_MOTOR_H_ */
