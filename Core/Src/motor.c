/*
 * motor.c
 *
 *  Created on: Oct 10, 2025
 *      Author: vinh
 */


#include "motor.h"
#include "main.h"
#include <stdlib.h>
#include <math.h>
#include "main.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

void MotorReset(Motor_t *tmotor)
{
tmotor->counter = 0;
tmotor->position = 0.0f;
tmotor->velocity = 0.0f;
tmotor->setPoint = 0.0f;
}
void MotorAngleInit(MOTOR_t *tMOTOR)
{
	tMOTOR->cnter = 0;
	tMOTOR->POS = 0.0f;
	tMOTOR->Taget_angle = 0.0f;
}
void ReadEncoder(Motor_t *tmotor, TIM_HandleTypeDef *htim)
{
    static int32_t last_count = 0;
    int32_t count = __HAL_TIM_GET_COUNTER(htim);
    int32_t diff = count - tmotor->last_count;


    if (diff > 32768)
        diff -= 65536;
    else if (diff < -32768)
        diff += 65536;

    tmotor->counter = count;
    tmotor->position += (float)diff / PPR;   //position
    float speed_rps = (diff / PPR) / SAMPLE_TIME;  // vòng/giây
    tmotor->velocity = speed_rps;
    tmotor->last_count = count;
}


void ReadEncoder_angular(MOTOR_t *tMOTOR, TIM_HandleTypeDef *htim)
{
    int32_t cnt = __HAL_TIM_GET_COUNTER(htim);
        int32_t DIFF = cnt - tMOTOR->last_cnt;
    if (DIFF > 32768)
            DIFF -= 65536;
        else if (DIFF < -32768)
            DIFF += 65536;
    float K = 30.0f / 16000.0f;
    tMOTOR->cnter = cnt;
    tMOTOR->POS += (float)DIFF*K ;  //position
    tMOTOR->last_cnt = cnt;
}
void MotorSetDuty1(int nDuty)
{
	if (nDuty > 4200) nDuty = 4200;
	if (nDuty < -4200) nDuty = -4200;

	if (nDuty >= 0)
	{
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, nDuty);
	}
	else
	{
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, abs(nDuty));
	}
}
void MotorSetDuty2(int nDuty)
{
	if (nDuty > 4200) nDuty = 4200;
	if (nDuty < -4200) nDuty = -4200;

	if (nDuty >= 0)
	{
    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, nDuty);
	}
	else
	{
    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, abs(nDuty));
	}
}


void MotorSetDuty3(int nDuty)
{
	if (nDuty > 4200) nDuty = 4200;
	if (nDuty < -4200) nDuty = -4200;

	if (nDuty >= 0)
	{
    HAL_GPIO_WritePin(IN5_GPIO_Port, IN5_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN6_GPIO_Port, IN6_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, nDuty);
	}
	else
	{
    HAL_GPIO_WritePin(IN5_GPIO_Port, IN5_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN6_GPIO_Port, IN6_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, abs(nDuty));
	}
}




float MotorPIDPosition(PID_CONTROL_t *PID_Ctrl , MOTOR_t *tMOTOR, float pos_set)
{
	 if (pos_set > ANGLE_RIGHT )
		{
			pos_set = ANGLE_RIGHT ;
		}
	  else if (pos_set < ANGLE_LEFT)
		{
			pos_set = ANGLE_LEFT;
		}
	tMOTOR->Taget_angle = pos_set;
	float g_duty = PIDCompute(PID_Ctrl, tMOTOR->Taget_angle, tMOTOR->POS, 0.03f);
	return g_duty;
}

float MotorPIDVelocity(PID_CONTROL_t *PID_Ctrl , Motor_t *tmotor, float vel_set)
{
	tmotor->setPoint = vel_set;
	float duty = PIDCompute(PID_Ctrl, tmotor->setPoint, tmotor->velocity, 0.01f);
	return duty;
}
