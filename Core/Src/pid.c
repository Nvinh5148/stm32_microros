/*
 * pid.c
 *
 *  Created on: Oct 10, 2025
 *      Author: vinh
 */

#ifndef SRC_PID_C_
#define SRC_PID_C_

#include "pid.h"

#include "main.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

void PIDReset(PID_CONTROL_t *PID_Ctrl)//pPID->dKp = 1.0f;tuong duong v?i:(*pPID).dKp = 1.0f;
{
		PID_Ctrl->dIntergral = 0.0f;
		PID_Ctrl->dErrorTerm = 0.0f;
		PID_Ctrl->result = 0.0f;
}


void PIDInit(PID_CONTROL_t *PID_Ctrl, float dKp, float dKi, float dKd)
{
		PIDReset(PID_Ctrl);
		PID_Ctrl->dKp = dKp;// = PID_Ctrl.dKp = dKp
		PID_Ctrl->dKi = dKi;
		PID_Ctrl->dKd = dKd;
}

float PIDCompute(PID_CONTROL_t *PID_Ctrl, float SetPoint, float Input, float dTs)
{
	float Error = SetPoint - Input;
	float dP = 0 , dI = 0 , dD = 0;

	dP = PID_Ctrl->dKp*Error;
	PID_Ctrl->dIntergral += Error;
	dI = PID_Ctrl->dKi * dTs/2 * PID_Ctrl->dIntergral;
	dD = PID_Ctrl->dKd * (Error - PID_Ctrl->dErrorTerm)/ dTs;

	PID_Ctrl->result = (dP+dI+dD);
	if(PID_Ctrl->result > 4200)
	{
		PID_Ctrl-> result = 4200;
	}
		if(PID_Ctrl->result < -4200)
	{
		PID_Ctrl-> result = -4200;
	}
	PID_Ctrl->dErrorTerm = Error;
	return PID_Ctrl->result;
}


#endif /* SRC_PID_C_ */
