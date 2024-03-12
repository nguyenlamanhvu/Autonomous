/*
 * motor.c
 *
 *  Created on: Mar 10, 2024
 *      Author: Nguyen Lam Anh Vu
 */

/*********************
 *      INCLUDES
 *********************/

#include "motor.h"
#include <stdlib.h>

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
/*
 * The function creates and initializes a new instance of the MOTOR structure.
 *
 * @return a pointer to a structure of type motor_t;
 */
motor_t *MOTOR_Create()
{
	motor_t *motor_p = malloc(sizeof(motor_t));
	if(motor_p != NULL)
	{
		MOTOR_Init(motor_p,MOTOR_ControlSpeed);
	}
	return motor_p;
}

/*
 * The function MOTOR_Destroy frees the memory allocated for an motor_t structure.
 *
 * @param motor_p motor_p is a pointer to a structure of type motor_t.
 */
void MOTOR_Destroy(motor_t * const motor_p)
{
	free(motor_p);
}

/*
 * The function MOTOR_Init initializes TIMER1_CH1 with PWM mode
 *
 * @param1: motor_p is a pointer to a structure of type motor_t.
 * @param2: (*ctrl_speed_func)(motor_t *motor_p) is a control speed function.
 */
void MOTOR_Init(motor_t *motor_p, void (*ctrl_speed_func)(motor_t *motor_p))
{
	motor_p->control_speed = ctrl_speed_func;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
}

/*
 * The function MOTOR_ControlSpeed controls speed of the motor.
 *
 * @param1: motor_p is a pointer to a structure of type motor_t.
 */
void MOTOR_ControlSpeed(motor_t *motor_p)
{
//	TIM1->CCR1 = motor_p->Prop_p.speed;
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,motor_p->Prop_p.out_speed);
}
