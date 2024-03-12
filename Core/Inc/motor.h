/*
 * motor.h
 *
 *  Created on: Mar 10, 2024
 *      Author: Nguyen Lam Anh Vu
 */
#ifndef MOTOR_H
#define MOTOR_H

/*********************
 *      INCLUDES
 *********************/

#include <stdint.h>
#include "main.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/
extern TIM_HandleTypeDef htim1;		//Timer for PWM mode
extern TIM_HandleTypeDef htim2;		//Timer for Encoder mode

/* Motor structure */
typedef struct _MOTOR_prop
{
	int16_t  out_speed;
	int16_t  timer_counter;
	int16_t  measure_speed;
	int16_t  set_point;
}MOTOR_prop_t;		//motor properties structure

typedef struct motor motor_t;

struct motor
{
	void (*control_speed)(motor_t *const motor_p);
	MOTOR_prop_t Prop_p;
};
/**********************
 *     OPERATION
 **********************/
motor_t *MOTOR_Create();
void MOTOR_Destroy(motor_t * const motor_p);

void MOTOR_Init(motor_t *motor_p, void (*ctrl_speed_func)(motor_t *motor_p));
void MOTOR_ControlSpeed(motor_t *motor_p);

#endif /*MOTOR_H*/

