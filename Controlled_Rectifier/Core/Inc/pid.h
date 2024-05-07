/*
 * pid.h
 *
 *  Created on: Feb 1, 2024
 *      Author: viktor
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"
#include "arm_math.h"

typedef struct
{
    float32_t targetCurrent;	// задание на ток
    float32_t ActualCurrent;	// измеренный ток
    float32_t err;			    // ошибка
    float32_t Kp,Ki;		  	// коэффициенты ПИ
    float32_t result;		    // результат регулятора
    float32_t integral;		  	// интеграл
}pid_p;

void PID_init( void);
float32_t PID_realize( float32_t c, float32_t c_r);



#endif /* INC_PID_H_ */
