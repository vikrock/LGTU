/*
 * pid.h
 *
 *  Created on: Feb 1, 2024
 *      Author: viktor
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"

typedef struct
{
    float targetCurrent;	// задание на ток
    float ActualCurrent;	// измеренный ток
    float err;			    // ошибка
    float Kp,Ki;		  	// коэффициенты ПИ
    float result;		    // результат регулятора
    float integral;		  	// интеграл
}pid_p;

void PID_init( void);
float PID_realize( float c, float c_r);



#endif /* INC_PID_H_ */
