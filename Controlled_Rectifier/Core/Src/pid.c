/*
 * pid.c
 *
 *  Created on: Feb 1, 2024
 *      Author: viktor
 */

#include "pid.h"

pid_p pid;

// заполнение структуры для регулятора PI
void PID_init()
{
    pid.targetCurrent= 0.0f;		// заданное значение тока
    pid.ActualCurrent= 0.0f;		// фактическое значение ADC тока
    pid.err= 0.0f;				    // текущее ошибка фактического и заданного
    pid.integral= 0.0f;			  	// значение интеграла
    pid.Kp= 800.0f;				    // пропорциональный коэффициент
    pid.Ki= 7.4f;				    	// интегральный коэффициент

}

float PID_realize( float32_t c, float32_t c_r)
{
    pid.targetCurrent = c;									// задание на ток
    pid.ActualCurrent = c_r;								// измеренное значение тока
    pid.err = pid.targetCurrent - pid.ActualCurrent;		// расчёт ошибки
    pid.integral += pid.err;								// накопление интеграла
        if (pid.integral > 1000U) {pid.integral = 1000U;}	// насыщение интеграла
    	else if (pid.integral < 0U) {pid.integral = 0U;}

    pid.result = (pid.Kp * pid.err) + (pid.integral * pid.Ki);// формула ПИ
    if (pid.result > 10000U) {pid.result = 10000U;}				// ограничение регулятора
        else if (pid.result < 0U) {pid.result = 0U;}

    return pid.result;
}

