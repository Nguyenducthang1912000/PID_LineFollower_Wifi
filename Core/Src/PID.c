/*
 * PID.c
 *
 *  Created on: Jan 8, 2021
 *      Author: DucThang
 */
#include "PID.h"
#include "main.h"
//P: ti le , I: tich phan , D: dao ham,vi phan
float adder_L,adder_R;
int PWM_Output_L,PWM_Output_R;
float Error_L,Error_R;
int16_t PID_val;
//PID for left engine
int RPM_Convert_PWM_L (int16_t RPM_L,float EncoderRead_L,PIDController *pid_L){
	HAL_Delay(2);
	Error_L = RPM_L - EncoderRead_L;
	pid_L->integral = pid_L->integral + Error_L;
	pid_L->derivative = (Error_L - pid_L->previous_error);
	adder_L = pid_L->Kp * Error_L + pid_L->Ki * pid_L->integral + pid_L->Kd * pid_L->derivative;
	pid_L->previous_error = Error_L;
	PWM_Output_L = RPM_L*7200/pid_L->MaxRPM + adder_L;
	return PWM_Output_L;
}
//PID for right engine
int RPM_Convert_PWM_R (int16_t RPM_R,float EncoderRead_R,PIDController *pid_R){
	HAL_Delay(2);
	Error_R = RPM_R - EncoderRead_R;
	pid_R->integral = pid_R->integral + Error_R;
	pid_R->derivative = (Error_R - pid_R->previous_error);
	adder_R = pid_R->Kp * Error_R + pid_R->Ki * pid_R->integral + pid_R->Kd * pid_R->derivative;
	pid_R->previous_error = Error_R;
	PWM_Output_R = RPM_R*7200/388 + adder_R;
	return PWM_Output_R;
}
int16_t Line_Follower_PID (int Setpoint , int Error,PIDController *Car)
{
	Car->propotional = Setpoint - Error;
	Car->integral = Car->integral + Error;
	Car->derivative = Error - Car->previous_error;
	PID_val = (Car->Kp * Car->propotional) + (Car->Ki * Car->integral) + (Car->Kd * Car->derivative);
	Car->previous_error = Error;
	return PID_val;
}
void PIDController_Car_Init (PIDController *Car){
	Car->derivative = 0.0f;
	Car->integral = 0.0f;
	Car->previous_error = 0.0f;
	Car->adder_out = 0.0f;
}
void PIDController_L_Init (PIDController *pid_L)
{
	/*Clear controller variables*/
	pid_L->derivative = 0.0f;
	pid_L->integral = 0.0f;
	pid_L->previous_error = 0.0f;
	pid_L->adder_out = 0.0f;
}
void PIDController_R_Init (PIDController *pid_R)
{
	/*Clear controller variables*/
	pid_R->derivative = 0.0f;
	pid_R->integral = 0.0f;
	pid_R->previous_error = 0.0f;
	pid_R->adder_out = 0.0f;
}
