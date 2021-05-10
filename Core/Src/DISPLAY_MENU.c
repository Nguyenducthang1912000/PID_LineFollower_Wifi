/*
 * Menu.c
 *
 *  Created on: May 6, 2021
 *      Author: DucThang
 */
#include <DISPLAY_MENU.h>

#define Running_Process 0
#define Main_menu 1
#define Color_Processing 2
#define PID_Menu 3
#define Engine_menu 4
#define LineDetect_Show 5
extern uint8_t Kp_modify_flag,Ki_modify_flag,Kd_modify_flag ;
extern uint8_t Left_modify_flag,Right_modify_flag;
extern uint8_t cancer_menu;
extern uint8_t cancer_running;
extern float Kp,Ki,Kd;
char kp_str[12],ki_str[12],kd_str[12];
char Left_str[20],Right_str[20];
uint8_t menu_display = 1;
uint8_t Menu_type = 1;
uint8_t Run_flag = 0;
uint8_t line = 1;
extern uint16_t Left,Right;
extern uint16_t Sensor_Threshold[6];
extern uint16_t Sensor_ADC_Value[6];
void Menu_system_control(uint8_t Menu_type,uint8_t line)
{
	switch(Menu_type){
	case 0:
		Running();
		break;
	case 1:
		Mainmenu(line);
		break;
	case 2:
		Color_Studying_process();
		break;
	case 3:
		PID_menu(line);
		break;
	case 4:
		Speed_menu(line);
		break;
	case 5:
		LineDetect_show();
		break;
	}
}
void Mainmenu(uint8_t line)
{
	switch(line){
	case 1:
		lcd_send_cmd (0x80|0x00);
		lcd_send_string(">Car Run            ");
		lcd_send_cmd (0x80|0x40);
		lcd_send_string(" Color studying     ");
		lcd_send_cmd (0x80|0x14);
		lcd_send_string(" PID value modify   ");
		lcd_send_cmd (0x80|0x54);
		lcd_send_string(" Max speed config   ");
		break;
	case 2:
		lcd_send_cmd (0x80|0x00);
		lcd_send_string(" Car Run            ");
		lcd_send_cmd (0x80|0x40);
		lcd_send_string(">Color studying     ");
		lcd_send_cmd (0x80|0x14);
		lcd_send_string(" PID value modify   ");
		lcd_send_cmd (0x80|0x54);
		lcd_send_string(" Max speed config   ");
		break;
	case 3:
		lcd_send_cmd (0x80|0x00);
		lcd_send_string(" Car Run            ");
		lcd_send_cmd (0x80|0x40);
		lcd_send_string(" Color studying     ");
		lcd_send_cmd (0x80|0x14);
		lcd_send_string(">PID value modify   ");
		lcd_send_cmd (0x80|0x54);
		lcd_send_string(" Max speed config   ");
		break;
	case 4:
		lcd_send_cmd (0x80|0x00);
		lcd_send_string(" Car Run            ");
		lcd_send_cmd (0x80|0x40);
		lcd_send_string(" Color studying     ");
		lcd_send_cmd (0x80|0x14);
		lcd_send_string(" PID value modify   ");
		lcd_send_cmd (0x80|0x54);
		lcd_send_string(">Max speed config   ");
		break;
	case 5:
		lcd_send_cmd (0x80|0x00);
		lcd_send_string(">Line Detect show   ");
		lcd_send_cmd (0x80|0x40);
		lcd_send_string("                    ");
		lcd_send_cmd (0x80|0x14);
		lcd_send_string("                    ");
		lcd_send_cmd (0x80|0x54);
		lcd_send_string("                    ");
		break;
	}
}
void PID_menu(uint8_t line){

	switch(line){
	case 1:
		sprintf(kp_str,">Kp = %1.2f",(float)Kp);
		lcd_send_cmd (0x80|0x00);
		lcd_send_string(kp_str);
		sprintf(ki_str," Ki = %1.2f",Ki);
		lcd_send_cmd (0x80|0x40);
		lcd_send_string(ki_str);
		sprintf(kd_str," Kd = %1.2f",Kd);
		lcd_send_cmd (0x80|0x14);
		lcd_send_string(kd_str);
		lcd_send_cmd (0x80|0x54);
		lcd_send_string(" Save & return      ");
		break;
	case 2:
		sprintf(kp_str," Kp = %1.2f",Kp);
		lcd_send_cmd (0x80|0x00);
		lcd_send_string(kp_str);
		sprintf(ki_str,">Ki = %1.2f",Ki);
		lcd_send_cmd (0x80|0x40);
		lcd_send_string(ki_str);
		sprintf(kd_str," Kd = %1.2f",Kd);
		lcd_send_cmd (0x80|0x14);
		lcd_send_string(kd_str);
		lcd_send_cmd (0x80|0x54);
		lcd_send_string(" Save & return     ");
		break;
	case 3:
		sprintf(kp_str," Kp = %1.2f",Kp);
		lcd_send_cmd (0x80|0x00);
		lcd_send_string(kp_str);
		sprintf(ki_str," Ki = %1.2f",Ki);
		lcd_send_cmd (0x80|0x40);
		lcd_send_string(ki_str);
		sprintf(kd_str,">Kd = %1.2f",Kd);
		lcd_send_cmd (0x80|0x14);
		lcd_send_string(kd_str);
		lcd_send_cmd (0x80|0x54);
		lcd_send_string(" Save & return     ");
		break;
	case 4:
		sprintf(kp_str," Kp = %1.2f",Kp);
		lcd_send_cmd (0x80|0x00);
		lcd_send_string(kp_str);
		sprintf(ki_str," Ki = %1.2f",Ki);
		lcd_send_cmd (0x80|0x40);
		lcd_send_string(ki_str);
		sprintf(kd_str," Kd = %1.2f",Kd);
		lcd_send_cmd (0x80|0x14);
		lcd_send_string(kd_str);
		lcd_send_cmd (0x80|0x54);
		lcd_send_string(">Save & return     ");
		break;
	}
}
void Speed_menu(uint8_t line)
{
	switch(line){
	case 1:
		sprintf(Left_str,">Left Engine = %u",Left);
		lcd_send_cmd (0x80|0x00);
		lcd_send_string(Left_str);
		sprintf(Right_str," Right Engine = %u",Right);
		lcd_send_cmd (0x80|0x40);
		lcd_send_string(Right_str);
		lcd_send_cmd (0x80|0x14);
		lcd_send_string(" Save & return     ");
		break;
	case 2:
		sprintf(Left_str," Left Engine = %u",Left);
		lcd_send_cmd (0x80|0x00);
		lcd_send_string(Left_str);
		sprintf(Right_str,">Right Engine = %u",Right);
		lcd_send_cmd (0x80|0x40);
		lcd_send_string(Right_str);
		lcd_send_cmd (0x80|0x14);
		lcd_send_string(" Save & return     ");
		break;
	case 3:
		sprintf(Left_str," Left Engine = %u",Left);
		lcd_send_cmd (0x80|0x00);
		lcd_send_string(Left_str);
		sprintf(Right_str," Right Engine = %u",Right);
		lcd_send_cmd (0x80|0x40);
		lcd_send_string(Right_str);
		lcd_send_cmd (0x80|0x14);
		lcd_send_string(">Save & return     ");
		break;
	}
}
void Color_Studying_process(void)
{
	for(int i = 0;i<5;i++){
	lcd_send_cmd (0x80|0x00);
	lcd_send_string("Processing .       ");
	HAL_Delay(500);
	lcd_send_cmd (0x80|0x00);
	lcd_send_string("Processing . .     ");
	HAL_Delay(500);
	lcd_send_cmd (0x80|0x00);
	lcd_send_string("Processing . . .   ");
	HAL_Delay(500);
	lcd_clear();
	}
	HAL_Delay(100);
	lcd_send_cmd (0x80|0x00);
	lcd_send_string("Done               ");
	HAL_Delay(500);
	Menu_type = Main_menu;
	lcd_clear();
}
void LineDetect_show(void)
{
	lcd_send_cmd (0x80|0x00);
	lcd_send_string("Line Detect        ");
	lcd_send_cmd (0x80|0x40);
	lcd_send_string("Press C for cancer ");
	while(cancer_menu){
		for(int i=0;i<6;i++)
		{
			if(Sensor_ADC_Value[0] > Sensor_Threshold[0]){
				lcd_send_cmd (0x80|0x16);
				lcd_send_string("1");}
			if(Sensor_ADC_Value[1] > Sensor_Threshold[1]){
				lcd_send_cmd (0x80|0x18);
				lcd_send_string("1");}
			if(Sensor_ADC_Value[2] > Sensor_Threshold[2]){
				lcd_send_cmd (0x80|0x1A);
				lcd_send_string("1");}
			if(Sensor_ADC_Value[3] > Sensor_Threshold[3]){
				lcd_send_cmd (0x80|0x1C);
				lcd_send_string("1");}
			if(Sensor_ADC_Value[4] > Sensor_Threshold[4]){
				lcd_send_cmd (0x80|0x1E);
				lcd_send_string("1");}
			if(Sensor_ADC_Value[5] > Sensor_Threshold[5]){
				lcd_send_cmd (0x80|0x20);
				lcd_send_string("1");}
			if(Sensor_ADC_Value[0] < Sensor_Threshold[0]){
				lcd_send_cmd (0x80|0x16);
				lcd_send_string(" ");}
			if(Sensor_ADC_Value[1] < Sensor_Threshold[1]){
				lcd_send_cmd (0x80|0x18);
				lcd_send_string(" ");}
			if(Sensor_ADC_Value[2] < Sensor_Threshold[2]){
				lcd_send_cmd (0x80|0x1A);
				lcd_send_string(" ");}
			if(Sensor_ADC_Value[3] < Sensor_Threshold[3]){
				lcd_send_cmd (0x80|0x1C);
				lcd_send_string(" ");}
			if(Sensor_ADC_Value[4] < Sensor_Threshold[4]){
				lcd_send_cmd (0x80|0x1E);
				lcd_send_string(" ");}
			if(Sensor_ADC_Value[5] < Sensor_Threshold[5]){
				lcd_send_cmd (0x80|0x20);
				lcd_send_string(" ");}
		}
	}
	lcd_clear();
}
void Running(void) // Activate the car for running
{
	while(cancer_running){
		lcd_send_cmd (0x80|0x00);
		lcd_send_string("Car is Running!        ");
		lcd_send_cmd (0x80|0x40);
		lcd_send_string("Press C for cancer     ");
	}
	lcd_clear();
}
void executeAction(uint8_t line)
{
	switch(line)
	{
	case 1:
		switch(Menu_type){
			case Main_menu:
				cancer_running = 1;
				Menu_type = Running_Process;
				break;
			case PID_Menu:
				if(Kp_modify_flag == 0)
				{
					Kp_modify_flag = 1;
					line = 1;
				}
				else
				{
					Kp_modify_flag = 0;
				}
				break;
			case Engine_menu:
				if( Left_modify_flag== 0)
				{
					Left_modify_flag = 1;
					line = 1;
				}
				else
				{
					Left_modify_flag = 0;
				}
				break;

			}
		lcd_clear();
		break;
	case 2:
		switch(Menu_type){
			case Main_menu:
				Menu_type = Color_Processing;
				break;
			case PID_Menu:
				if(Ki_modify_flag == 0)
				{
					Ki_modify_flag = 1;
					line = 2;
				}
				else
				{
					Ki_modify_flag = 0;
				}
				break;
			case Engine_menu:
				if( Right_modify_flag== 0)
				{
					Right_modify_flag = 1;
					line = 2;
				}
				else
				{
					Right_modify_flag = 0;
				}
				break;
			}
		lcd_clear();
		break;
	case 3:
		switch(Menu_type){
			case Main_menu:
				Menu_type = PID_Menu;
				break;
			case PID_Menu:
				if(Kd_modify_flag == 0)
				{
					Kd_modify_flag = 1;
					line = 3;
				}
				else
				{
					Kd_modify_flag = 0;
				}
				break;
			case Engine_menu:
				Menu_type = Main_menu;
				break;

		}
		lcd_clear();
		break;
	case 4:
		switch(Menu_type){
			case Main_menu:
				Menu_type = Engine_menu;
				break;
			case PID_Menu:
				Menu_type = Main_menu;
				break;
			}
		lcd_clear();
		break;
	case 5:
		switch(Menu_type){
			case Main_menu:
				cancer_menu = 1;
				Menu_type = LineDetect_Show;
				break;
		}
		lcd_clear();
		break;
	}
}
