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
#define Wifi_connect 6
extern uint8_t Kp_modify_flag, Ki_modify_flag, Kd_modify_flag;
extern uint8_t Left_modify_flag, Right_modify_flag;
extern uint8_t cancer_menu;
extern uint8_t cancer_running;
extern float Kp, Ki, Kd;
extern char kp_Rx[5],ki_Rx[5],kd_Rx[5];
char kp_val[5], ki_val[5], kd_val[5];
char kp_str[20], ki_str[20], kd_str[20];
char string[12],string_eng[12];
char Left_str[20], Right_str[20];
char Left_val[5], Right_val[5];
uint8_t menu_display = 1;
uint8_t Menu_type = 1;
uint8_t Run_flag = 0;
uint8_t line = 1;
extern int16_t Left, Right;
extern uint16_t Sensor_Threshold[6];
extern uint16_t Sensor_ADC_Value[6];
void Menu_system_control(uint8_t Menu_type, uint8_t line) {
	switch (Menu_type) {
	case 0:
		Running();
		break;
	case Main_menu:
		Mainmenu(line);
		break;
	case Color_Processing:
		Color_Studying_process();
		break;
	case PID_Menu:
		PID_menu(line);
		break;
	case Engine_menu:
		Speed_menu(line);
		break;
	case LineDetect_Show:
		LineDetect_show();
		break;
	case Wifi_connect:
		Wifi_Connect_establish();
		break;
	}
}
void Mainmenu(uint8_t line) {
	switch (line) {
	case 1:
		lcd_send_cmd(0x80 | 0x00);
		lcd_send_string(">Car Run            ");
		lcd_send_cmd(0x80 | 0x40);
		lcd_send_string(" Color studying     ");
		lcd_send_cmd(0x80 | 0x14);
		lcd_send_string(" PID value modify   ");
		lcd_send_cmd(0x80 | 0x54);
		lcd_send_string(" Max speed config   ");
		break;
	case 2:
		lcd_send_cmd(0x80 | 0x00);
		lcd_send_string(" Car Run            ");
		lcd_send_cmd(0x80 | 0x40);
		lcd_send_string(">Color studying     ");
		lcd_send_cmd(0x80 | 0x14);
		lcd_send_string(" PID value modify   ");
		lcd_send_cmd(0x80 | 0x54);
		lcd_send_string(" Max speed config   ");
		break;
	case 3:
		lcd_send_cmd(0x80 | 0x00);
		lcd_send_string(" Car Run            ");
		lcd_send_cmd(0x80 | 0x40);
		lcd_send_string(" Color studying     ");
		lcd_send_cmd(0x80 | 0x14);
		lcd_send_string(">PID value modify   ");
		lcd_send_cmd(0x80 | 0x54);
		lcd_send_string(" Max speed config   ");
		break;
	case 4:
		lcd_send_cmd(0x80 | 0x00);
		lcd_send_string(" Car Run            ");
		lcd_send_cmd(0x80 | 0x40);
		lcd_send_string(" Color studying     ");
		lcd_send_cmd(0x80 | 0x14);
		lcd_send_string(" PID value modify   ");
		lcd_send_cmd(0x80 | 0x54);
		lcd_send_string(">Max speed config   ");
		break;
	case 5:
		lcd_send_cmd(0x80 | 0x00);
		lcd_send_string(">Line Detect show   ");
		lcd_send_cmd(0x80 | 0x40);
		lcd_send_string(" Save system value  ");
		lcd_send_cmd(0x80 | 0x14);
		lcd_send_string(" Wifi Connect       ");
		lcd_send_cmd(0x80 | 0x54);
		lcd_send_string("                    ");
		break;
	case 6:
		lcd_send_cmd(0x80 | 0x00);
		lcd_send_string(" Line Detect show   ");
		lcd_send_cmd(0x80 | 0x40);
		lcd_send_string(">Save system value  ");
		lcd_send_cmd(0x80 | 0x14);
		lcd_send_string(" Wifi Connect       ");
		lcd_send_cmd(0x80 | 0x54);
		lcd_send_string("                    ");
		break;
	case 7:
		lcd_send_cmd(0x80 | 0x00);
		lcd_send_string(" Line Detect show   ");
		lcd_send_cmd(0x80 | 0x40);
		lcd_send_string(" Save system value  ");
		lcd_send_cmd(0x80 | 0x14);
		lcd_send_string(">Wifi Connect       ");
		lcd_send_cmd(0x80 | 0x54);
		lcd_send_string("                    ");
		break;
	}
}
void PID_menu(uint8_t line) {

	switch (line) {
	case 1:
		sprintf(kp_str, ">Kp = %1.2f", Kp);
		lcd_send_cmd(0x80 | 0x00);
		lcd_send_string(kp_str);
		sprintf(ki_str, " Ki = %1.2f", Ki);
		lcd_send_cmd(0x80 | 0x40);
		lcd_send_string(ki_str);
		sprintf(kd_str, " Kd = %1.2f", Kd);
		lcd_send_cmd(0x80 | 0x14);
		lcd_send_string(kd_str);
		lcd_send_cmd(0x80 | 0x54);
		lcd_send_string(" Return to main menu");
		break;
	case 2:
		sprintf(kp_str, " Kp = %1.2f", Kp);
		lcd_send_cmd(0x80 | 0x00);
		lcd_send_string(kp_str);
		sprintf(ki_str, ">Ki = %1.2f", Ki);
		lcd_send_cmd(0x80 | 0x40);
		lcd_send_string(ki_str);
		sprintf(kd_str, " Kd = %1.2f", Kd);
		lcd_send_cmd(0x80 | 0x14);
		lcd_send_string(kd_str);
		lcd_send_cmd(0x80 | 0x54);
		lcd_send_string(" Return to main menu");
		break;
	case 3:
		sprintf(kp_str, " Kp = %1.2f", Kp);
		lcd_send_cmd(0x80 | 0x00);
		lcd_send_string(kp_str);
		sprintf(ki_str, " Ki = %1.2f", Ki);
		lcd_send_cmd(0x80 | 0x40);
		lcd_send_string(ki_str);
		sprintf(kd_str, ">Kd = %1.2f", Kd);
		lcd_send_cmd(0x80 | 0x14);
		lcd_send_string(kd_str);
		lcd_send_cmd(0x80 | 0x54);
		lcd_send_string(" Return to main menu");
		break;
	case 4:
		sprintf(kp_str, " Kp = %1.2f", Kp);
		lcd_send_cmd(0x80 | 0x00);
		lcd_send_string(kp_str);
		sprintf(ki_str, " Ki = %1.2f", Ki);
		lcd_send_cmd(0x80 | 0x40);
		lcd_send_string(ki_str);
		sprintf(kd_str, " Kd = %1.2f", Kd);
		lcd_send_cmd(0x80 | 0x14);
		lcd_send_string(kd_str);
		lcd_send_cmd(0x80 | 0x54);
		lcd_send_string(">Return to main menu");
		break;
	}
}
void Speed_menu(uint8_t line) {
	switch (line) {
	case 1:
		sprintf(Left_str, ">Left Eng = %d", Left);
		lcd_send_cmd(0x80 | 0x00);
		lcd_send_string(Left_str);
		sprintf(Right_str, " Right Eng = %d", Right);
		lcd_send_cmd(0x80 | 0x40);
		lcd_send_string(Right_str);
		lcd_send_cmd(0x80 | 0x14);
		lcd_send_string(" Return to main menu");
		break;
	case 2:
		sprintf(Left_str, " Left Eng = %d", Left);
		lcd_send_cmd(0x80 | 0x00);
		lcd_send_string(Left_str);
		sprintf(Right_str, ">Right Eng = %d", Right);
		lcd_send_cmd(0x80 | 0x40);
		lcd_send_string(Right_str);
		lcd_send_cmd(0x80 | 0x14);
		lcd_send_string(" Return to main menu");
		break;
	case 3:
		sprintf(Left_str, " Left Eng = %d", Left);
		lcd_send_cmd(0x80 | 0x00);
		lcd_send_string(Left_str);
		sprintf(Right_str, " Right Eng = %d", Right);
		lcd_send_cmd(0x80 | 0x40);
		lcd_send_string(Right_str);
		lcd_send_cmd(0x80 | 0x14);
		lcd_send_string(">Return to main menu");
		break;
	}
}
void Color_Studying_process(void) {
	for (int i = 0; i < 5; i++) {
		lcd_send_cmd(0x80 | 0x00);
		lcd_send_string("Processing .       ");
		HAL_Delay(500);
		lcd_send_cmd(0x80 | 0x00);
		lcd_send_string("Processing . .     ");
		HAL_Delay(500);
		lcd_send_cmd(0x80 | 0x00);
		lcd_send_string("Processing . . .   ");
		HAL_Delay(500);
		lcd_clear();
	}
	HAL_Delay(100);
	lcd_send_cmd(0x80 | 0x00);
	lcd_send_string("Done               ");
	HAL_Delay(500);
	Menu_type = Main_menu;
	lcd_clear();
}
void LineDetect_show(void) {
	lcd_send_cmd(0x80 | 0x00);
	lcd_send_string("Line Detect        ");
	lcd_send_cmd(0x80 | 0x40);
	lcd_send_string("Press C for cancer ");
	while (cancer_menu) {
		for (int i = 0; i < 6; i++) {
			if (Sensor_ADC_Value[0] > Sensor_Threshold[0]) {
				lcd_send_cmd(0x80 | 0x16);
				lcd_send_string("1");
			}
			if (Sensor_ADC_Value[1] > Sensor_Threshold[1]) {
				lcd_send_cmd(0x80 | 0x18);
				lcd_send_string("1");
			}
			if (Sensor_ADC_Value[2] > Sensor_Threshold[2]) {
				lcd_send_cmd(0x80 | 0x1A);
				lcd_send_string("1");
			}
			if (Sensor_ADC_Value[3] > Sensor_Threshold[3]) {
				lcd_send_cmd(0x80 | 0x1C);
				lcd_send_string("1");
			}
			if (Sensor_ADC_Value[4] > Sensor_Threshold[4]) {
				lcd_send_cmd(0x80 | 0x1E);
				lcd_send_string("1");
			}
			if (Sensor_ADC_Value[5] > Sensor_Threshold[5]) {
				lcd_send_cmd(0x80 | 0x20);
				lcd_send_string("1");
			}
			if (Sensor_ADC_Value[0] < Sensor_Threshold[0]) {
				lcd_send_cmd(0x80 | 0x16);
				lcd_send_string(" ");
			}
			if (Sensor_ADC_Value[1] < Sensor_Threshold[1]) {
				lcd_send_cmd(0x80 | 0x18);
				lcd_send_string(" ");
			}
			if (Sensor_ADC_Value[2] < Sensor_Threshold[2]) {
				lcd_send_cmd(0x80 | 0x1A);
				lcd_send_string(" ");
			}
			if (Sensor_ADC_Value[3] < Sensor_Threshold[3]) {
				lcd_send_cmd(0x80 | 0x1C);
				lcd_send_string(" ");
			}
			if (Sensor_ADC_Value[4] < Sensor_Threshold[4]) {
				lcd_send_cmd(0x80 | 0x1E);
				lcd_send_string(" ");
			}
			if (Sensor_ADC_Value[5] < Sensor_Threshold[5]) {
				lcd_send_cmd(0x80 | 0x20);
				lcd_send_string(" ");
			}
		}
	}
	lcd_clear();
}
void Running(void) // Activate the car for running
{
	while (cancer_running) {
		lcd_send_cmd(0x80 | 0x00);
		lcd_send_string("Car is Running!        ");
		lcd_send_cmd(0x80 | 0x40);
		lcd_send_string("Press C for cancer     ");
	}
	lcd_clear();
}
void Saving_Process(void)
{
		sprintf(kp_val,"%1.2f ",Kp);
		strcat(string,kp_val);
		sprintf(ki_val,"%1.2f ",Ki);
		strcat(string,ki_val);
		sprintf(kd_val,"%1.2f ",Kd);
		strcat(string,kd_val);
//		sprintf(Left_val,"%d ",Left);
//		strcat(string,Left_val);
//		sprintf(Right_val,"%d ",Right);
//		strcat(string,Right_val);
		Flash_Write_Data(0x08020000, string);
		HAL_NVIC_SystemReset();
}
void Wifi_Connect_establish(void)
{
		for (int i = 0; i < 5; i++) {
			lcd_send_cmd(0x80 | 0x00);
			lcd_send_string("Connecting .       ");
			HAL_Delay(500);
			lcd_send_cmd(0x80 | 0x00);
			lcd_send_string("Connecting . .     ");
			HAL_Delay(500);
			lcd_send_cmd(0x80 | 0x00);
			lcd_send_string("Connecting . . .   ");
			HAL_Delay(500);
			lcd_clear();
		}
		HAL_Delay(100);
		lcd_send_cmd(0x80 | 0x00);
		lcd_send_string("Done               ");
		HAL_Delay(500);
		Menu_type = Main_menu;
		lcd_clear();
}
void executeAction(uint8_t line) {
	switch (line) {
	case 1:
		switch (Menu_type) {
		case Main_menu:
			cancer_running = 1;
			Menu_type = Running_Process;
			break;
		case PID_Menu:
			if (Kp_modify_flag == 0) {
				Kp_modify_flag = 1;
				line = 1;
			} else {
				Kp_modify_flag = 0;
			}
			break;
		case Engine_menu:
			if (Left_modify_flag == 0) {
				Left_modify_flag = 1;
				line = 1;
			} else {
				Left_modify_flag = 0;
			}
			break;

		}
		lcd_clear();
		break;
	case 2:
		switch (Menu_type) {
		case Main_menu:
			Menu_type = Color_Processing;
			break;
		case PID_Menu:
			if (Ki_modify_flag == 0) {
				Ki_modify_flag = 1;
				line = 2;
			} else {
				Ki_modify_flag = 0;
			}
			break;
		case Engine_menu:
			if (Right_modify_flag == 0) {
				Right_modify_flag = 1;
				line = 2;
			} else {
				Right_modify_flag = 0;
			}
			break;
		}
		lcd_clear();
		break;
	case 3:
		switch (Menu_type) {
		case Main_menu:
			Menu_type = PID_Menu;
			break;
		case PID_Menu:
			if (Kd_modify_flag == 0) {
				Kd_modify_flag = 1;
				line = 3;
			} else {
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
		switch (Menu_type) {
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
		switch (Menu_type) {
		case Main_menu:
			cancer_menu = 1;
			Menu_type = LineDetect_Show;
			break;
		}
		lcd_clear();
		break;
	case 6:
		Saving_Process();
		Menu_type = Main_menu;
		break;
	case 7:
		Menu_type = Wifi_connect;
		lcd_clear();
		break;
	}

}
