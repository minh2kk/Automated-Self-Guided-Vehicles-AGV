#include "stm32f4xx.h"                  // Device header

#include "mfrc522.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "cJson.h"
#include "filter.h"

#include "tm_stm32f4_servo.h"
char minh_json[200];
uint8_t tcrt_arr[5] = {0};
uint16_t dis = 0;
char buffer[15];
uint8_t ngat_usart = 0;
uint8_t status;
uint8_t g_ucTempbuf[20];
bool flag_loop=0;
char receive;
char buff[200];
unsigned int buff_index = 0 ;

/*
	TM_SERVO_SetDegrees(&Servo[0], 100);//1
	TM_SERVO_SetDegrees(&Servo[1], 120);//2
	TM_SERVO_SetDegrees(&Servo[2], 20);//3
	TM_SERVO_SetDegrees(&Servo[3], 10);//4
	TM_SERVO_SetDegrees(&Servo[4], 30);//5
*/
uint8_t Robot[5] = {100, 120, 20, 10, 30};

double kp = 3;
double kd = 1.5f;
double ki = 0.00000000001f;
int PWM_Value = 0;
//int PWM_Value_old = 0;
uint8_t max_speed = 80;
int Target = 50;
int Actual=0;
int Error = 0;
int ErrorOld = 0;
int ErrorChange = 0;
int ErrorSlope = 0;
int ErrorArea = 0;
long left = 0;
long right = 0;


cJSON *quan_ly_data;
char R0[10] = "22061B34";
//char R1[10] = "D3F7ECAB";
//char R2[10] = "C3E449AD";
//char R3[10] = "63D3A1AC";
//char R4[10] = "830D9EAC";
//char R5[10] = "B35435AD";
//char R6[10] = "236F4EAD";
char R7[10] = "B3B595AC";
char R8[10] = "838F94AC";
char R9[10] = "53ED3DAD";
char R10[10] = "5328FBAC";




/*

//GLCD
#define KS0108_PORT_DATA  GPIOA
#define KS0108_PORT_CONTROL  GPIOB

#define KS0108_RS    GPIO_Pin_10
#define KS0108_RW    GPIO_Pin_11
#define KS0108_EN    GPIO_Pin_12

#define KS0108_CS1   GPIO_Pin_13
#define KS0108_CS2   GPIO_Pin_14
#define KS0108_CS3   GPIO_Pin_15

#define KS0108_D0    0

#define DISPLAY_STATUS_BUSY	0x80
extern unsigned char screen_x;
extern unsigned char screen_y;

GPIO_InitTypeDef GPIO_InitStructure;
//-------------------------------------------------------------------------------------------------
// Delay function /for 8MHz/
//-------------------------------------------------------------------------------------------------
void GLCD_Delay(void)
{
  __asm("nop");__asm("nop");__asm("nop");__asm("nop");
}
//-------------------------------------------------------------------------------------------------
// Enalbe Controller (0-2)
//-------------------------------------------------------------------------------------------------
void GLCD_EnableController(unsigned char controller)
{
switch(controller){
	case 0 : GPIO_ResetBits(KS0108_PORT_CONTROL, KS0108_CS1); break;
	case 1 : GPIO_ResetBits(KS0108_PORT_CONTROL, KS0108_CS2); break;
	case 2 : GPIO_ResetBits(KS0108_PORT_CONTROL, KS0108_CS3); break;
	}
}
//-------------------------------------------------------------------------------------------------
// Disable Controller (0-2)
//-------------------------------------------------------------------------------------------------
void GLCD_DisableController(unsigned char controller)
{
switch(controller){
	case 0 : GPIO_SetBits(KS0108_PORT_CONTROL, KS0108_CS1); break;
	case 1 : GPIO_SetBits(KS0108_PORT_CONTROL, KS0108_CS2); break;
	case 2 : GPIO_SetBits(KS0108_PORT_CONTROL, KS0108_CS3); break;
	}
}
//-------------------------------------------------------------------------------------------------
// Read Status byte from specified controller (0-2)
//-------------------------------------------------------------------------------------------------
unsigned char GLCD_ReadStatus(unsigned char controller)
{
  unsigned char status;

  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = 0xFF << KS0108_D0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(KS0108_PORT_DATA, &GPIO_InitStructure);

  GPIO_SetBits(KS0108_PORT_CONTROL, KS0108_RW);
  GPIO_ResetBits(KS0108_PORT_CONTROL, KS0108_RS);
  GLCD_EnableController(controller);
  GLCD_Delay();
  GPIO_SetBits(KS0108_PORT_CONTROL, KS0108_EN);
  GLCD_Delay();
  status = ((GPIO_ReadInputData(KS0108_PORT_DATA) >> KS0108_D0) & 0xFF);
  GPIO_ResetBits(KS0108_PORT_CONTROL, KS0108_EN);
  GLCD_DisableController(controller);
  return status;
}
//-------------------------------------------------------------------------------------------------
// Write command to specified controller
//-------------------------------------------------------------------------------------------------
void GLCD_WriteCommand(unsigned char commandToWrite, unsigned char controller)
{
  while(GLCD_ReadStatus(controller)&DISPLAY_STATUS_BUSY);
  
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin  = (0xFF << KS0108_D0);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(KS0108_PORT_DATA, &GPIO_InitStructure);

  GPIO_ResetBits(KS0108_PORT_CONTROL, KS0108_RS | KS0108_RW);
  GLCD_Delay();
  GLCD_EnableController(controller);
  GLCD_Delay();
  GPIO_SetBits(KS0108_PORT_DATA, (commandToWrite << KS0108_D0));
  commandToWrite ^= 0xFF;
  GPIO_ResetBits(KS0108_PORT_DATA, (commandToWrite << KS0108_D0));
  GLCD_Delay();
  GPIO_SetBits(KS0108_PORT_CONTROL, KS0108_EN);
  GLCD_Delay();
  GPIO_ResetBits(KS0108_PORT_CONTROL, KS0108_EN);
  GLCD_Delay();
  GLCD_DisableController(controller);
}

//-------------------------------------------------------------------------------------------------
// Read data from current position
//-------------------------------------------------------------------------------------------------
unsigned char GLCD_ReadData(void)
{
  unsigned char tmp;
  while(GLCD_ReadStatus(screen_x / 64)&DISPLAY_STATUS_BUSY);
  
  GPIO_StructInit(&GPIO_InitStructure);  
  GPIO_InitStructure.GPIO_Pin = 0xFF << KS0108_D0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(KS0108_PORT_DATA, &GPIO_InitStructure);

  GPIO_SetBits(KS0108_PORT_CONTROL, KS0108_RS | KS0108_RW);

  GLCD_EnableController(screen_x / 64);
  GLCD_Delay();
  GPIO_SetBits(KS0108_PORT_CONTROL, KS0108_EN);
  GLCD_Delay();
  tmp = ((GPIO_ReadInputData(KS0108_PORT_DATA) >> KS0108_D0) & 0xFF);
  GPIO_ResetBits(KS0108_PORT_CONTROL, KS0108_EN);
  GLCD_DisableController(screen_x / 64);
  screen_x++;
  return tmp;
}
//-------------------------------------------------------------------------------------------------
// Write data to current position
//-------------------------------------------------------------------------------------------------
void GLCD_WriteData(unsigned char dataToWrite)
{
  while(GLCD_ReadStatus(screen_x / 64)&DISPLAY_STATUS_BUSY); 
  
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = (0xFF << KS0108_D0);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(KS0108_PORT_DATA, &GPIO_InitStructure);

  GPIO_ResetBits(KS0108_PORT_CONTROL, KS0108_RW);
  GLCD_Delay();
  GPIO_SetBits(KS0108_PORT_CONTROL, KS0108_RS);
  GLCD_Delay();
  GPIO_SetBits(KS0108_PORT_DATA, (dataToWrite << KS0108_D0));
  dataToWrite ^= 0xFF;
  GPIO_ResetBits(KS0108_PORT_DATA, (dataToWrite << KS0108_D0));
  GLCD_Delay();
  GLCD_EnableController(screen_x / 64);
  GLCD_Delay();
  GPIO_SetBits(KS0108_PORT_CONTROL, KS0108_EN);
  GLCD_Delay();
  GPIO_ResetBits(KS0108_PORT_CONTROL, KS0108_EN);
  GLCD_Delay();
  GLCD_DisableController(screen_x / 64);
  screen_x++;
}
//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
void GLCD_InitalizePorts(void)
{
	GPIO_InitTypeDef            GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin   =  GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  
  GPIO_InitStructure.GPIO_Pin   =  KS0108_RS|KS0108_RW|KS0108_EN|KS0108_CS1|KS0108_CS2|KS0108_CS3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
  GPIO_Init(KS0108_PORT_CONTROL, &GPIO_InitStructure);
  
  GPIO_Write(KS0108_PORT_DATA, (0xFF<<KS0108_D0));
  GPIO_Write(KS0108_PORT_CONTROL, KS0108_CS1 | KS0108_CS2 | KS0108_CS3 | KS0108_RS | KS0108_RW);
}
//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
unsigned char GLCD_ReadByteFromROMMemory(char * ptr)
{
  return *(ptr);
}

*/
///////////////CONFIGUARATION//////////////////////////////

void RCC_Setup(void);
void Motor_Init(void);

void TCRT5000(void);
void XuLyTCRT5000(void);
long MAP(long value, long in_min, long in_max, long out_min, long out_max);
void rfid_read(void);
uint8_t CompareID(char* CardID, char* CompareIDs);
void USART_Puts(USART_TypeDef* USARTx, char* str);
void Usart_Cofig(void);
void xu_ly_json(char* datajson);
void TIMER13_Init(void);
void delay(uint16_t timedl);
void delay_us(uint16_t timedlus);
void HCSR04_Init(void);
uint16_t HCSR_GetDistance(void);


//1:huong trai, 0:tai tram xuat phat 	2: huong phai
uint8_t kq = 0;
//0:AUTO	1:MANUAL
uint8_t mode = 0;
uint8_t stop = 0;
 uint8_t stopW = 0;
uint8_t Power = 0;

char hientai[10] = "22061B34";
char batdau[10] = "B3B595AC";
char ketthuc[10] = "838F94AC";
long bd_kt = 78;
int huong = 1;  //     0: Ben trai vi tri xuat phat   1: Tai vi tri xuat phat   2: Ben phai vi tri xuat phat
TM_SERVO_t Servo[5];
int main(){
	uint32_t i = 0;
	GPIO_InitTypeDef gpio;
	
	
	RCC_Setup();

	TM_SERVO_Init(&Servo[0], TIM4, TM_PWM_Channel_1, TM_PWM_PinsPack_1);
	TM_SERVO_Init(&Servo[1], TIM4, TM_PWM_Channel_2, TM_PWM_PinsPack_1);
	TM_SERVO_Init(&Servo[2], TIM4, TM_PWM_Channel_3, TM_PWM_PinsPack_1);
	TM_SERVO_Init(&Servo[3], TIM4, TM_PWM_Channel_4, TM_PWM_PinsPack_1);
	TM_SERVO_Init(&Servo[4], TIM3, TM_PWM_Channel_2, TM_PWM_PinsPack_2);
	TCRT5000();

	TIMER13_Init();

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	
	gpio.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4;
	gpio.GPIO_Mode = GPIO_Mode_IN;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOE, &gpio);

	HCSR04_Init();
	
	MF522_init();
	//delay_init(168);
	MFRC522_Reset();
	MFRC522_AntennaOn();  
//    //MFRC522_AntennaOff(); 

	Usart_Cofig();
	
	Motor_Init();
	  
	//dc trai tim11
	GPIOB->ODR |= 1<<14;
	GPIOB->ODR &= ~(1<<13);
	//dc phai tim10
	GPIOB->ODR |= 1<<12;
	GPIOB->ODR &= ~(1<<11);
	TIM9->CCR1 = 0;
	TIM9->CCR2 = 0;

	
	TM_SERVO_SetDegrees(&Servo[0], 100);//1
	TM_SERVO_SetDegrees(&Servo[1], 120);//2
	TM_SERVO_SetDegrees(&Servo[2], 20);//3
	TM_SERVO_SetDegrees(&Servo[3], 10);//4
	TM_SERVO_SetDegrees(&Servo[4], 30);//5
	stopW = 1;
	stop = 1;
	while(1){
		if(ngat_usart==1){
			sprintf(minh_json, "%s", buff);
			xu_ly_json(minh_json);
			//USART_Puts(USART1, minh_json);
			for(i = 0; i < 200; i++) buff[i] = '\0';
		}
		
//		TM_SERVO_SetDegrees(&Servo[0], 100);//1
//		TM_SERVO_SetDegrees(&Servo[1], 120);//2
//		TM_SERVO_SetDegrees(&Servo[2], 20);//3
//		TM_SERVO_SetDegrees(&Servo[3], 10);//4
//		TM_SERVO_SetDegrees(&Servo[4], 30);//5

		//XuLyTCRT5000();
		
		if(stopW==0){
			dis = HCSR_GetDistance();
			if(dis<30){
				stop = 1;
				TIM9->CCR1 = 0;
				TIM9->CCR2 = 0;
			}else stop = 0;
		}
		if(stop==0&&stopW==0&&mode==0){
			XuLyTCRT5000();
			rfid_read();
			if(CompareID(buffer, batdau)== 0&&kq==0){
				TIM9->CCR1 = 0;
				TIM9->CCR2 = 0;
				for(i = 20; i <= 120; i++){
					TM_SERVO_SetDegrees(&Servo[2], i);//3 
					delay(20);
				}
				for(i = 100; i <= 180; i++){
					TM_SERVO_SetDegrees(&Servo[0], i);//3 
					delay(20);
				}
				for(i = 10; i <= 20; i++){
					TM_SERVO_SetDegrees(&Servo[3], i);//3 
					delay(20);
				}
				for(i = 110; i <= 160; i++){
					TM_SERVO_SetDegrees(&Servo[1], i);//3
					delay(20);
				}
				for(i = 30; i <= 130; i++){
					TM_SERVO_SetDegrees(&Servo[4], i);//5
					delay(20);
				}
				delay(500);
	//			for(i = 130; i >= 30; i--){
	//				TM_SERVO_SetDegrees(&Servo[4], i);//5
	//				delay(20);
	//			}
				for(i = 160; i >= 85; i--){
					TM_SERVO_SetDegrees(&Servo[1], i);//3
					delay(20);
				}
				for(i = 20; i >= 10; i--){
					TM_SERVO_SetDegrees(&Servo[3], i);//3 
					delay(20);
				}
				for(i = 180; i >= 100; i--){
					TM_SERVO_SetDegrees(&Servo[0], i);//3 
					delay(20);
				}
				for(i = 120; i >= 20; i--){
					TM_SERVO_SetDegrees(&Servo[2], i);//3 
					delay(20);
				}
				delay(500);
				kq=1;
			}
			else if(CompareID(buffer, ketthuc)== 0&&kq==1){
				TIM9->CCR1 = 0;
				TIM9->CCR2 = 0;
				for(i = 20; i <= 120; i++){
					TM_SERVO_SetDegrees(&Servo[2], i);//3 
					delay(20);
				}
				for(i = 100; i > 0; i--){
					TM_SERVO_SetDegrees(&Servo[0], i);//3 
					delay(20);
				}
				for(i = 10; i > 0; i--){
					TM_SERVO_SetDegrees(&Servo[3], i);//3 
					delay(20);
				}
				for(i = 85; i <= 160; i++){
					TM_SERVO_SetDegrees(&Servo[1], i);//3
					delay(20);
				}
				for(i = 130; i >= 30; i--){
					TM_SERVO_SetDegrees(&Servo[4], i);//5
					delay(20);
				}
				delay(500);

				for(i = 160; i >= 110; i--){
					TM_SERVO_SetDegrees(&Servo[1], i);//3
					delay(20);
				}
				for(i = 0; i <= 10; i++){
					TM_SERVO_SetDegrees(&Servo[3], i);//3 
					delay(20);
				}
				for(i = 0; i <= 100; i++){
					TM_SERVO_SetDegrees(&Servo[0], i);//3 
					delay(20);
				}
				for(i = 120; i >= 20; i--){
					TM_SERVO_SetDegrees(&Servo[2], i);//3 
					delay(20);
				}
				delay(500);
				kq=0;
			}
			for(i = 0; i<15;i++) buffer[i]='\0';
		}
	}
}


//Start
long MAP(long value, long in_min, long in_max, long out_min, long out_max){
	long kq = 0;
  kq = ((value * (out_min + out_max)) / (in_min + in_max));
  if (kq < out_min) {
    kq = out_min;
  }
  else if (kq > out_max) {
    kq = out_max;
  }
  return kq;
}
void TCRT5000(void) {
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4| GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}
void XuLyTCRT5000(void){
	//Doc trang thai mat
	tcrt_arr[0] = GPIOA->IDR&(1<<3);
	tcrt_arr[1] = GPIOA->IDR&(1<<4);
	tcrt_arr[2] = GPIOA->IDR&(1<<5);
	tcrt_arr[3] = GPIOA->IDR&(1<<6);
	tcrt_arr[4] = GPIOA->IDR&(1<<7);
	
	if(tcrt_arr[0]==8&&tcrt_arr[1]==16&&tcrt_arr[2]==32&&tcrt_arr[3]==64&&tcrt_arr[4]==128){
//		//			//thang
//		GPIOB->ODR |= 1<<14;
//		GPIOB->ODR &= ~(1<<13);
//		GPIOB->ODR |= 1<<12;
//		GPIOB->ODR &= ~(1<<11);
//		TIM9->CCR1 = 100;
//		TIM9->CCR2 = 100;
//		delay(900);
////		//			//xoay trai
////		GPIOB->ODR |= 1<<13;
////		GPIOB->ODR &= ~(1<<14);
////		GPIOB->ODR |= 1<<12;
////		GPIOB->ODR &= ~(1<<11);
////		TIM10->CCR1 = 100;
////		TIM11->CCR1 = 100;
//		//			//xoay phai
//		GPIOB->ODR |= 1<<14;
//		GPIOB->ODR &= ~(1<<13);
//		GPIOB->ODR |= 1<<11;
//		GPIOB->ODR &= ~(1<<12);
//		TIM9->CCR1 = 100;
//		TIM9->CCR2 = 100;
//		delay(1100);
//		GPIOB->ODR |= 1<<14;
//		GPIOB->ODR &= ~(1<<13);
//		GPIOB->ODR |= 1<<12;
//		GPIOB->ODR &= ~(1<<11);
////		TIM9->CCR1 = 0;
////		TIM9->CCR2 = 0;
	}
	else{
		if(tcrt_arr[0]==8&&tcrt_arr[1]==0&&tcrt_arr[2]==0&&tcrt_arr[3]==0&&tcrt_arr[4]==0){
			Actual = 0;
		}
		else if((tcrt_arr[0]==0&&tcrt_arr[1]==16&&tcrt_arr[2]==0&&tcrt_arr[3]==0&&tcrt_arr[4]==0) || (tcrt_arr[0]==0&&tcrt_arr[1]==16&&tcrt_arr[2]==32&&tcrt_arr[3]==0&&tcrt_arr[4]==0)){
			Actual = 25;
		}
		else if(tcrt_arr[0]==0&&tcrt_arr[1]==0&&tcrt_arr[2]==32&&tcrt_arr[3]==0&&tcrt_arr[4]==0){
			Actual = 50;
		}
		else if((tcrt_arr[0]==0&&tcrt_arr[1]==0&&tcrt_arr[2]==0&&tcrt_arr[3]==64&&tcrt_arr[4]==0) || (tcrt_arr[0]==0&&tcrt_arr[1]==0&&tcrt_arr[2]==32&&tcrt_arr[3]==64&&tcrt_arr[4]==0)){
			Actual = 75;
		}
		else if(tcrt_arr[0]==0&&tcrt_arr[1]==0&&tcrt_arr[2]==0&&tcrt_arr[3]==0&&tcrt_arr[4]==128){
			Actual = 100;
		}
		
		Error = Target - Actual;
		ErrorChange = Error - ErrorOld;
		ErrorSlope = ErrorChange;
		ErrorArea = ErrorArea + Error;
		PWM_Value = kp * Error + ki * ErrorArea + kd * ErrorSlope;
		ErrorOld = Error;
		//if (PWM_Value >= 1000 || PWM_Value <= -1000) PWM_Value = PWM_Value_old;
		left = (kp*Target) + PWM_Value;
		right = (kp*Target) - PWM_Value;
		left = MAP(left, 0, (kp*Target), 0, max_speed);
		right = MAP(right, 0, (kp*Target), 0, max_speed);
		TIM9->CCR1 = right;
		TIM9->CCR2 = left;
//		PWM_Value_old = PWM_Value;
	}
}



void rfid_read(void){
	uint8_t i =0;
	char rfid_info[50] = "";
	status = MFRC522_Request(PICC_REQALL, g_ucTempbuf);
	if (status != MI_OK)
	{  
		flag_loop=0;  
		return;
	}
	status = MFRC522_Anticoll(g_ucTempbuf);
	if (status != MI_OK)
 {  
	 flag_loop=0;  
	 return;
 }
	if(flag_loop==1) 
	{
	 MFRC522_Halt();	
	 return;
	}
	flag_loop=1;
	sprintf(buffer, "%02X%02X%02X%02X",	g_ucTempbuf[0],g_ucTempbuf[1],g_ucTempbuf[2],g_ucTempbuf[3]);
	strcat(rfid_info, "{\"ma_rfid\":\"");
	strcat(rfid_info, buffer);
	
	strcat(rfid_info,"\"}\n");
	USART_Puts(USART3, rfid_info);
	for(i = 0; i < 50 ; i++){
				rfid_info[i] = 0;
	}
	//printf("\n UID=%x:%x:%x:%x\r\n",g_ucTempbuf[0],g_ucTempbuf[1],g_ucTempbuf[2],g_ucTempbuf[3] );
	MFRC522_Halt();
}
void USART_Puts(USART_TypeDef* USARTx, char* str) {

    /* Go through entire string */
    while (*str) {
        /* Wait to be ready, buffer empty */
        while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET){}
        /* Send data */
        USARTx->DR = (uint16_t)(*str++ & 0x01FF);
        /* Wait to be ready, buffer empty */
        while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET){}
    }
}
void USART3_IRQHandler(void){
	if(USART_GetITStatus(USART3, USART_IT_RXNE)){
    receive = USART_ReceiveData(USART3);
    USART_ClearITPendingBit(USART3,USART_IT_RXNE);
		if(receive != '\n'){
			//du lieu nhan tu esp32
			buff[buff_index++] = receive;
		}
		else{
			buff_index = 0;
			ngat_usart = 1;
		}
  }
}
void Usart_Cofig(void){
	GPIO_InitTypeDef gpio;
	USART_InitTypeDef uart;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC, &gpio);
	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);
	/*
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &gpio);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
	*/
	uart.USART_BaudRate = 115200;
	uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	uart.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
	uart.USART_Parity = USART_Parity_No;
	uart.USART_StopBits = USART_StopBits_1;
	uart.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART3, &uart);
	
	USART_Cmd(USART3, ENABLE);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	NVIC_EnableIRQ(USART3_IRQn);
}
void delay(uint16_t timedl){
	while(timedl){
		TIM_SetCounter(TIM13, 0);
		while(TIM_GetCounter(TIM13) < 1000){}
		timedl--;
	}
}
void delay_us(uint16_t timedlus){
	while(timedlus){
		TIM_SetCounter(TIM13, 0);
		while(TIM_GetCounter(TIM13) < 1){}
		timedlus--;
	}
}
void TIMER13_Init(void){
	TIM_TimeBaseInitTypeDef tim;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);
	
	tim.TIM_ClockDivision = TIM_CKD_DIV1;
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_Period = 10000 - 1;
	tim.TIM_Prescaler = 84 - 1;
	tim.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM13, &tim);
	TIM_Cmd(TIM13, ENABLE);
}


uint8_t CompareID(char* CardID, char* CompareIDs) {
	uint8_t i;
	for (i = 0; i < 5; i++) {
		if (CardID[i] != CompareIDs[i]) 
			{
			return 1;
		  }
	}
	return 0;
}

int so = 0;
void xu_ly_json(char* dataJson){
	char keyJson[10] = "";
	char valueJson[10] = "";
	char go[10] = "";
	char bac[10]= {0};
	int i = 0;
	quan_ly_data = cJSON_Parse(dataJson);
	
  if(!quan_ly_data){
//    printf("Json ERROR!!!!!\n");
    return;
  }
  else{
//		so++;
		if(cJSON_HasObjectItem(quan_ly_data, "DiChuyen")==1){
			sprintf(go, "%s", cJSON_GetObjectItem(quan_ly_data,"DiChuyen")->valuestring);
			if(strcmp(go, "Tien")==0){
				//		//			//thang
				GPIOB->ODR |= 1<<14;
				GPIOB->ODR &= ~(1<<13);
				GPIOB->ODR |= 1<<12;
				GPIOB->ODR &= ~(1<<11);
				TIM9->CCR1 = 100;
				TIM9->CCR2 = 100;
				stopW = 0;
			}
			else if(strcmp(go, "Lui")==0){
				//			//lui
				GPIOB->ODR |= 1<<13;
				GPIOB->ODR &= ~(1<<14);
				GPIOB->ODR |= 1<<11;
				GPIOB->ODR &= ~(1<<12);
				TIM9->CCR1 = 100;
				TIM9->CCR2 = 100;
				stopW = 0;
			}
			else if(strcmp(go, "Trai")==0){
				//			//xoay trai
				GPIOB->ODR |= 1<<13;
				GPIOB->ODR &= ~(1<<14);
				GPIOB->ODR |= 1<<12;
				GPIOB->ODR &= ~(1<<11);
				TIM9->CCR1 = 100;
				TIM9->CCR2 = 100;
				stopW = 0;
			}
			else if(strcmp(go, "Phai")==0){
				//			//xoay phai
				GPIOB->ODR |= 1<<14;
				GPIOB->ODR &= ~(1<<13);
				GPIOB->ODR |= 1<<11;
				GPIOB->ODR &= ~(1<<12);
				TIM9->CCR1 = 100;
				TIM9->CCR2 = 100;
				stopW = 0;
			}
			else if(strcmp(go, "S")==0){
				GPIOB->ODR |= 1<<14;
				GPIOB->ODR &= ~(1<<13);
				GPIOB->ODR |= 1<<12;
				GPIOB->ODR &= ~(1<<11);
				TIM9->CCR1 = 0;
				TIM9->CCR2 = 0;
				stopW = 1;
			}
			else if(strcmp(go, "F")==0){
				GPIOB->ODR |= 1<<14;
				GPIOB->ODR &= ~(1<<13);
				GPIOB->ODR |= 1<<12;
				GPIOB->ODR &= ~(1<<11);
				stopW = 0;
				stop = 0;
			}
		}
		else if(cJSON_HasObjectItem(quan_ly_data, "Robot")==1){
			so++;
			sprintf(bac, "%s", cJSON_GetObjectItem(quan_ly_data,"Robot")->valuestring);
			i = (bac[0]-'0');

			if(bac[1]=='+'){
				Robot[i-1] = Robot[i-1] + 5;
			}else{
				Robot[i-1] = Robot[i-1] - 5;
			}
			TM_SERVO_SetDegrees(&Servo[i-1], Robot[i-1]);
			
			
		}
		else if(cJSON_HasObjectItem(quan_ly_data, "Mode")==1){
			mode = cJSON_GetObjectItem(quan_ly_data,"Mode")->valueint;
		}
		else if(cJSON_HasObjectItem(quan_ly_data, "Start")==1){
			sprintf(keyJson, "%s", cJSON_GetObjectItem(quan_ly_data,"Start")->valuestring);
			sprintf(valueJson, "%s", cJSON_GetObjectItem(quan_ly_data,"End")->valuestring);
			while(1){
				if(strcmp("Tram 1", keyJson) == 0){
					sprintf(batdau, "%s", R7);
					break;
				}
				else if(strcmp("Tram 2", keyJson) == 0){
					sprintf(batdau, "%s", R8);
					break;
				}
				else if(strcmp("Tram 3", keyJson) == 0){
					sprintf(batdau, "%s", R9);
					break;
				}
				else if(strcmp("Tram 4", keyJson) == 0){
					sprintf(batdau, "%s", R10);
					break;
				}
			}

			while(1){
				if(strcmp("Tram 1", valueJson) == 0){
					sprintf(ketthuc, "%s", R7);
					break;
				}
				else if(strcmp("Tram 2", valueJson) == 0){
					sprintf(ketthuc, "%s", R8);
					break;
				}
				else if(strcmp("Tram 3", valueJson) == 0){
					sprintf(ketthuc, "%s", R9);
					break;
				}
				else if(strcmp("Tram 4", valueJson) == 0){
					sprintf(ketthuc, "%s", R10);
					break;
				}
			}
		}
		cJSON_Delete(quan_ly_data);
		for(i = 0; i < 200; i++){
				buff[i] = '\0';
			minh_json[i]= '\0';
			}
			ngat_usart = 0;
  }

}


void RCC_Setup(void){
	ErrorStatus HSEStartUpStatus;
	
	/* RCC system reset(for debug purpose) */
	RCC_DeInit();
	
	 /* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON);
	
	/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();
	
	while(HSEStartUpStatus != SUCCESS){
		HSEStartUpStatus = RCC_WaitForHSEStartUp();
	}
	
	if(HSEStartUpStatus == SUCCESS){
		/* Enable Prefetch Buffer */
		FLASH_PrefetchBufferCmd(ENABLE);
		
		/* Flash 5 wait state */
		FLASH_SetLatency(FLASH_Latency_5);
		
		/* HCLK = SYSCLK */
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		
		/*PCLK2 = HCLK/2*/
		RCC_PCLK2Config(RCC_HCLK_Div2);
		
		/*PCLK1 = HCLK/4*/
		RCC_PCLK1Config(RCC_HCLK_Div4);
		
		/*PLLCLK = 168MHz*/
		RCC_PLLConfig(RCC_PLLSource_HSE, 4, 168, 2, 4);
		
		/*Enable PLL*/
		RCC_PLLCmd(ENABLE);
		
		/* Wait till PLL is ready */
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){}
		
		/* Select PLL as system clock source */			
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		
		/* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08) {}
	}
	
}

void Motor_Init(void){
	uint16_t PrescalerValue = (uint16_t) 16800-1;
	GPIO_InitTypeDef            GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;
	TIM_OCInitTypeDef           TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType   = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_UP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	
	

	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_TIM9);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIOB->ODR |= 1<<14;
	GPIOB->ODR &= ~(1<<13);
	GPIOB->ODR |= 1<<12;
	GPIOB->ODR &= ~(1<<11);
	/*
	* 84M / TIM_Prescaler = 1M
	* 1M / TIM_ClockDivision = 1M
	* 1M / (TIM_Period + 1) = 50Hz (20ms)
	* TIM_Pulse = 500(us) ~ 2500(us)
	*/


	TIM_TimeBaseStructure.TIM_Period        = 1000-1;
	TIM_TimeBaseStructure.TIM_Prescaler     = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//0;
	TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse       = 0; 
	TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;

	TIM_OC1Init(TIM9, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);

	TIM_OC2Init(TIM9, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM9, ENABLE);

	TIM_Cmd(TIM9, ENABLE);
//	TIM_CtrlPWMOutputs(TIM8, ENABLE);
}


uint16_t HCSR_GetDistance(){
	uint16_t distance;
	TIM2->CNT = 0;
	TIM_Cmd(TIM2, ENABLE);
	while(!TIM_GetFlagStatus(TIM2, TIM_FLAG_Update)){}
	TIM_Cmd(TIM2, DISABLE);
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	distance = (TIM_GetCapture2(TIM2) - TIM_GetCapture1(TIM2))*0.0171;
	return distance; 
}
void HCSR04_Init(void){
	TIM_TimeBaseInitTypeDef hcsr04Tim;
	TIM_OCInitTypeDef hcsr04OC;
	TIM_ICInitTypeDef hcsr04IC;
	GPIO_InitTypeDef hcsr04GPIO;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	//Triger - PA2 + Echo - PA0
	hcsr04GPIO.GPIO_Mode = GPIO_Mode_AF;
	hcsr04GPIO.GPIO_OType = GPIO_OType_PP;
	hcsr04GPIO.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_2;
	hcsr04GPIO.GPIO_PuPd = GPIO_PuPd_NOPULL;
	hcsr04GPIO.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOA, &hcsr04GPIO);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);
	
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	//TIM2
	TIM_DeInit(TIM2);
	
	hcsr04Tim.TIM_Prescaler = 84 - 1;
	hcsr04Tim.TIM_Period = 0xFFFF;
	hcsr04Tim.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM2, &hcsr04Tim);
	
	hcsr04OC.TIM_OCMode = TIM_OCMode_PWM1;
	hcsr04OC.TIM_OCPolarity = TIM_OCPolarity_High;
	hcsr04OC.TIM_Pulse = 15; //us
	hcsr04OC.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC3Init(TIM2, &hcsr04OC);
	
	
	
	hcsr04IC.TIM_Channel = TIM_Channel_1;
	hcsr04IC.TIM_ICPolarity = TIM_ICPolarity_Rising;
	hcsr04IC.TIM_ICSelection = TIM_ICSelection_DirectTI;
	hcsr04IC.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	hcsr04IC.TIM_ICFilter = 0;
	TIM_PWMIConfig(TIM2, &hcsr04IC);
	
	TIM_SelectInputTrigger(TIM2, TIM_TS_TI1FP1);
	TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
}

//End








