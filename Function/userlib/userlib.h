/*
Lib viet ham delay va PWM tu tao
Lib khong su dung Timer, lib co viet su dung ham while neu muon su dung
Lib co su dung PWM nhung co the khong chinh xac
Su dung DHT, bat Timer
*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include <string.h>
#ifndef __DEFINE_MODE_H_
#define __DEFINE_MODE_H_

#define OUTPUT_PP 			GPIO_MODE_OUTPUT_PP
#define INPUT 					GPIO_MODE_INPUT
#define OUTPUT_OD				GPIO_MODE_OUTPUT_OD

#endif

#ifndef __DEFINE_CONFIG_H_ 
#define __DEFINE_CONFIG_H_

#define PA					GPIOA
#define PB					GPIOB
#define PC					GPIOC

#define FREQ_LOW			GPIO_SPEED_FREQ_LOW
#define FREQ_MEDIUM		GPIO_SPEED_FREQ_MEDIUM
#define FREQ_HIGH			GPIO_SPEED_FREQ_HIGH

#define SET						GPIO_PIN_SET
#define RESET					GPIO_PIN_RESET

#define PULLUP			GPIO_PULLUP
#define PULLDOWN 		GPIO_PULLDOWN
#define NOPULL			GPIO_NOPULL

#endif

#ifndef __DEFINE_PORT_H_
#define __DEFINE_PORT_H_

#define PWM_PORT GPIOA
#define PWM_PIN GPIO_PIN_3
#endif

//----
#ifndef __PWM_H_
#define __PWM_H_

/*====================================
CONFIGURATION OUT/IN  Funct
======================================*/

/*****************************************************************************
**						GPIOx: PA/PB/PC/PD																						**
**						GPIO_PIN: GPIO_PIN_0-15																				**
**						mode: INPUT(Floating)/OUTPUT_PP/OUTPUT_OD											**
**						config: PULLUP/ PULLDOWN/ NOPULL															**
**						status: SET/RESET																							**
**						freq: FREQ_LOW/FREQ_HIGH/FREQ_MEDIUM													**
*****************************************************************************/
void GPIO_Output_Config(GPIO_TypeDef  *GPIOx ,uint16_t GPIO_PIN, uint8_t mode, uint8_t config, uint8_t status, uint8_t freq);//Cau hinh Output


/*====================================
PWM & DELAY Funct
======================================*/
void delay(uint32_t time); //ham delay su dung vong while
void PWM_Config(void);
void PWM_SetDutyCycle(uint16_t period, uint16_t duty_cycle);
#endif

/*=================================================
DELAY USE TIMER Funct
==================================================*/

#ifndef __DELAY_TIMER_H
#define __DELAY_TIMER_H
#include "stm32f1xx_hal.h"
void DELAY_TIM_Init(TIM_HandleTypeDef *htim);
void DELAY_TIM_Us(TIM_HandleTypeDef *htim, uint16_t time);
void DELAY_TIM_Ms(TIM_HandleTypeDef *htim, uint16_t Time);
#endif

/*====================================
DHT11 Funct
======================================*/
#ifndef __DHT11_H_
#define DHT11_STARTTIME 18000
#define DHT22_STARTTIME 12000
#define DHT11 0x01
#define DHT22 0x02
#define DHT_Port GPIOA
#define DHT_Pin GPIO_PIN_1
typedef struct
{	
	uint16_t Type;
	TIM_HandleTypeDef* Timer;
	uint16_t Pin;
	GPIO_TypeDef* PORT;
	float Temp;
	float Humi;
}DHT_Name;

void DHT_Init(DHT_Name* DHT, uint8_t DHT_Type, TIM_HandleTypeDef* Timer, GPIO_TypeDef* DH_PORT, uint16_t DH_Pin);
uint16_t DHT_ReadTempHum(DHT_Name* DHT);
#endif

/*====================================
LCD I2C Funct
======================================*/
#ifndef __I2C_H_
#define __I2C_H_
void lcd_init (void);   // initialize lcd

void lcd_send_cmd (char cmd);  // send command to the lcd

void lcd_send_data (char data);  // send data to the lcd

void lcd_send_string (char *str);  // send string to the lcd

void lcd_put_cur(int row, int col);  // put cursor at the entered position row (0 or 1), col (0-15);

void lcd_clear (void);
#endif



#ifndef UARTRINGBUFFER_H_
#define UARTRINGBUFFER_H_

/* change the size of the buffer */
#define UART_BUFFER_SIZE 64

typedef struct
{
  unsigned char buffer[UART_BUFFER_SIZE];
  volatile unsigned int head;
  volatile unsigned int tail;
} ring_buffer;


/* reads the data in the rx_buffer and increment the tail count in rx_buffer of the given UART */
int Uart_read(UART_HandleTypeDef *uart);

/* writes the data to the tx_buffer and increment the head count in tx_buffer */
void Uart_write(int c, UART_HandleTypeDef *uart);

/* function to send the string to the uart */
void Uart_sendstring(const char *s, UART_HandleTypeDef *uart);

/* Print a number with any base
 * base can be 10, 8 etc*/
void Uart_printbase (long n, uint8_t base, UART_HandleTypeDef *uart);

/* Initialize the ring buffer */
void Ringbuf_init(void);

void Uart_flush (UART_HandleTypeDef *uart);

/* checks if the data is available to read in the rx_buffer of the uart */
int IsDataAvailable(UART_HandleTypeDef *uart);

/* Look for a particular string in the given buffer
 * @return 1, if the string is found and -1 if not found
 * @USAGE:: if (Look_for ("some string", buffer)) do something
 */
int Look_for (char *str, char *buffertolookinto);

/* Peek for the data in the Rx Bffer without incrementing the tail count
* Returns the character
* USAGE: if (Uart_peek () == 'M') do something
*/
int Uart_peek(UART_HandleTypeDef *uart);


/* Copy the data from the Rx buffer into the buffer, Upto and including the entered string
* This copying will take place in the blocking mode, so you won't be able to perform any other operations
* Returns 1 on success and -1 otherwise
* USAGE: while (!(Copy_Upto ("some string", buffer, uart)));
*/
int Copy_upto (char *string, char *buffertocopyinto, UART_HandleTypeDef *uart);


/* Copies the entered number of characters (blocking mode) from the Rx buffer into the buffer, after some particular string is detected
* Returns 1 on success and -1 otherwise
* USAGE: while (!(Get_after ("some string", 6, buffer, uart)));
*/
int Get_after (char *string, uint8_t numberofchars, char *buffertosave, UART_HandleTypeDef *uart);

/* Wait until a paricular string is detected in the Rx Buffer
* Return 1 on success and -1 otherwise
* USAGE: while (!(Wait_for("some string", uart)));
*/
int Wait_for (char *string, UART_HandleTypeDef *uart);

/* the ISR for the uart. put it in the IRQ handler */
void Uart_isr (UART_HandleTypeDef *huart);

/*** Depreciated For now. This is not needed, try using other functions to meet the requirement ***/
/* get the position of the given string within the incoming data.
 * It returns the position, where the string ends
 */
/* get the position of the given string in the given UART's incoming data.
 * It returns the position, where the string ends */
//int16_t Get_position (char *string, UART_HandleTypeDef *uart);


#endif /* UARTRINGBUFFER_H_ */

/*====================================
STM32-ESP8266 CONNECTION Funct
======================================*/
#ifndef INC_ESP8266_HAL_H_
#define INC_ESP8266_HAL_H_


void ESP_Init (char *SSID, char *PASSWD);

void Server_Start (void);


#endif /* INC_ESP8266_HAL_H_ */
