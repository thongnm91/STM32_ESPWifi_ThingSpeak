/*LIB*/
#include "stm32l1xx.h"
#define HSI_VALUE    ((uint32_t)16000000)
#include "nucleo152start.h"
#include <stdio.h>
#include <string.h>

/*---END */

/*Prototype*/
void delay_ms(unsigned long delay);
void delay_10us(unsigned long delay);

void USART2_write(char data);
void USART2_Init(void);
void USART2_write_string(char* data);

int am2302Request(uint8_t *array);
int am2302ShowUart2(uint8_t* am2302) ;
int AM2302_Read_Data(float* humidity, float* temperature, uint8_t* data);

char USART1_read();
void USART1_read_string(char* data);
void USART1_write_string(char* data);
void USART1_write(char data);
void USART1_Init(void);

void wifi_init();

/*Global variable*/
char buf[100]="";
/*--- END */

int main(void)
{
	uint8_t am2302[4];

	/* Configure the system clock to 32 MHz and update SystemCoreClock */
	SetSysClock();
	SystemCoreClockUpdate();
	USART2_Init();

	USART1_Init();

	RCC->AHBENR|=RCC_AHBENR_GPIOAEN;
	GPIOA->MODER|=GPIO_MODER_MODER5_0 | GPIO_MODER_MODER7_0;

	wifi_init();

  while (1)
  {

	  if(am2302Request(am2302)){
		  am2302ShowUart2(am2302);
	  }

	  GPIOA->ODR|=GPIO_ODR_ODR_5|GPIO_ODR_ODR_7;
	  delay_ms(200);
	  GPIOA->ODR&=~(GPIO_ODR_ODR_5|GPIO_ODR_ODR_7);
	  delay_ms(200);

	  delay_ms(5000);


  }
  return 0;
}

/*Sub Function*/
void delay_ms(unsigned long delay)
{
	unsigned long i=0;
	RCC->APB1ENR|= RCC_APB1ENR_TIM2EN; 	//TIM2EN: Timer 2 clock enable. p160
	TIM2->PSC=32-1; 		//32 000 000 MHz / 32 = 1 000 000 Hz. p435
	TIM2->ARR=1000-1; 		//TIM2 counter. 1 000 000 Hz / 1000 = 1000 Hz ~ 1ms. p435
	TIM2->CNT=0;			//counter start value = 0
	TIM2->CR1=1; 			//TIM2 Counter enabled. p421

	  while(i<delay)
	  {
		  while(!((TIM2->SR)&1)){} //Update interrupt flag. p427
		  i++;
		  TIM2->SR &= ~1; 	//flag cleared. p427
		  TIM2->CNT=0;	  	//counter start value = 0
	  }
	  TIM2->CR1=0; 		//TIM2 Counter disabled. p421
}

void delay_10us(unsigned long delay)
{
	unsigned long i=0;
	RCC->APB1ENR|= RCC_APB1ENR_TIM6EN; 	//TIM6EN: Timer 6 clock enable. p160
	TIM6->PSC=16-1; 		//32 000 000 MHz / 16 = 2 000 000 Hz. p435
	TIM6->ARR=13-1; 		//TIM6 counter. 2 000 000 Hz / 20 = 100 000 Hz ~ 10us. p435
	TIM6->CNT=0;			//counter start value = 0
	TIM6->CR1=1; 			//TIM2 Counter enabled. p421

	  while(i<delay)
	  {
		  while(!((TIM6->SR)&1)){} //Update interrupt flag. p427
		  i++;
		  TIM6->SR &= ~1; 	//flag cleared. p427
		  TIM6->CNT=0;	  	//counter start value = 0
	  }
	  TIM6->CR1=0; 		//TIM2 Counter disabled. p421
}

void USART2_Init(void)
{
	RCC->APB1ENR|=0x00020000;
	RCC->AHBENR|=0x00000001;
	GPIOA->AFR[0]=0x00000700; //RX
	GPIOA->MODER|=0x00000020;

	USART2->BRR = 0x00000D05;	// 9600
	USART2->CR1 = 0x00000008;	//nable transmit
	USART2->CR1 |= 0x00002000;	//Uart enable
}

void USART2_write(char data)
{
	//wait while TX buffer is empty
	while(!(USART2->SR&0x0080)){} 	//TXE: Transmit data register empty. p736-737
		USART2->DR=(data);			//p739
}

void USART2_write_string(char* data)
{
	int i=0;
	while(data[i]!='\0'){
		USART2_write(data[i]);
		i++;
	}
	USART2_write('\n');
	USART2_write('\r');
}

int am2302Request(uint8_t *array){
//1. Initial GPIO mode Output
	//1. Enable Clock bus for GPIO pin PB3 (D3)
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	//2. Config GPIO pin mode output
	GPIOB->MODER |= GPIO_MODER_MODER3_0;
//2. Start communication
	 //1. Set Pin 6 HIGH
	GPIOB->ODR |= GPIO_ODR_ODR_3;
	//	2. Delay 1 ms
	delay_ms(1);

	 //3. Set pin LOW
	GPIOB->ODR &= ~GPIO_ODR_ODR_3;
	//4. Delay 1 ms
	delay_ms(1);
//3. Request data
	 //1. Set pin HIGH
	GPIOB->ODR |= GPIO_ODR_ODR_3;
	 //2. Delay 40 us
	delay_10us(5);
//4. Change GPIO mode Input
	GPIOB->MODER &= ~GPIO_MODER_MODER3;
//5. Wait for am2302 respond
	 //1. Wait forever until Input Data Register Pin LOW
	while((GPIOB->IDR & GPIO_IDR_IDR_3)){}
	//2. Wait forever until Input Data Register Pin HIGH
	while(!(GPIOB->IDR & GPIO_IDR_IDR_3)){}
	 //3. Wait forever until Input Data Register Pin LOW
	while((GPIOB->IDR & GPIO_IDR_IDR_3)){}
//6. Get data
	//1. User provide pointer array uint8 array[5] to save data.
	uint8_t ar[5]={0x00};
	//2. Read 40 bits and add each 8 bits to array
	uint8_t i = 0;
	while(i<40){
		//2.1 Wait until Input Data Register Pin HIGH
		while(!(GPIOB->IDR & GPIO_IDR_IDR_3)){}
		//2.2 Delay 50 us
		delay_10us(4);
		//2.3 If PIN still HIGH over 45us, the vol-lengh mean data "1" else mean "0".
		//Then add each 8bits to array.
		if(GPIOB->IDR & GPIO_IDR_IDR_3){
			ar[i/8] |= 0x80 >> (i%8);//(i/8): i increase every 8bit, (i%8): reset counting from 0->7
		}
		//2.4 i++;
		i++;
		//2.5 wait pin change HIGH to LOW to finish this bit
		while(GPIOB->IDR & GPIO_IDR_IDR_3){}
	}
//7. Calculate CRC
	//Checksum=(Byte 0 + Byte 1 + Byte 2 + Byte 3) &0xFF
	uint8_t crc = 0;
	for(i=0;i<4;i++){
		crc += ar[i];
	}
	crc &= 0xFF;
//8. Check CRC if correct then return data
	if(crc==ar[4]){
		for(i=0;i<4;i++){
			array[i]=ar[i];
		}
		return 1;
	}
	else
		return 0;
}


/*
 *
 * Function format and send data to UART2
 * 1. Read humidity
 * 		1. Combine 8 bits high and 8 bit low to 16 bit humidity
 * 2. Check minus template
 * 		1. read bit[7] if 1 mean negative temperature else mean positive
 * 3. Read temperature
 * 		1. Remove bit[7]
 * 		2. Combine 8 bits high and 8 bit low to 16 bit temperature
 * 4. Write UART2
 * 		1. Format data and save in an array
 * 		2. Uart2 write string
 * */
int  am2302ShowUart2(uint8_t* am2302) {
	uint16_t h = 0x0000, t = 0x0000;
	char minus = '0';
	char buffer[20]={'\0'};

  //Read humidity
  h = ((am2302[0]&0xFFFF)<<8) | (am2302[1]&0xFFFF);

  //Check minus template
  minus = (am2302[2]>>7)? '-' : '+';
  //Read temperature
  t = ((am2302[2]&0xFF7F)<<8) | (am2302[3]&0xFFFF); //&0xFF7F to remove minus

  //Write UART2
  sprintf(buffer, "%d,%d %%RH  %c%d,%d °C\n\r", h/10,h%10,minus,t/10,t%10);
  USART2_write_string(buffer);

  /*Wifi*/
  USART1_write_string("AT+CIPMUX=0\r\n");
  delay_ms(2000);
  USART1_write_string("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n");
  delay_ms(2000);
  USART1_write_string("AT+CIPSEND=70\r\n");
  delay_ms(2000);

  if(minus=='-'){
	  sprintf(buf,"GET /update?api_key=W75FJS0F0705E3DU&field1=%c%d&field2=%d\r\n",minus,t/10,h/10);
  }
  else
	  sprintf(buf,"GET /update?api_key=W75FJS0F0705E3DU&field1=%d&field2=%d\r\n",t/10,h/10);
  USART1_write_string(buf);
  delay_ms(10000);

  USART1_write_string("AT+CIPCLOSE\r\n");
  delay_ms(2000);

  return 0;
}
int AM2302_Read_Data(float* humidity, float* temperature, uint8_t* data){
	uint16_t h = 0x0000, t = 0x0000;
	//Read humidity
	  h = ((data[0]&0xFFFF)<<8) | (data[1]&0xFFFF);

	  //Check minus template
	  int minus = (data[2]>>7)? -1 : 1;
	  //Read temperature
	  t = ((data[2]&0xFF7F)<<8) | (data[3]&0xFFFF); //&0xFF7F to remove minus

	  *humidity = h/10.0;
	  *temperature = (t/10.0) * minus;

	  return 0;
}
void USART1_Init(void)
{
	RCC->APB2ENR|=RCC_APB2ENR_USART1EN;
	RCC->AHBENR|=RCC_AHBENR_GPIOAEN;
	GPIOA->AFR[1]|=7<<GPIO_AFRH_AFRH1_Pos; //PA9 TX
	GPIOA->AFR[1]|=7<<GPIO_AFRH_AFRH2_Pos;  //PA10 RX
	GPIOA->MODER|=GPIO_MODER_MODER9_1;
	GPIOA->MODER|=GPIO_MODER_MODER10_1;

	USART1->BRR = 0x00000116;	// 115200
	USART1->CR1 = 0x00000008;	//nable transmit
	USART1->CR1 |= 0x00000004;
	USART1->CR1 |= 0x00002000;	//Uart enable

}

void USART1_write(char data)
{
	//wait while TX buffer is empty
	while(!(USART1->SR&0x0080)){} 	//TXE: Transmit data register empty. p736-737
		USART1->DR=(data);		//p739
}

void USART1_write_string(char* data)
{
	int i=0;
	while(data[i]!='\0'){
		USART1_write(data[i]);
		i++;
	}
	USART1_write('\n');
	USART1_write('\r');
}
char USART1_read()
{
	char data=0;
	//wait while RX buffer is data is ready to be read
	while(!(USART1->SR&0x0020)){} 	//Bit 5 RXNE: Read data register not empty
		data=USART1->DR;			//p739
		return data;
}
void USART1_read_string(char* data){
	int i = 0;
	char a = 0;
	while(a!=data){
		a = USART1_read();
		buf[i]=a;
		i++;
	}
	delay_ms(1);
}
void wifi_init(){
	USART1_write_string("AT\r\n");
	delay_ms(500);
	USART1_write_string("AT\r\n");
	delay_ms(500);

	USART1_write_string("AT+RST\r\n");
	delay_ms(500);

	USART1_write_string("AT+GMR\r\n");
	delay_ms(500);

	USART1_write_string("AT+CWMODE=3\r\n");
	delay_ms(500);

	USART1_write_string("AT+CWLAP\r\n");
	delay_ms(500);

	USART1_write_string("AT+CWJAP=\"***\",\"***\"\r\n");
	delay_ms(3000);

	USART1_write_string("AT+CWJAP?\r\n");
	delay_ms(1000);
}
