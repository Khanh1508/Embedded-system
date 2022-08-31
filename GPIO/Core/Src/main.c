#include "main.h"
#include<stdint.h>    //library for uint32_t ( int type)
#include<string.h>   //Library for memcpy


void timer1_update_handler();
void uart2_handler();
void dma_handler();

uint16_t systick_cnt;
uint16_t cnt = 0;
uint32_t time_tick=0;

unsigned char flag=0;
char value=0;
int button_cnt=0;
char Rx_data;
char uart2_rx_buff[10];
char spi1_rx_buff[1028];

unsigned char spi_index=0;
unsigned char uart_rx_index=0;
char low;
char high;

uint8_t i2c_ss_id=0;
int16_t x_axis; // 16 bit for paring  8 bit high and low
int16_t y_axis;
int16_t z_axis;


/*Function  for LED LD4 in PD12*/
void gpio_d12_init()
{
	__HAL_RCC_GPIOD_CLK_ENABLE(); //Khoi tao xung cap clock
	uint32_t* GPIOD_MODER =0x40020c00;// tro toi dia chi Moder
	*GPIOD_MODER &= ~(0b11<<24);//dich bit 24 25 clear
	*GPIOD_MODER |= 0b01<<24;// or voi bit 01 for output mode
}

/*
 #define ON  1
 #define OFF 0
*/
typedef enum{
	OFF,
	ON
}led_state_t;

          /*PD12 Set mode */
void led_control(led_state_t value)/*REQUEST USER TRUYEN VAO 1 GIA TRI 0 OR 1*/
{
	uint32_t* GPIOD_ODR=0x40020c14;  // tro to thanh ghi OUTPUT DATA REGISTER
	if(value==ON)
		*GPIOD_ODR |= 1<<12; // dich trai 12 bit  den thanh ghi ORD12
	else
		*GPIOD_ODR &=~(1<<12);
}


		/* READ BUTTON Function*/
void gpio_button_init()
{
	  /*ENABLE CLOCK FOR GPIOA*/
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Set PA0 in floating input mode*/
	uint32_t* GPIOA_MODER=0x40020000;
	*GPIOA_MODER &= ~(0b11<<0);//clear -set PA0 as input
	uint32_t* GPIOA_PUPDR=0x4002000c;
	*GPIOA_PUPDR &= ~(0b11<<0);  //set floating

	/*Set EXTI for interrupt*/
	uint32_t* EXTI_RTSR=0x40013c08;
	*EXTI_RTSR|=(1<<0);        // Set rising mode up eaged
	uint32_t* IMR=0x40013c00; //set 1 de AND voi RTSR =1
	*IMR |=(1<<0);

	uint32_t* ISER0 =0xe000e100; //NVIC
	*ISER0 |= (1<<6); //Enable interrupt event at position 6
}

int gpio_button_read()
{
	uint32_t* GPIOA_IDR=0x40020010;
	if((*GPIOA_IDR>>0 & 1)==1)
		return 1;
	else
		return 0;
}

	/*timer initialization*/
void timer1_init()
{
		/*Cap clock*/
	__HAL_RCC_TIM1_CLK_ENABLE();
	uint32_t* ARR=0x4001002c;
	uint32_t* PSC=0x40010028;
	/*RCC-> 16MHZ  ->Prescaler (16000)-> 1KHZ(1ms) */

	*ARR = 1000-1;
	*PSC = 16000-1;

	/*Control register enable 1, set bit =1 -> timer start counts*/
	uint32_t* CR1=0x40010000;
	*CR1 |=1<<0;//SET bit CEN up 1

	/*Enable update interrupt event UIE*/
	uint32_t* DIER=0x4001000c;
	*DIER |=1<<0;

	uint32_t* ISER0 =0xe000e100; //NVIC
	*ISER0 |= (1<<25); //Enable interrupt event at position 25
}

       /* Use Default Function */
void TIM1_UP_TIM10_IRQHandler()
{
	__asm("NOP");
	button_cnt++;
	uint32_t* SR=0x40010010;
	*SR &=~1; // Clear ve 0
}


     /*Timer function custom subcried on vector table*/
void timer1_update_handler()
{
	static flag= 0;  /*bien static : ra khoi ham van ton tai( cuc bo tinh)*/
	flag= 1-flag;
	led_control(flag);
	__asm("NOP");
	uint32_t* SR=0x40010010;
	*SR &=~1;/*status register Clear*/
	time_tick++;
}




/*Declare fucntion for interrupt*/
void custom_exti0();
void my_delay(uint32_t sec)// ham delay unit second
{
	time_tick= 0;
	while(time_tick < sec);
}

         /*PWM function*/
void pwm_init()
{
	__HAL_RCC_GPIOD_CLK_ENABLE();
	uint32_t* MODER=0x40020c00;
	*MODER &=~(0b11<<24); //CLEAR BIT MODER12
	*MODER |=(0b10<<24); //SET PD12 IN ALTERNATE FUCNTION

	/*Chuyen quyen dieu khi GPIO sang ngoai vi timer4_ch1*/
	uint32_t* AFRH=0x40020c24;
	*AFRH &=~(0b1111<<16);
	*AFRH |=(0b0010<<16);

	__HAL_RCC_TIM4_CLK_ENABLE();
	uint32_t* ARR=0x4000082c;
	uint32_t* PSC=0x40000828;
	uint32_t* CCR1=0x40000834; // su dung CH1

	*ARR=1000-1;
	*PSC=16000-1;
	*CCR1=500-1;

	/*sET Capture/compare mode cho CH1  */
	uint32_t* CCMR1=0x40000818;
	*CCMR1 &=~(0b111<<4);
	*CCMR1 |=~(0b110<<4);

	uint32_t* CCER=0x40000820;
	*CCER |=1;

	uint32_t* CR1=0x40000800;
	*CR1 |=1;

}

void capture_init()
{                /*Set GPIO*/
	__HAL_RCC_GPIOE_CLK_ENABLE();
	uint32_t* MODER=(uint32_t*)0x40021000;
	uint32_t* AFRH=(uint32_t*)0x40021024; //SET PE9 as alternate function AFRH01

	*MODER &=~(0b11<<18);   //MODER9
    *MODER|=(0b10<<18);    //SET PE9 IN ALTERNATE FUCNTION
	*AFRH &=~(0b1111<<4); /*CHANGE GPIO to peripheral TIMER*/
	*AFRH |=(0b0001<<4);  //0001: AF1  AFRH9 vi tri 4-7

	        /*Set TIMER*/
	__HAL_RCC_TIM1_CLK_ENABLE();
	uint32_t* CR1=(uint32_t*)0x40010000;  //control register
	uint32_t* SMCR=(uint32_t*)0x40010008; //slave mode control register
	uint32_t* CCMR1_Input=(uint32_t*)0x40010018; //capture/compare mode register 1 (output mode)
	uint32_t* CCER=(uint32_t*)0x40010020;   //capture/compare enable
	uint32_t* ARR=(uint32_t*)0x4001002c;
	uint32_t* PSC=(uint32_t*)0x40010028;    //0x40000828;

	*ARR=0xffff; // 65535 , maximum value
	*PSC=1600-1;

	*CCMR1_Input &=~(0b11<<0);//Clear bit
	*CCMR1_Input |=(0b01<<0);// set capture as channel 1 map with TI1

	*SMCR &=~(0b111<<0);//Slave mode in RESET MODE
	*SMCR|=(0b100<<0); // set Reset mode

	*SMCR &=~(0b111<<4);
	*SMCR |=(0b101<<4);

	*CCER|=1;//Enable capture/compare channel 1

	*CR1|=1; //enable counter
}



       /*UART*/

void uart_init()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	uint32_t* MODER=(uint32_t*)0x40020000;
	*MODER &=~(0b1111<<4); //clear bitr
	*MODER|=(0b10<<4) | (0b10<<6);// Set UART TX PA2 -UART RX PA3

	uint32_t* AFRL=(uint32_t*)0x40020020;
	*AFRL &=~(0xff<<8);
    *AFRL |= 7 <<8; //AND 0111  //*AFRL |= (0b0111<<8);
    *AFRL |= 7 <<12;   //AFRL PA3

    /*
     * f for UART=16MHZ -> BAUDR=9600 ==> 104,1875
     * man=104
     * fran=0,1875=16 =3
     */
    __HAL_RCC_USART2_CLK_ENABLE();
     uint32_t* BRR=0x40004408;
    *BRR=(104<<4)|3;  // *BBR= 104<<4;  *BBR|=3<<0;

       uint32_t* CR1=0x4000440c;
//    *CR1|=(1<<5);//Enale interrupt UART
      *CR1|=(1<<3)|(1<<2) |(1<<13); //SET WORDLENGTH( MAC DINH) ,enable TX RX bit 2 3
      uint32_t* NVIC_ISER1=(uint32_t)0xe000e104;
//    *NVIC_ISER1 |=(1 <<(38-32));
      *NVIC_ISER1 |=(1 <<(47-32)); //POSITION 15 at NVIC
      uint32_t* CR3=0x40004414;
      *CR3|=(1<<6); // Bit 6 DMAR: DMA enable receiver -if has data , Generate a signal send to DMA


}
      /*Send data UART function*/
void uart_write_byte(char data)
{
	uint32_t* SR=(uint32_t*)0x40004400;
	uint8_t* DR=(uint8_t*)0x40004404;
	while(((*SR>>7)&1) !=1);//wait TXE duoc keo len 1  , NEU =0 thi wait ,cho den khi bang 1, dieu kien sai=> thi thoat khoi while
	*DR=data; //write du lieu vao bien data
	while(((*SR>>6)&1) !=1);// WAIT TC len 1, clear
	*SR &=~(1<<6);

}

   /*Recieve data UART function*/
char uart_read_byte()
{

	uint32_t* SR=(uint32_t*)0x40004400;
	uint8_t* DR=(uint8_t*)0x40004404;
    while(((*SR>>5)&1) !=1);
		 //write du lieu vao bien data
	*SR &=~(1<<5);

	    return *DR;
}


void systick_init()
{
	uint32_t* STRVR=(uint32_t)0xe000e014;
	*STRVR=16000000;
	uint32_t* STCSR=(uint32_t)0xe000e010;
	*STRVR |=1 |(1<<1);

}

  /*DMA direct memory access*/
void dma_uart_init()
{
	__HAL_RCC_DMA1_CLK_ENABLE();
	uint32_t* S7CR=0x400260b8;
	uint32_t* S7NDTR=0x400260bc;
	uint32_t* S7PAR=0x400260c0;
	uint32_t* S7M0AR=0x400260c4;//stream x memory 0 address

	*S7NDTR=sizeof(uart2_rx_buff);
	*S7PAR=0x40004404;
	*S7M0AR=(uint32_t)uart2_rx_buff;

	*S7CR &=~(0b111<<25);
	*S7CR |=(0b110<<25)|(1<<0)|(1<<10)|(1<<8); //Select channel, enable DMA, Enable Memory increment mode
	*S7CR|=(1<<4); //Transfer complete interrupt enable
}



/*SPI protocol*/
void spi_init()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	uint32_t* MODER=0x40020000;
	//*MODER &=~(0x<<10); //CLEAR BIT
	*MODER|=(0b10<<10)|(0b10<<12)|(0b10<<14); //Set PA5,6,7 as alternate function

	uint32_t* AFRL=(uint32_t*)0x40020020;
	//*AFRL &=~(0xfff<<);
	*AFRL |= (5 <<20); //AND 0101   //Set MOSI, MISO, SCLK MODE for PA5,6,7
	*AFRL |= (5 <<24);
	*AFRL |= (5 <<28);

   __HAL_RCC_SPI1_CLK_ENABLE();
	uint32_t* SPI_CR1=0x40013000;
	*SPI_CR1|=(1<<2);// set Master mode
	*SPI_CR1|=(0b011<<3); //set fSPI  start 3-5  -Baudrate
	*SPI_CR1|=(1<<9)|(1<<8);////Set Select slave
	//*SPI_CR1|=(1<<11);// d a t a frame 8 bit


	//Set mode 3 do cam bien dung mode 3
	*SPI_CR1 |= (1<<1);
	*SPI_CR1 |= (1<<0);

	*SPI_CR1|=(1<<6);//enable SPI

     /*Set select slave*/
	__HAL_RCC_GPIOE_CLK_ENABLE();//SET OUPUT PE3
	uint32_t* GPIOE_MODER=0x40021000;
	*GPIOE_MODER &=~(0b11<<6);
	*GPIOE_MODER |=(0b01<<6);//Set PE3 PORT
	uint32_t* GPIOE_ODR=0x40021014;
	*GPIOE_ODR|=(1<<3); //Set output

}

     /*Send Data SPI*/
void spi_send(uint32_t data)
{

	uint8_t temp;
	//active
	uint32_t* GPIOE_ODR=0x40021014;
	*GPIOE_ODR &=~(1<<3); //Set SS ACTIVE
	uint32_t* SR=(uint32_t*)0x40013008;
    uint8_t* DR=(uint8_t*)0x4001300c;
	while(((*SR>>1)&1) !=1);
	//*DR=(data<<8)|(0XFF); //write du lieu can truyen vao thanh ghi DR
	*DR=data;
	while(((*SR>>7)&1) ==1);//CHECK BSY  register
	while(((*SR>>0)&1) !=1);
	temp=*DR;  //Could use (void)*DR

}

       /*Recieve Data SPI*/
uint32_t spi_recv()
{
	uint8_t data;
	uint32_t* GPIOE_ODR=0x40021014;
	*GPIOE_ODR&=~(1<<3);
	uint32_t* SR=(uint32_t*)0x40013008;
	uint8_t* DR=(uint8_t*)0x4001300c;

    while(((*SR>>1)&1) !=1);
    *DR=0xFF;// Creat a fake clock for slave
    while(((*SR>>7)&1) !=1);
    while(((*SR>>0)&1) !=1);
    data = *DR;
	return data;
}

             /*I2C*/
void i2c_init()
{
	/* Initialize Alternate function*/
	__HAL_RCC_GPIOB_CLK_ENABLE();
	uint32_t* MODER=(uint32_t*)0x40020400;
	uint32_t* AFRL=(uint32_t*)0x40020420;
	uint32_t* AFRH=(uint32_t*)0x40020424;

	*MODER &=~(0b11<<12);
	*MODER|=(0b10<<12);
	*MODER &=~(0b11<<18);
	*MODER|=(0b10<<18);

	*AFRL &=~(0b1111<<24);
	*AFRL |=(4<<24);
	*AFRH &=~(0b1111<<4);
	*AFRH |=(4<<4);

	__HAL_RCC_I2C1_CLK_ENABLE();
	uint32_t* CR1=(uint32_t*)0x40005400;
	uint32_t* CR2=(uint32_t*)0x40005404;
	uint32_t* CCR=(uint32_t*)0x4000541c;

	*CR1&=~(1);//RESET before setting up I2C
	*CR2 |=16; //config peripheral clock  16Mhz= FI2C
	*CCR |=160;  //CONFIG clock 16Mhz /80 =200Khz
    *CR1 |=(1); //enable I2C
}

uint8_t i2c_read()
{
	uint32_t* CR1 =(uint32_t*)0x40005400;
	uint32_t* SR1 =(uint32_t*)0x40005414;
	uint32_t* SR2 =(uint32_t*)0x40005418;
	uint8_t*  DR  =(uint32_t*)0x40005410;

	const uint8_t sensor_address =0b0011001<<1;//sensor address of LSM303agr
	const uint8_t write_bit=0;
	const uint8_t read_bit=1;

	while(((*SR2>>1)&1) ==1);                   //wait  busy flag is cleaned
	*CR1|=(1<<8);                              // GENERATE Start bit
	while(((*SR1>>0)&1) !=1);                 //wait start bit is generated
	*DR=sensor_address|write_bit;            //address sensor + bit write
	while(((*SR1>>1)&1) !=1);               // wait address is sent
    uint32_t temp= *SR2;                   //read this bit to clear ADD in SR1
	*DR=0x0F; //Send  who am i
	while(((*SR1>>2) & 1) !=1);          //wait  Byte transfer finished
	if((*SR1>>10)==1)                   //wait ACK
	{
		*SR1 &=~(1<<10);
		return 0;
	}

	*CR1&=~(1<<10);   // Turn off ack before recieve data
	*CR1|=(1<<8);  //generate12 Start bit
	while(((*SR1>>0) & 1)!=1); //wait bit is generate
	*DR=sensor_address|read_bit;  //address + bit read
	while(((*SR1>>1) & 1)!=1);
	temp=*SR2;                 //read this bit to clear ADD in SR1
	while(((*SR1>>6) & 1)!=1);
	uint8_t data=*DR;
	*CR1 |=(1<<9); //Stop generation
	return data;

}


               /*Flash*/
typedef enum{
	SECTION_0,
	SECTION_1,
	SECTION_2,
	SECTION_3,
	SECTION_4,
	SECTION_5,
	SECTION_6,
	SECTION_7
}section_num_t;

void flash_erase(section_num_t num)
{
	uint32_t* SR=(uint32_t*)0x40023c0c;      //Status register
	uint32_t* CR=(uint32_t*)0x40023c10;      //Control register
	uint32_t* KEYR=(uint32_t*)0x40023c04;    //Flash key register

    if(((*CR>>31)&1)==1) //check lock ->if CR is locked -> write 0x45 to unclock
		{
			*KEYR=0x45670123;
			*KEYR=0xCDEF89AB;
		}
	while(((*SR>>16)&1)==1); //check bit busy in SR

	*CR |=1<<1;                     //sector erase mode
	*CR |=num<<3;                    //section 7  from 0x080600000-(128kbytes)

	*CR |=1<<16;                 //Start
	while(((*SR>>16)&1)==1);     //check bit busy in SR
	*CR &=~(1<<1);	           //Erase done-> pull to 0
}

void program_flash(void* address,char* buffer,int size)
{
	uint32_t* SR=(uint32_t*)0x40023c0c;      //Status register
	uint32_t* CR=(uint32_t*)0x40023c10;      //Control register
	uint32_t* KEYR=(uint32_t*)0x40023c04;
	if(((*CR>>31)&1)==1)
			{
				*KEYR=0x45670123;
				*KEYR=0xCDEF89AB;
			}

	while(((*SR>>16)&1)==1);

    *CR |=1;
    //Select program mode


    uint8_t* flash_data=(uint8_t*)address;
    for(int i=0; i<size ; i++)
    {
    	*flash_data=buffer[i]; //creat a pointer, points to memory wanna write data
    	 flash_data++;
    }

   // *flash_data= 'x';
    while(((*SR>>16)&1)==1);
}





          /*ADC*/
void adc_int()
{
	uint32_t* MODER=(uint32_t*)0x40020000;
	*MODER &=~(11<<2);//CLEAR
	*MODER |=(11<<2); //Set Analog mode for PA1 ADC1_1
}

                      /*Main Function*/
int main()
{
	// HAL_Init();
	 gpio_d12_init();
	 gpio_button_init();
	 void* dst=0x20000000;//address of RAM   memory- distance
	 void* src=0x08000000;//address of flash memory source
     memcpy(dst,src,0x198);

     /*Bao voi ARM len RAM tim bang VTTB*/
     uint32_t* VTOR=0xe000ed08;
     *VTOR=0x20000000; //Inform for ARM- VTTB at this address

     /*Khai bao dia chi ham thuc thi EXTI0 tren RAM*/
     uint32_t* temp=0x20000058;
     *temp=(uint32_t)custom_exti0 |1;// Subcribe function custom_extio0 into temp -->theo lenh thumd QUY TAC phai or voi 1

     /*Khai bao dia chi ham thuc thi timer1_update tren RAM*/
     temp=0x200000a4;
     *temp=(uint32_t)timer1_update_handler |1; /*Dang ki len bang vector table da doi len RAM cho ham custom timer1_update*/

     // timer1_init();

     temp=0x200000d8;
     *temp=(uint32_t)uart2_handler|1;

     temp=0x200000FC;
     *temp=(uint32_t)dma_handler|1;

     //temp=0x2000003c;
     //*temp=(uint32_t)systick_handler()|1;
     pwm_init();
     capture_init();
     uint16_t* CCR1=(uint16_t*)0x40010034;

     uart_init();
     dma_uart_init();
     systick_init();
     spi_init();

     //i2c_init();
    //i2c_ss_id=i2c_read();
    char msg[]="Hello Flash";
    flash_erase(SECTION_7);
    program_flash(0x08060000,msg,sizeof(msg));
     //char* msg="HELLO \r\n";
     //uint32_t* SR=(uint32_t*)0x40004400;
     //uint8_t* DR=(uint8_t*)0x40004404;
	 //uint32_t* GPIOE_ODR=0x40021014;
	 // *GPIOE_ODR&=~(1<<3); //Enale CS
	 // spi_send(0X8F);  //SEND 0X0F 00001111 + BIT READ = 1 at position 7 =>> 1000 1111
	 // spi1_rx_buff[spi_index]=spi_recv(); //revice ID device
	 // *GPIOE_ODR|=(1<<3); //Disable CS

  while(1)
	{

    }
     return 0;
}

/* Send string data UART

	  int len=strlen(msg);
	for( int i=0;i<=len;i++)
	{
	 uart_write_byte(msg[i]);
	}
*/

/*Read  data UART
*  uart_write_byte('x');
	Rx_data=uart_read_byte();
*/

/* Read value of Z Y Z axis (i3g4250) in while
	     *GPIOE_ODR&=~(1<<3); //Enale CS
	     spi_send(0x20);    //CTRL_REG1 (20h) register
	     spi_send(0x0F); // set PD ,x,y,z axis
		 *GPIOE_ODR |=(1<<3); //Enale CS

       *GPIOE_ODR&=~(1<<3); //Enale CS
		  spi_send(0x28|1<<7|1<<6); // Adress of  OUT_X_L , set read, set MS : multi
		  low=spi_recv();
		  high=spi_recv();
		  x_axis=((high<<8)|low);

		  spi_send(0x2A|1<<7|1<<6); //Adress of  OUT_Y_L , set read, set MS : multi
		  low=spi_recv();
		  high=spi_recv();
		  y_axis=((high<<8)|low);

		  spi_send(0x2C|1<<7|1<<6); //Adress of  OUT_Y_L , set read, set MS : multi
		  low=spi_recv();
		  high=spi_recv();
		  z_axis=((high<<8)|low);
       *GPIOE_ODR|=(1<<3);//Disale CS UP TO 1 to complete the transfer and recieve process
*/

                                        /*Interupt functions*/

         /*EXTI function custom*/
void custom_exti0()
{

	__asm("NOP");
	button_cnt++; //Moi lan nhan nut nhay vao function interrupt thi tang len 1
	Custom_delay(500000);
	//gpio_d14_init();
	uint32_t* PR= 0x40013c14;
	*PR |=(1<<0);
}


       /* Delay function custom */
void Custom_delay(uint32_t time)// TRUYEN VAO 1 gia tri
{
	for(uint32_t i=0; i<time; i++)
	{
		__asm("NOP");
	}
}



//void Custom_delay(uint64_t time)
//{
//	systick_cnt=0;
//	while(systick_cnt<time);
//}


           /*Interrup function*/
void EXTI0_IRQHandler()
{
    __asm("NOP");
    button_cnt++; //Moi lan nhan nut nhay vao function interrupt thi tang len 1
    Custom_delay(500000);
	uint32_t* PR= 0x40013c14;
	*PR |=(1<<0);
}


/* Uart interrupt*/
void uart2_handler() // Khi co data gui xuong ,jump into this function instead of waiting
{
	//while(((*SR>>5)&1) !=1);
	uint32_t* SR=(uint32_t*)0x40004400;
	uint8_t* DR=(uint8_t*)0x40004404;
    uart2_rx_buff[uart_rx_index]= *DR;
	uart_rx_index++;// moi lan doc bo vao vung nho tiep theo
	*SR &=~(1<<5);
}



/*DMA function*/
void dma_handler()
{

//
	static uint8_t flag;
	flag = 1 - flag;
	__HAL_RCC_GPIOD_CLK_ENABLE(); //Khoi tao xung cap clock
	uint32_t* GPIOD_MODER =0x40020c00;// tro toi dia chi Moder
	*GPIOD_MODER &= ~(0b11<<24);//dich bit 24 25 clear
	*GPIOD_MODER |= 0b01<<24;// or voi bit 01 for output mode

	uint32_t* GPIOD_ODR=(uint32_t*)0x40020c14;
	if (flag)
		*GPIOD_ODR |= 1<<12;
	else
		*GPIOD_ODR &= ~(1<<12);
  //__asm("NOP");
	uint32_t* HIFCR=(uint32_t*)0x4002600c; //Clear
	*HIFCR|=(1<<27);
}


void systick_handler()
{
	__asm("NOP");
}


/*
void i2c_send(uint32_t address, uint32_t subaddrr,uint32_t data)
{
	//char temp;
	uint32_t* CR1 =(uint32_t*)0x40005400;
	uint8_t*  DR  =(uint32_t*)0x40005410;
	uint32_t* SR1 =(uint32_t*)0x40005414;
	uint32_t* SR2 =(uint32_t*)0x40005418;
	//*CR1 |=(1<<0);
	address =address<<1;//sensor address

	while(((*SR2>>1)&1) ==1);//wait  busy flag is cleaned

	*CR1|=(1<<8);// GENERATE Start bit
	while((*SR1>>1)&1 !=1); //wait start bit is generated
	*DR=address|0; //address + bit write
	while(((*SR1>>1)&1) !=1);// wait address is sent
	uint32_t temp= *SR2;//clear ADDR


	while(((*SR1>>7) & 1) ==1);//WAIT TXE
	*DR=0x0F; //Send  who am i
	while(((*SR1>>2) & 1) !=1); //wait  Byte transfer finished
	while(((*SR1>>10) & 1) ==1); //wait ACK

	 temp= *SR2;
	while(((*SR1>>7) & 1) ==1); //wait ACK
	*DR=subaddrr; //Send  who am i
	while(((*SR1>>2) & 1) !=1); //wait  Byte transfer finished
	while(((*SR1>>7) & 1) ==1); //wait ACK


}
*/


