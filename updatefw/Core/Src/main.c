
#include "main.h"

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

char new_fw[2684]={0};     //define a var to get the gpio_hal.bin file with 2684 bytes
char recv_fw_done=0;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);

typedef  enum
{
	SECTION_0,
	SECTION_1,
	SECTION_2,
	SECTION_3,
	SECTION_4,
	SECTION_5,
	SECTION_6,
	SECTION_7,
}section_num_t;    //Define section 0-7 for user

__attribute__((section(".RunInRam"))) void flash_erase(section_num_t num)
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
	*CR |=num<<3;                    //num = sector 0-7  start from 0x08000000-(128kbytes)

	*CR |=1<<16;                 //Start
	while(((*SR>>16)&1)==1);     //check bit busy in SR
	*CR &=~(1<<1);	           //Erase done-> pull to 0
}

__attribute__((section(".RunInRam"))) void program_flash_write(void* address, char* buffer, int size)
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

    *CR |=1;//Select program mode
    uint8_t* flash=(uint8_t*)address;    //creat a pointer, points to memory wanna write data

    for(int i=0;i<size;i++)
        {
	      *flash=buffer[i]; //write data into flash
	       flash++;
        }
   while(((*SR>>16)&1)==1);
}

__attribute__((section(".RunInRam"))) void update_fw()
{
	uint32_t* STCSR=(uint32_t*)0xe000e010;
	*STCSR &=~(1); //reset systemtick before erase
	flash_erase(SECTION_0); //Alway start section 0 for updating FW
	program_flash_write(0x08000000,new_fw,sizeof(new_fw));
	uint32_t* AIRCR=(uint32_t*)(0XE000ED0C);
	*AIRCR =(0x5fa<<16) | (1<<2); //Reset chip
}

int main(void)
{

  HAL_Init();
  //SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  HAL_UART_Receive_DMA(&huart2,new_fw,sizeof(new_fw));
  while(recv_fw_done !=1);
  update_fw();
  while (1)
  {

  }

}


static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/*Enable DMA controller clock*/
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}


static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

