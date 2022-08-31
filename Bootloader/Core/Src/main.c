//bootloader
#include "main.h"
static void MX_GPIO_Init(void);
int main(void)
{
    HAL_Init();
    MX_GPIO_Init();
    uint32_t* reset_pos=0;

    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)==GPIO_PIN_SET)
        reset_pos=(uint32_t*)0x08004004;
    else
    	reset_pos=(uint32_t*)0x08008004;

    uint32_t reset_handler_app1=*reset_pos;
    void(*reset_app1)()=reset_handler_app1;
    reset_app1();
  while (1)
  	  {

  	  }
}
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
