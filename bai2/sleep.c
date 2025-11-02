#include "stm32f10x.h"

volatile uint32_t tick = 0;  

void GPIO_Config(void);
void SysTick_Handler(void);
void SysTick_Handler(void)
{
    tick++; 
}

void GPIO_Config(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_1);
}
int main(void)
{
    SystemInit();      
    GPIO_Config();     

    SysTick_Config(SystemCoreClock / 1000);

    uint32_t last = 0;
    uint8_t ledState = 0;  

    while (1)
    {
        __WFI();  
        if (tick - last >= 500)
        {
            last = tick;

            if (ledState == 0)
            {
                GPIO_SetBits(GPIOB, GPIO_Pin_0);
                GPIO_ResetBits(GPIOB, GPIO_Pin_1);
                ledState = 1;
            }
            else
            {
                GPIO_ResetBits(GPIOB, GPIO_Pin_0);
                GPIO_SetBits(GPIOB, GPIO_Pin_1);
                ledState = 0;
            }
        }
    }
}
