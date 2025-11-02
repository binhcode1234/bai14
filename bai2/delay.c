#include "stm32f10x.h"

void GPIO_Config(void);
void Delay_ms(uint32_t time);
void Delay_ms(uint32_t time)
{
    for (uint32_t i = 0; i < time * 800; i++)
    {
        __NOP(); 
    }
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

    while (1)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_0);
        GPIO_ResetBits(GPIOB, GPIO_Pin_1);
        Delay_ms(500);
        GPIO_ResetBits(GPIOB, GPIO_Pin_0);
        GPIO_SetBits(GPIOB, GPIO_Pin_1);
        Delay_ms(500);
    }
}
