#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

/* Event group */
EventGroupHandle_t xEventGroup;

#define BIT_TASK1   (1 << 0)
#define BIT_TASK2   (1 << 1)
#define BIT_TASK3   (1 << 2)

/* ----------------- Hardware Config ----------------- */

void GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* PA1 PA2 PA3 OUTPUT */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PA0 BUTTON (PULL-UP) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

uint8_t Button_Read(void)
{
    return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);  // 0 = nh?n
}

/* ----------------- TASKS ----------------- */

void Task_Control(void *pvParameters)
{
    uint8_t mode = 0;
    uint8_t last = 1;

    while (1)
    {
        uint8_t now = Button_Read();

        if (now == 0 && last == 1)   // detect press
        {
            mode++;
            if (mode > 4) mode = 0;

            /* Clear all bits */
            xEventGroupClearBits(xEventGroup,
                                 BIT_TASK1 | BIT_TASK2 | BIT_TASK3);

            if (mode == 1)
                xEventGroupSetBits(xEventGroup, BIT_TASK1);
            else if (mode == 2)
                xEventGroupSetBits(xEventGroup, BIT_TASK2);
            else if (mode == 3)
                xEventGroupSetBits(xEventGroup, BIT_TASK3);
            else if (mode == 4)
                xEventGroupSetBits(xEventGroup,
                                   BIT_TASK1 | BIT_TASK2 | BIT_TASK3);

            vTaskDelay(200 / portTICK_RATE_MS); // ch?ng d?i
        }

        last = now;
        vTaskDelay(20 / portTICK_RATE_MS);
    }
}

void Task1(void *pvParameters)
{
    while (1)
    {
        xEventGroupWaitBits(xEventGroup, BIT_TASK1,
                            pdFALSE, pdTRUE, portMAX_DELAY);

        GPIOA->ODR ^= GPIO_Pin_1;
        vTaskDelay(200);   // Nháy nhanh
    }
}

void Task2(void *pvParameters)
{
    while (1)
    {
        xEventGroupWaitBits(xEventGroup, BIT_TASK2,
                            pdFALSE, pdTRUE, portMAX_DELAY);

        GPIOA->ODR ^= GPIO_Pin_2;
        vTaskDelay(500);   // Nháy v?a
    }
}

void Task3(void *pvParameters)
{
    while (1)
    {
        xEventGroupWaitBits(xEventGroup, BIT_TASK3,
                            pdFALSE, pdTRUE, portMAX_DELAY);

        GPIOA->ODR ^= GPIO_Pin_3;
        vTaskDelay(800);   // Nháy ch?m
    }
}

/* ----------------- MAIN ----------------- */

int main(void)
{
    SystemInit();
    GPIO_Config();

    xEventGroup = xEventGroupCreate();

    xTaskCreate(Task_Control, "CTRL", 128, NULL, 3, NULL);
    xTaskCreate(Task1, "T1", 128, NULL, 1, NULL);
    xTaskCreate(Task2, "T2", 128, NULL, 1, NULL);
    xTaskCreate(Task3, "T3", 128, NULL, 1, NULL);

    vTaskStartScheduler();

    while (1);
}
