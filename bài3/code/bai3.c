/*
 * STM32F103C8 — Periodic RTC Alarm ? Wake, send UART, then Standby.
 * UART1 @ 9600 (PA9/PA10).
 * PC13 làm "RUN marker": RUN = LED ON (PC13 LOW), tru?c Standby = LED OFF r?i AIN.
 */

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_pwr.h"
#include "stm32f10x_bkp.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_rtc.h"

#define WAKE_INTERVAL_S  10U
#define UART_BAUD        9600U
#define RTC_MAGIC        0xA5A5
#define BKP_MAGIC_REG    BKP_DR1
#define BKP_COUNT_REG    BKP_DR2

/* ---------------- Tiny delay ---------------- */
static void uart_delay_flush_ms(uint32_t ms){
  for (uint32_t i=0;i<ms;i++) for (volatile uint32_t j=0;j<8000;j++) __NOP();
}

/* ---------------- RUN marker (PC13) ---------------- */
static void MARKER_Init_PC13(void){
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  GPIO_InitTypeDef io; GPIO_StructInit(&io);
  io.GPIO_Pin   = GPIO_Pin_13;
  io.GPIO_Speed = GPIO_Speed_2MHz;
  io.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &io);
}
static void MARKER_Run_ON(void){
  // BluePill LED PC13 active-low: kéo LOW -> sáng
  GPIO_ResetBits(GPIOC, GPIO_Pin_13);
}
static void MARKER_Run_OFF(void){
  GPIO_SetBits(GPIOC, GPIO_Pin_13);
}
static void MARKER_To_Analog(void){
  // Ðua PC13 v? Analog In d? gi?m rò khi Standby
  GPIO_InitTypeDef io; GPIO_StructInit(&io);
  io.GPIO_Pin  = GPIO_Pin_13;
  io.GPIO_Mode = GPIO_Mode_AIN;
  io.GPIO_Speed= GPIO_Speed_2MHz;
  GPIO_Init(GPIOC, &io);
  // (Không t?t clock GPIOC d? gi? c?u hình; Standby s? t?t VCORE)
}

/* ---------------- UART1 ---------------- */
static void UART1_Init_9600(void){
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1, ENABLE);
  GPIO_InitTypeDef io; GPIO_StructInit(&io);
  io.GPIO_Pin=GPIO_Pin_9; io.GPIO_Speed=GPIO_Speed_50MHz; io.GPIO_Mode=GPIO_Mode_AF_PP; GPIO_Init(GPIOA,&io);
  io.GPIO_Pin=GPIO_Pin_10; io.GPIO_Mode=GPIO_Mode_IN_FLOATING; GPIO_Init(GPIOA,&io);
  USART_InitTypeDef u; USART_StructInit(&u);
  u.USART_BaudRate=UART_BAUD; u.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;
  USART_Init(USART1,&u); USART_Cmd(USART1,ENABLE);
}
static void UART1_SendChar(char c){
  if (c=='\n'){ while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET){} USART_SendData(USART1,'\r'); }
  while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET){} USART_SendData(USART1,(uint16_t)c);
}
static void UART1_SendString(const char* s){ while(*s) UART1_SendChar(*s++); }
static void UART1_SendDec(uint32_t v){
  char buf[12]; int i=0; if(!v){ UART1_SendChar('0'); return; }
  while(v && i<11){ buf[i++]='0'+(v%10); v/=10; } while(i){ UART1_SendChar(buf[--i]); }
}
static void UART1_DeInit_LowLeak(void){
  USART_Cmd(USART1,DISABLE); USART_DeInit(USART1);
  GPIO_InitTypeDef io; GPIO_StructInit(&io);
  io.GPIO_Mode=GPIO_Mode_AIN; io.GPIO_Speed=GPIO_Speed_2MHz;
  io.GPIO_Pin=GPIO_Pin_9;  GPIO_Init(GPIOA,&io);
  io.GPIO_Pin=GPIO_Pin_10; GPIO_Init(GPIOA,&io);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,DISABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,DISABLE);
}

/* ---------------- RTC 1 Hz (init once) ---------------- */
static int RTC_Ensure_1Hz_InitOnce(void){
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR|RCC_APB1Periph_BKP, ENABLE);
  PWR_BackupAccessCmd(ENABLE);

  if (BKP_ReadBackupRegister(BKP_MAGIC_REG)!=RTC_MAGIC){
    RCC_LSEConfig(RCC_LSE_ON);
    uint32_t t=0x400000;
    while((RCC_GetFlagStatus(RCC_FLAG_LSERDY)==RESET) && t--){}
    if (RCC_GetFlagStatus(RCC_FLAG_LSERDY)==SET){
      RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
    }else{
      RCC_LSICmd(ENABLE);
      t=0x400000; while((RCC_GetFlagStatus(RCC_FLAG_LSIRDY)==RESET)&&t--){}
      if (RCC_GetFlagStatus(RCC_FLAG_LSIRDY)==RESET) return 0;
      RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
    }
    RCC_RTCCLKCmd(ENABLE); RTC_WaitForSynchro();
    uint32_t presc=(RCC_GetFlagStatus(RCC_FLAG_LSERDY)==SET)?32767U:39999U;
    RTC_WaitForLastTask(); RTC_SetPrescaler(presc); RTC_WaitForLastTask();
    RTC_SetCounter(0); RTC_WaitForLastTask();
    BKP_WriteBackupRegister(BKP_MAGIC_REG, RTC_MAGIC);
    BKP_WriteBackupRegister(BKP_COUNT_REG, 0);
  } else {
    RCC_RTCCLKCmd(ENABLE); RTC_WaitForSynchro();
  }

  RTC_ClearFlag(RTC_FLAG_ALR|RTC_FLAG_OW|RTC_FLAG_SEC); RTC_WaitForLastTask();
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  EXTI_ClearITPendingBit(EXTI_Line17);
  EXTI_InitTypeDef ex; ex.EXTI_Line=EXTI_Line17; ex.EXTI_Mode=EXTI_Mode_Event; ex.EXTI_Trigger=EXTI_Trigger_Rising; ex.EXTI_LineCmd=ENABLE; EXTI_Init(&ex);
  RTC_ITConfig(RTC_IT_ALR, ENABLE); RTC_WaitForLastTask();
  return 1;
}
static void RTC_SetAlarm_In(uint32_t seconds){
  uint32_t now=RTC_GetCounter();
  RTC_WaitForLastTask(); RTC_SetAlarm(now+seconds); RTC_WaitForLastTask();
}

/* ---------------- Low-power prep & Standby ---------------- */
static void IO_AllB_ToAnalog(void){
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  GPIO_InitTypeDef io; GPIO_StructInit(&io);
  io.GPIO_Mode=GPIO_Mode_AIN; io.GPIO_Speed=GPIO_Speed_2MHz; io.GPIO_Pin=GPIO_Pin_All;
  GPIO_Init(GPIOB,&io);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, DISABLE);
}
static void Standby_Enter(void){
  PWR_ClearFlag(PWR_FLAG_WU); PWR_ClearFlag(PWR_FLAG_SB);
  RTC_ClearFlag(RTC_FLAG_ALR); RTC_WaitForLastTask();
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  EXTI_ClearITPendingBit(EXTI_Line17);
  PWR_WakeUpPinCmd(DISABLE);
  __SEV(); __WFE();
  PWR_EnterSTANDBYMode();
  while(1){}
}

/* ---------------- Boot counter ---------------- */
static uint32_t Counter_Inc_Read(void){
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR|RCC_APB1Periph_BKP, ENABLE);
  PWR_BackupAccessCmd(ENABLE);
  uint16_t v=BKP_ReadBackupRegister(BKP_COUNT_REG); v++; BKP_WriteBackupRegister(BKP_COUNT_REG, v);
  return v;
}

/* ----------------------------------- main ----------------------------------- */
int main(void){
  /* X? lý c? Standby/WU s?m */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  if (PWR_GetFlagStatus(PWR_FLAG_SB)!=RESET){ PWR_ClearFlag(PWR_FLAG_SB); }
  PWR_ClearFlag(PWR_FLAG_WU);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  EXTI_ClearITPendingBit(EXTI_Line17);

  /* Marker PC13: b?t ngay khi RUN */
  MARKER_Init_PC13();
  MARKER_Run_ON();   // RUN => LED ON

  /* UART + RTC */
  UART1_Init_9600();
  if(!RTC_Ensure_1Hz_InitOnce()){
    UART1_SendString("RTC clock not available!\r\n");
    while(1){ uart_delay_flush_ms(200); }
  }

  uint32_t cnt=Counter_Inc_Read();
  UART1_SendString("\r\n[BOOT] Wake cycle #: "); UART1_SendDec(cnt); UART1_SendString("\r\n");
  UART1_SendString("Hello PC, woke from Standby. Next sleep in "); UART1_SendDec(WAKE_INTERVAL_S); UART1_SendString(" s.\r\n");

  /* Lên l?ch Alarm k? ti?p */
  RTC_ClearFlag(RTC_FLAG_ALR|RTC_FLAG_OW|RTC_FLAG_SEC); RTC_WaitForLastTask();
  EXTI_ClearITPendingBit(EXTI_Line17);
  RTC_SetAlarm_In(WAKE_INTERVAL_S);

  /* Ð?m b?o truy?n xong tru?c khi gi?m công su?t */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET) {}
  UART1_DeInit_LowLeak();

  /* T?t marker (LED OFF) và dua PC13 v? Analog In tru?c Standby d? gi?m rò */
  MARKER_Run_OFF();     // LED OFF
  MARKER_To_Analog();   // PC13 -> AIN

  /* Các chân khác v? analog n?u mu?n */
  IO_AllB_ToAnalog();

  /* T?t SysTick n?u có */
  SysTick->CTRL = 0;

  /* Standby: sau WAKE_INTERVAL_S, RTC Alarm (EXTI17) s? reset MCU và l?p l?i */
  Standby_Enter();

  while(1){}
}
