/*
 * test_nico.c
 *
 *  Created on: Jul 24, 2013
 *      Author: Nicolas Turro <nicolas.turro.at.inria.fr>
 */

#include "platform.h"
#include "iotlab-a8-m3/iotlab-a8-m3.h"
#include "printf.h"
#include "gpio.h"
#include "afio.h"
#include "nvic_.h"

#include "memmap.h"
#include "afio_registers.h"
#include "timer_registers.h"

#include "stm32f1xx.h"

// our STM32F103 runs at 72Mhz, 72000065 is an average measurement from a external GPS PPS source.
#define CYCLESPERSEC 72000065.0
#define CYCLES2NS 1000000000.0/CYCLESPERSEC

#define START 0
#define MAGIC1 1
#define MAGIC2 2
#define TIMEOK 3
#define MAGICCHAR1 0x16
#define MAGICCHAR2 0x3

//  DWT_CYCNT counter is used instead of SysTick because it's a 32bits counter which overflows once a minute whereas
// SysTick overflows every 234 ms
volatile unsigned int *DWT_CYCCNT     = (volatile unsigned int *)0xE0001004; //address of the register
volatile unsigned int *DWT_CONTROL    = (volatile unsigned int *)0xE0001000; //address of the register
volatile unsigned int *SCB_DEMCR         = (volatile unsigned int *)0xE000EDFC; //address of the register

volatile uint32_t seconds=0;
volatile int A8comState=START;

typedef struct {
  uint32_t sec;
  uint32_t nsec;
} time_t;


// return the time elapsed since the beginning of the application
void getTime(time_t * curtime){
  // the seconds are updated by the PPShandler, nothing to do
  (*curtime).sec=seconds;
  // the nsec are computed from the number of cycles elapsed since the last second
  (*curtime).nsec=*DWT_CYCCNT*(double)CYCLES2NS;
}

#define TIMcap TIM_2
#define X 16384

static unsigned int sysTickCounter = 0;

void printb(uint32_t data)
{
  int i;
  for (i=0;i<32;i++) {
    int flag = ((data & (1<<(31-i))) != 0);
    if (i%8 == 0  && i != 0)
      printf(" ");
    if (flag) printf("1");
    else printf("0");
  }
}

// TIMx_CCMR2::CC4S is bits 9:8 (RM0008 / DocID 13902 Rev 15 / p409/1128)
#define AS_CC4S(x) ((x)<<8)

// TIMx_CCMR2::IC4F is bits 15:12 (RM0008 / DocID 13902 Rev 15 / p410/1128)
#define AS_IC4F(x) ((x)<<12)

// TIMx_CCER::CC4P is bit 13 (RM0008 / DocID 13902 Rev 15 / p410/1128)
#define AS_CC4P(x) ((x)<<13)

// TIMx_CCER::CC4E is bit 12 (RM0008 / DocID 13902 Rev 15 / p410/1128)
#define AS_CC4E(x) ((x)<<12)

// TIMx_CCMR2::IC4F is bits 11:10 (RM0008 / DocID 13902 Rev 15 / p410/1128)
#define AS_IC4PSC(x) ((x)<<10)

void PPShandler(handler_arg_t arg) {
  (void)arg;


  static unsigned int lastSysTickCounter = 0;
  static unsigned int lastTimerCounter = 0;

  sysTickCounter = *DWT_CYCCNT;
  unsigned int timer2 = *timer_get_CNT(TIM_2);
  unsigned int timer4 = *timer_get_CNT(TIM_4);

  unsigned int timerCounter = timer2 + (timer4 * X);


#define TIM2_BASE 0x40000000ul
#define TIM4_BASE 0x40000800ul
#define TIM_CNT_OFFSET 0x24

#define TIM2_COUNTER (*((volatile uint16_t*)(TIM2_BASE+TIM_CNT_OFFSET)))
#define TIM4_COUNTER (*((volatile uint16_t*)(TIM4_BASE+TIM_CNT_OFFSET)))

  unsigned int timerCounter2 = TIM2_COUNTER + TIM4_COUNTER * X;

  // we recieve this interrupt each second from the GPS PPS
  // so we increment the number of seconds elapsed from the beginning
  seconds++;
  // and we reset the cycle counter
  //*DWT_CYCCNT = 0;

  printf("%u %u %u\n", sysTickCounter-lastSysTickCounter,
	 timerCounter -lastTimerCounter, timerCounter2-timerCounter );
  printf("timer4=%u\n", timer2);
  //  printf("ccr4=%u ", *timer_get_CCRx(TIMcap, 3));
  //printf("sr="); printb(*timer_get_SR(TIMcap)); printf("\n");
  //printf("ccr4=%u ", *timer_get_CCRx(TIMcap, 3));
  //printf("sr="); printb(*timer_get_SR(TIMcap)); printf("\n");
  lastSysTickCounter = sysTickCounter;
  lastTimerCounter = timerCounter;
  printf("DELTA=%u\n", timer2-*timer_get_CCRx(TIMcap, 3));

  volatile uint16_t* ccer = timer_get_CCER(TIMcap);
  *ccer = ((*ccer) ^ AS_CC4E(1)); // capture toggle
  //*timer_get_CNT(TIM_2) = 0;
  //*timer_get_CNT(TIM_4) = 0;
}

void char_rx(handler_arg_t arg, uint8_t c)  {
  static union {
          uint8_t c[4];
          uint32_t i;
  } A8buf;
  static int A8comCount;

  asm volatile ("cpsid i");

  switch (A8comState)  {
  case ' ':
    printf(".");
    break;
  case START :
    if (c==MAGICCHAR1) A8comState=MAGIC1;
    break;
  case MAGIC1 :
    if (c==MAGICCHAR2) {
      A8comState=MAGIC2;
      A8comCount=0;
    }
    else A8comState=START;
    break;
  case MAGIC2 :
    A8buf.c[A8comCount]=c;
    A8comCount++;
    if (A8comCount==4) {
      seconds = A8buf.i;
      A8comState=TIMEOK;
    }
    break;
  case TIMEOK :
    if (c==MAGICCHAR1) A8comState=MAGIC1;
    break;
  }
  asm volatile ("cpsie i");
}

//typedef typeof(TIM_2) openlab_timer_t;

void set_capture(typeof(TIM_2) _timer)
{
  volatile uint16_t* ccmr2 = timer_get_CCMRx(_timer, 2);
  volatile uint16_t* ccer = timer_get_CCER(_timer);
  printf("CCMR2="); printb(*ccmr2); printf("\n");
  *ccmr2 = ((*ccmr2) & ~AS_CC4S(0x3)) | AS_CC4S(0x1); // IC4 is mapped on TI4
  *ccmr2 = ((*ccmr2) & ~AS_IC4F(0xf)) | AS_IC4F(0x0); // no filtering in IC4
  *ccer = ((*ccer) & ~AS_CC4P(1)) | AS_CC4P(0); // capture on rising edge of IC4
  *ccmr2 = ((*ccmr2) & ~AS_IC4PSC(0x3)) | AS_IC4PSC(0); // no prescaler
  *ccer = ((*ccer) & ~AS_CC4E(1)) | AS_CC4E(1); // capture enabled for IC4
  printf("CCMR2="); printb(*ccmr2); printf("\n");
  printf("CCER="); printb(*ccer); printf("\n");
  printf("DIER="); printb(*timer_get_DIER(_timer)); printf("\n");
}

int main() {
  time_t curtime;
  int i;
  platform_init();

  printf("nico test v2\n===================\n");
  printf("get_clock_frequency=%u\n", 
	 rcc_sysclk_get_clock_frequency(RCC_SYSCLK_CLOCK_PCLK1_TIM));

  //..................................................
  timer_enable(TIM_2);
  timer_select_internal_clock(TIM_2,
       (rcc_sysclk_get_clock_frequency(RCC_SYSCLK_CLOCK_PCLK1_TIM)/1000000)-1);
  // rcc_sysclk_get_clock_frequency(RCC_SYSCLK_CLOCK_PCLK1_TIM));
  timer_select_internal_clock(TIM_2, 0);
  //(rcc_sysclk_get_clock_frequency(RCC_SYSCLK_CLOCK_PCLK1_TIM)/1000000)-1);
  timer_start(TIM_2, X-1, NULL, NULL);

  // config TRGO
  *timer_get_SMCR(TIM_2) |= TIMER_SMCR__MSM;
  #define TIM_TRGOSource_Update              ((uint16_t)0x0020)
  *timer_get_CR2(TIM_2) |= TIM_TRGOSource_Update;

  //....................
  timer_enable(TIM_4);
  *timer_get_SMCR(TIM_4) = TIMER_SMCR__SMS_EXTERNAL_CLOCK_MODE_1;

  // Input trigger
  uint16_t tmpsmcr = 0;
  tmpsmcr = *timer_get_SMCR(TIM_4);
  tmpsmcr &= (uint16_t)(~((uint16_t)TIMER_SMCR__TS_MASK));
  #define TIM_TS_ITR1      ((uint16_t)0x0010)
  tmpsmcr |= TIM_TS_ITR1;
  *timer_get_SMCR(TIM_4) = tmpsmcr;

  // Enable CR1 ARPE
  *timer_get_CR1(TIM_4) |= TIMER_CR1__ARPE;

  // Enable the counter
  *timer_get_CR1(TIM_4) |= TIMER_CR1__CEN;
  //....................

  *timer_get_CNT(TIM_2) = 0;
  *timer_get_CNT(TIM_4) = 0;
  //..................................................

  set_capture(TIMcap);

  uart_set_rx_handler(uart_print, char_rx, NULL);

#if 0
  while(A8comState!=TIMEOK){
    asm volatile ("wfi");
  }
#endif

  printf("AFIO_EVCR: "); printb(*afio_get_EVCR()); printf("\n");
  printf("AFIO_MAPR: "); printb(*afio_get_MAPR()); printf("\n");
  printf("0xabcdef75: "); printb(0xabcdef75); printf("\n");


  // Init cycle counter
  *SCB_DEMCR = *SCB_DEMCR | 0x01000000;
  *DWT_CYCCNT = 0; // reset the counter
  *DWT_CONTROL = *DWT_CONTROL | 1 ; // enable the counter

  printf("init cycle\n");

  // GPS PPS output is wired to GPIO PA3
  gpio_set_input(gpioA, GPIO_PIN_3);
  gpio_enable(gpioA);

  // link GPIO PA3 to IRQ_LINE_EXTI3
  afio_select_exti_pin(EXTI_LINE_Px3, AFIO_PORT_A);
  nvic_enable_interrupt_line(NVIC_IRQ_LINE_EXTI3);

  // set IRQ handler and configure call to RISING signal on PA3
  exti_set_handler(EXTI_LINE_Px3, PPShandler, NULL);
  exti_enable_interrupt_line(EXTI_LINE_Px3, EXTI_TRIGGER_RISING);

  while(1){
    //wait a bit
    for(i=0;i<10000000;i++)    asm("nop");
    //get and print the elapsed time
    getTime(&curtime);
    printf("Time : %d.%09d\n",curtime.sec,curtime.nsec);
  };
  return 0;
}
