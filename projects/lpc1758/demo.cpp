#include <LPC17xx.H>                        /* LPC17xx definitions */
#include "demo.h"
#include "IRremote.h"

static IRrecv irrecv;
static decode_results results;

uint32_t demo_result_for_add_watch[128];
uint32_t demo_result_for_add_watch_index = 0;

extern "C" void SysTick_Handler(void)
{
  irrecv.timerIrqHandle();
}

uint8_t demo_readIrPin(void)
{
  if((GPIO0->FIOPIN & (1UL<<26)))
    return 1;
  else
    return 0;
}

void demo_TimerReset(void)
{
}

void demo_TimerSet(uint32_t frequency)
{
  SysTick_Config(SystemFrequency/frequency);
}

int main(void)
{
  SystemInit();/* initialize clocks */
  GPIO0->FIODIR   &= ~(1UL<<26);//IR INPUT
  irrecv.readIrPin = demo_readIrPin;
  irrecv.TimerReset = demo_TimerReset;
  irrecv.TimerSet = demo_TimerSet;
  irrecv.irparams.rawlen = 0;
  irrecv.irparams.rcvstate = IRrecv::STATE_IDLE;
  irrecv.timerInit();
  while(true)
  {
    if(irrecv.decode(&results))
    {
      demo_result_for_add_watch[demo_result_for_add_watch_index++]=results.value;
      demo_result_for_add_watch_index&=127;
      irrecv.resume();
    }
  }
}


