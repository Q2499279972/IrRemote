/*
  Based On IRremote Arduino Library: https://github.com/z3t0/Arduino-IRremote/
*/

#include "IRremote.h"

//IrState::IrState(){}
//IrState::~IrState(){}

bool IrState::match(int measured,  int desired)
{
  return ((measured >= TICKS_LOW(desired))
          && (measured <= TICKS_HIGH(desired)));
}

bool IrState::matchMark(int measured_ticks,  int desired_us)
{
  return ((measured_ticks >= TICKS_LOW(desired_us + MARK_EXCESS))
          && (measured_ticks <= TICKS_HIGH(desired_us + MARK_EXCESS)));
}

bool IrState::matchSpace(int measured_ticks,  int desired_us)
{
  return ((measured_ticks >= TICKS_LOW(desired_us - MARK_EXCESS))
          && (measured_ticks <= TICKS_HIGH(desired_us - MARK_EXCESS)));
}

void IrState::timerInit()
{
  TimerSet((1000000+USECPERTICK/2)/USECPERTICK);
}

void IrState::timerIrqHandle()
{
  TimerReset();
  int irdata = readIrPin();
  irparams.timer++;
  if(irparams.rawlen >= RAWBUF)  irparams.rcvstate = STATE_OVERFLOW ;
  switch(irparams.rcvstate)
  {
  //......................................................................
  case STATE_IDLE: // In the middle of a gap
    if(irdata == MARK)
    {
      if(irparams.timer < GAP_TICKS)      // Not big enough to be a gap.
      {
        irparams.timer = 0;
      }
      else
      {
        // Gap just ended; Record duration; Start recording transmission
        irparams.overflow                  = false;
        irparams.rawlen                    = 0;
        irparams.rawbuf[irparams.rawlen++] = irparams.timer;
        irparams.timer                     = 0;
        irparams.rcvstate                  = STATE_MARK;
      }
    }
    break;
  //......................................................................
  case STATE_MARK:  // Timing Mark
    if(irdata == SPACE)      // Mark ended; Record time
    {
      irparams.rawbuf[irparams.rawlen++] = irparams.timer;
      irparams.timer                     = 0;
      irparams.rcvstate                  = STATE_SPACE;
    }
    break;
  //......................................................................
  case STATE_SPACE:  // Timing Space
    if(irdata == MARK)     // Space just ended; Record time
    {
      irparams.rawbuf[irparams.rawlen++] = irparams.timer;
      irparams.timer                     = 0;
      irparams.rcvstate                  = STATE_MARK;
    }
    else if(irparams.timer > GAP_TICKS)       // Space
    {
      // A long Space, indicates gap between codes
      // Flag the current code as ready for processing
      // Switch to STOP
      // Don't reset timer; keep counting Space width
      irparams.rcvstate = STATE_STOP;
    }
    break;
  //......................................................................
  case STATE_STOP:  // Waiting; Measuring Gap
    if(irdata == MARK)  irparams.timer = 0 ;   // Reset gap timer
    break;
  //......................................................................
  case STATE_OVERFLOW:  // Flag up a read overflow; Stop the State Machine
    irparams.overflow = true;
    irparams.rcvstate = STATE_STOP;
    break;
  }
}

/*                          */


bool  IRrecv::decode(decode_results *results)
{
  results->rawbuf   = irparams.rawbuf;
  results->rawlen   = irparams.rawlen;
  results->overflow = irparams.overflow;
  if(irparams.rcvstate != STATE_STOP)
    return false ;
#if DECODE_NEC
  if(decodeNEC(results))
    return true ;
#endif
  // decodeHash returns a hash on any input.
  // Thus, it needs to be last in the list.
  // If you add any decodes, add them before this.
  if(decodeHash(results))
    return true ;
  // Throw away and start over
  resume();
  return false;
}

bool  IRrecv::isIdle()
{
  return (irparams.rcvstate == STATE_IDLE
          || irparams.rcvstate == STATE_STOP) ? true : false;
}

void  IRrecv::resume()
{
  irparams.rcvstate = STATE_IDLE;
  irparams.rawlen = 0;
}

bool  IRrecv::decodeHash(decode_results *results)
{
#define FNV_PRIME_32 16777619
#define FNV_BASIS_32 2166136261U
  uint32_t hash = FNV_BASIS_32;
  // Require at least 6 samples to prevent triggering on noise
  if(results->rawlen < 6)  return false ;
  for(int i = 1; (i + 2) < results->rawlen;  i++)
  {
    int value =  compare(results->rawbuf[i], results->rawbuf[i+2]);
    // Add value into the hash
    hash = (hash * FNV_PRIME_32) ^ value;
  }
  results->value       = hash;
  results->bits        = 32;
  results->decode_type = UNKNOWN;
  return true;
}

int  IRrecv::compare(uint32_t oldval, uint32_t newval)
{
  //if      (newval < oldval * .8)  return 0 ;
  //else if (oldval < newval * .8)  return 2 ;
  //else                            return 1 ;
  if(newval * 5 < oldval * 4)  return 0 ;
  else if(oldval * 5 < newval * 4)  return 2 ;
  else                               return 1 ;
}
