/*
  Based On IRremote Arduino Library: https://github.com/z3t0/Arduino-IRremote/
*/

#ifndef _IR_CONFIGS_H_
#define _IR_CONFIGS_H_

//------------------------------------------------------------------------------
// Information for the Interrupt Service Routine
//
#define RAWBUF  101  // Maximum length of raw duration buffer

//------------------------------------------------------------------------------
// Pulse parms are ((X*50)-100) for the Mark and ((X*50)+100) for the Space.
// First MARK is the one after the long gap
// Pulse parameters in uSec
//

// Due to sensor lag, when received, Marks  tend to be 100us too long and
//                                   Spaces tend to be 100us too short
#define MARK_EXCESS    100

// microseconds per clock interrupt tick
#define USECPERTICK    50

// Upper and Lower percentage tolerances in measurements
#define TOLERANCE       25
#define LTOL            (1.0 - (TOLERANCE/100.))
#define UTOL            (1.0 + (TOLERANCE/100.))

// Minimum gap between IR transmissions
#define _GAP            5000
#define GAP_TICKS       (_GAP/USECPERTICK)

#define TICKS_LOW(us)   ((int)(((us)*LTOL/USECPERTICK)))
#define TICKS_HIGH(us)  ((int)(((us)*UTOL/USECPERTICK + 1)))
//------------------------------------------------------------------------------
// IR detector output is active low
//
#define MARK   0
#define SPACE  1

#define REPEAT (0xFFFFFFFF)

//------------------------------------------------------------------------------
// Supported IR protocols
// Each protocol you include costs memory and, during decode, costs time
// Disable (set to 0) all the protocols you do not need/want!
//
#define DECODE_NEC           1


#endif
