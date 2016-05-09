/*
  Based On IRremote Arduino Library: https://github.com/z3t0/Arduino-IRremote/
*/

#ifndef _IRREMOTE_H_
#define _IRREMOTE_H_

#include "stdint.h"
#include "ir_configs.h"

typedef enum
{
  UNKNOWN      = -1,
  UNUSED       =  0,
  RC5,
  RC6,
  NEC,
  SONY,
  PANASONIC,
  JVC,
  SAMSUNG,
  WHYNTER,
  AIWA_RC_T501,
  LG,
  SANYO,
  MITSUBISHI,
  DISH,
  SHARP,
  DENON,
  PRONTO,
} decode_type_t;

class IrState
{
public:
  enum
  {
    STATE_IDLE     =   2,
    STATE_MARK     =   3,
    STATE_SPACE    =   4,
    STATE_STOP     =   5,
    STATE_OVERFLOW =   6,
  };

  struct irparams_t
  {
    // The fields are ordered to reduce memory over caused by struct-padding
    uint8_t       rcvstate;        // State Machine state
    uint8_t       recvpin;         // Pin connected to IR data from detector
    uint8_t       blinkpin;
    uint8_t       blinkflag;       // true -> enable blinking of pin on IR processing
    uint8_t       rawlen;          // counter of entries in rawbuf
    uint32_t      timer;           // State timer, counts 50uS ticks.
    uint32_t      rawbuf[RAWBUF];  // raw data
    uint8_t       overflow;        // Raw buffer overflow occurred
  } irparams;

  uint8_t (*readIrPin)(void);
  void (*SetIrLedOn)(void);
  void (*SetIrLedOff)(void);
  void (*TimerReset)(void);
  void (*TimerSet)(uint32_t frequency);

public:
  //IrState();
  //~IrState();
  bool match(int measured,  int desired);
  bool matchMark(int measured_ticks,  int desired_us);
  bool matchSpace(int measured_ticks,  int desired_us);
  void timerInit();
  void timerIrqHandle();

};

class decode_results
{
public:
  decode_type_t          decode_type;  // UNKNOWN, NEC, SONY, RC5, ...
  uint32_t               address;      // Used by Panasonic & Sharp [16-bits]
  uint32_t               value;        // Decoded value [max 32-bits]
  uint32_t               bits;         // Number of bits in decoded value
  uint32_t               *rawbuf;      // Raw intervals in 50uS ticks
  uint32_t               rawlen;       // Number of records in rawbuf
  uint32_t               overflow;     // true iff IR raw code too long
};

class IRrecv: public IrState
{
public:
  bool decode(decode_results *results);
  bool isIdle();
  void resume();
  bool decodeHash(decode_results *results);
  int compare(uint32_t oldval, uint32_t newval);
  //
  bool decodeNEC(decode_results *results);
  //...
};

extern IRrecv irrecv;

#endif
