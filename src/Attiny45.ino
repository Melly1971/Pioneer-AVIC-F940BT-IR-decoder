/*
   IR NEC protocol reader (38kHz, TSOP1738) and 8 pin out for Pioneer AVIC-F940BT car stereo and Pioneer remote controller.
   ATtiny45, MCU Settings: FUSE_L=0xE2, FUSE_H=0xDF, FUSE_E=0xFF, F_CPU=8MHz internal 14CK+65 ms
*/

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// in , Vss, Vee to GND
// out 0 1k2 resistor connected to tip of 3.5mm jack, B000 
// out 1 3k3 resistor connected to tip of 3.5mm jack, B001 
// out 2 5k6 resistor connected to tip of 3.5mm jack, B010 
// out 3 8k2 resistor connected to tip of 3.5mm jack, B011 
// out 4 12k resistor connected to tip of 3.5mm jack, B100 
// out 5 15k resistor connected to tip of 3.5mm jack, B101 
// out 6 24k resistor connected to tip of 3.5mm jack, B110 
// out 7 68k resistor connected to tip of 3.5mm jack, B111 

#define APort4051 PB5    //  BXX1
#define BPort4051 PB3    //  BX1X
#define CPort4051 PB4    //  B1XX
#define ShiftPort PB1    // connected to ring of 3.5mm jack and GND   
#define Inhibit4051 PB0  // PB0 inhibit CD4051

#define IrInPin PB2  // TSOP1338 out to pin PB2 INT0

#define IrSuccess (0)
#define IrError (1)

#define Low (0)
#define High (1)

#define IrEventIdle (0)
#define IrEventInit (1)
#define IrEventEnd (2)
#define IrEventProcess (3)

#define IrProtocolEventInit (0)
#define IrProtocolEventData (1)
#define IrProtocolEventEnd (2)
#define IrProtocolEventHook (3)

// NEC IR command and address of Pioneer remote controller
#define CommandRepeat (0x19)
#define AddressRepeat (0xAD)
#define CommandVolumeMinus (0x0B)
#define AddressVolumeMinus (0xAD)
#define CommandVolumePlus (0x0A)
#define AddressVolumePlus (0xAD)
#define CommandBandEsc (0x12)
#define AddressBandEsc (0xAD)
#define CommandMute (0x30)
#define AddressMute (0xAF)
#define CommandFunction (0x67)
#define AddressFunction (0xAF)
#define CommandAudio (0x0D)
#define AddressAudio (0xAD)
#define CommandSrc (0x1A)
#define AddressSrc (0xAD)
#define CommandPause (0x58)
#define AddressPause (0xAD)
#define CommandDisplay (0x6D)
#define AddressDisplay (0xAF)
#define CommandLeft (0x42)
#define AddressLeft (0xAD)
#define CommandRight (0x43)
#define AddressRight (0xAD)
#define CommandUp (0x40)
#define AddressUp (0xAD)
#define CommandDown (0x41)
#define AddressDown (0xAD)
#define CommandEnter (0x20)
#define AddressEnter (0xAF)

// timer to turn out off
volatile uint16_t OffCounter = { 0 };
volatile uint16_t OffInterval = { 10000 };  //delay in 2*38.222kHz pulse, 10000 =131 ms
static boolean OutPortFlag = false;

// timer for NEC protocol
volatile uint16_t IrTimeout{ 0 };
volatile uint16_t IrCounter{ 0 };
volatile uint32_t IrRawdata{ 0 };

// variables for NEC protocol
uint8_t IrEvent{ 0 };
uint8_t IrProtocolEvent{ 0 };
uint8_t IrIndex{ 0 };
uint32_t IrData{ 0 };
uint8_t IrAddress{ 0 };
uint8_t IrCommand{ 0 };

// from raw 32 bit to address 8 bit and command 8 bit
int IrRead(uint8_t *Address, uint8_t *Command) {
  if (!IrRawdata)
    return IrError;
  *Address = IrRawdata;
  *Command = IrRawdata >> 16;
  IrRawdata = 0;
  return IrSuccess;
}


int IrNecProcess(uint16_t counter, uint8_t value) {
  int8_t retval = IrError;

  switch (IrProtocolEvent) {
    case IrProtocolEventInit:
      /* expecting a space */
      if (value == High) {
        if (counter > 330 && counter < 360) {
          /* a 4.5ms space for regular transmition of NEC Code; counter => 0.0045/(1.0/38222.0) * 2 = 344 (+/- 15) */
          IrProtocolEvent = IrProtocolEventData;
          IrData = IrIndex = 0;
          retval = IrSuccess;
        } else if (counter > 155 && counter < 185) {
          /* a 2.25ms space for NEC Code repeat; counter => 0.00225/(1.0/38222.0) * 2 = 172 (+/- 15) */
          IrProtocolEvent = IrProtocolEventEnd;
          retval = IrSuccess;
        }
      }
      break;
    case IrProtocolEventData:
      /* Reading 4 octets (32bits) of data:
           1) the 8-bit address for the receiving device
           2) the 8-bit logical inverse of the address
           3) the 8-bit command
           4) the 8-bit logical inverse of the command
          Logical '0' ??? a 562.5??s pulse burst followed by a 562.5??s
           (<90 IR counter cycles) space, with a total transmit time of 1.125ms
          Logical '1' ??? a 562.5??s pulse burst followed by a 1.6875ms
           (>=90 IR counter cycles) space, with a total transmit time of 2.25ms */
      if (IrIndex < 32) {
        if (value == High) {
          IrData |= ((uint32_t)((counter < 90) ? 0 : 1) << IrIndex++);
          if (IrIndex == 32) {
            IrProtocolEvent = IrProtocolEventHook;
          }
        }
        retval = IrSuccess;
      }
      break;
    case IrProtocolEventHook:
      // expecting a final 562.5??s pulse burst to signify the end of message transmission
      if (value == Low) {
        IrProtocolEvent = IrProtocolEventEnd;
        retval = IrSuccess;
      }
      break;
    case IrProtocolEventEnd:
      // copying data to volatile variable; raw data is ready
      IrRawdata = IrData;
      break;
    default:
      break;
  }
  return retval;
}

void IrProcess() {
  uint8_t value;
  uint16_t counter;
  // load IrCounter value to local variable, then reset counter
  counter = IrCounter;
  IrCounter = 0;
  // read IrInPin digital value (NOTE: logical inverse value = value ^ 1 due to sensor used)
  value = (PINB & (1 << IrInPin)) > 0 ? Low : High;  // Low : High;

  switch (IrEvent) {
    case IrEventIdle:  // awaiting for an initial signal
      if (value == High) {
        IrEvent = IrEventInit;
      }
      break;
    case IrEventInit:  // consume leading pulse burst
      if (value == Low) {
        if (counter > 655 && counter < 815) {
          // a 9ms leading pulse burst, NEC Infrared Transmission Protocol detected, counter = 0.009/(1.0/38222.) * 2 = 343.998 * 2 = 686 (+/- 30)
          IrEvent = IrEventProcess;
          IrProtocolEvent = IrProtocolEventInit;
          IrTimeout = 5000;  // 5000 clock at 38kHz > 62.5ms total time NEC protocol - was 7400
        } else {
          IrEvent = IrEventEnd;
        }
      } else {
        IrEvent = IrEventEnd;
      }
      break;
    case IrEventProcess:  // read and decode NEC Protocol data
      if (IrNecProcess(counter, value))
        IrEvent = IrEventEnd;
      break;

    case IrEventEnd:  // clear timeout and set idle mode
      IrEvent = IrEventIdle;
      IrTimeout = 0;
      break;
    default:
      break;
  }
}

void OutPort(uint8_t OutPortBit)  // out portB to 4051 - shift,C,B,A
{
  if ((OutPortBit & B00001000) >> 3) PORTB |= _BV(ShiftPort); // shift
  if ((OutPortBit & B00000100) >> 2) PORTB |= _BV(CPort4051); // C on CD4051
  if ((OutPortBit & B00000010) >> 1) PORTB |= _BV(BPort4051); // B on CD4051
  if ( OutPortBit & B00000001)       PORTB |= _BV(APort4051); // A on CD4051
   
  PORTB &= ~(_BV(Inhibit4051)); // inhibit CD4051 to LOW

  OutPortFlag = true;
  OffCounter = 0;
}

ISR(INT0_vect) {
  IrProcess();
}

ISR(TIMER0_COMPA_vect) {
  if (IrCounter++ > 10000)
    IrEvent = IrEventIdle;

  if (IrTimeout && --IrTimeout == 0)
    IrEvent = IrEventIdle;
  OffCounter++;
}

void setup() {
  
  DDRB = (B111011); // PORTB 0,1,3,4,5 output, 2 input  
  PORTB |= _BV(Inhibit4051); // port inhibit CD4051 HIGH
  PORTB &= ~(_BV(APort4051) | _BV(BPort4051) | _BV(CPort4051) | _BV(ShiftPort) | _BV(IrInPin)); // port A, B, C, shift and IrInPin LOW

  TCNT0 = 0;            // Count up from 0
  TCCR0A = 2 << WGM00;  // CTC mode

  TCCR0B = (1 << CS00);  // Set prescaler to /1

  GTCCR |= 1 << PSR0;  // Reset prescaler
  OCR0A = 104;          // set OCR0n to get ~38.222kHz timer frequency, 104 for 8MHz (104=(4MHz/38kHz)-1)
  TIFR = 1 << OCF0A;    // Clear output compare interrupt flag

  TIMSK |= 1 << OCIE0A;  // Enable output compare interrupt

  GIMSK |= _BV(INT0);    // enable INT0 interrupt handler
  MCUCR &= ~_BV(ISC01);  // trigger INTO interrupt on raising
  MCUCR |= _BV(ISC00);   // and falling edge
  sei();                 // enable global interrupts
}

void loop() {

  if ((OutPortFlag == true) && (OffCounter >= OffInterval))  // if any output to CD4051 is high and time (OffInterval) has passed, switch all outputs to low and inhibit to high
  {
    PORTB |= _BV(Inhibit4051); // inhibit CD4051 to HIGH
    PORTB &= ~(_BV(APort4051) | _BV(BPort4051) | _BV(CPort4051) | _BV(ShiftPort)); // A, B, C and Shift to LOW
    
    OutPortFlag = false;
  }

  // if IR read was a success then outup
  if (IrRead(&IrAddress, &IrCommand) == IrSuccess) {

    // on address and command out shift, C, B, A

    if (IrAddress == AddressVolumeMinus && IrCommand == CommandVolumeMinus)  // volume - B110
    {
      OutPort(B00000110); 
    }

    if (IrAddress == AddressVolumePlus && IrCommand == CommandVolumePlus)  // volume + B101
    {
      OutPort(B00000101);
    }

    if (IrAddress == AddressBandEsc && IrCommand == CommandBandEsc)  // band /esc B111
    {
      OutPort(B00000111);
    }

    if (IrAddress == AddressMute && IrCommand == CommandMute)  // att sound B001
    {
      OutPort(B00000001);
    }

    if (IrAddress == AddressFunction && IrCommand == CommandFunction)  // phone menu B000 + shift
    {
      OutPort(B00001000);
    }

    if (IrAddress == AddressAudio && IrCommand == CommandAudio)  // phone off B010 + shift
    {
      OutPort(B00001010);
    }

    if (IrAddress == AddressSrc && IrCommand == CommandSrc)  // src / 2sec OFF B000
    {
      OutPort(B00000000);
    }

    if (IrAddress == AddressPause && IrCommand == CommandPause)  // mute B111 + shift
    {
      OutPort(B00001111);
    }

    if (IrAddress == AddressDisplay && IrCommand == CommandDisplay)  // display B010
    {
      OutPort(B00000010);
    }

    if (IrAddress == AddressLeft && IrCommand == CommandLeft)  // left B100
    {
      OutPort(B00000100);
    }

    if (IrAddress == AddressRight && IrCommand == CommandRight)  // right B011
    {
      OutPort(B00000011);
    }

    if (IrAddress == AddressUp && IrCommand == CommandUp)  // up B100 + shift
    {
      OutPort(B00001100);
    }

    if (IrAddress == AddressDown && IrCommand == CommandDown)  // down B011 + shift
    {
      OutPort(B00001011);
    }

    if (IrAddress == AddressEnter && IrCommand == CommandEnter)  // shift att sound  B110 + shift
    {
      OutPort(B00001110);
    }

    if (IrAddress == AddressRepeat && IrCommand == CommandRepeat)  // ****
    {
    }
  }
}
