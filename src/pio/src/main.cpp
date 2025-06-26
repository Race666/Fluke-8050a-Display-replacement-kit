#include <Arduino.h>
/*
  TM1637 LED display implementation for Fluke 8050A multimeter

  Honer to https://www.vondervotteimittiss.de/post/a-new-led-display-for-a-fluke-8050a-multimeter
  and https://github.com/jasonacox/TM1637TinyDisplay

  20250312 Michael Albert michael@albert-hetzles.de

  TM1637TinyDisplay => Cannot add by add library function. Not the latest version available in Arduino
  TM1637TinyDisplay Installed by
  cd src
  git clone https://github.com/jasonacox/TM1637TinyDisplay
  delete all expect TM1637TinyDisplay6.*


   avrdude location: 
   & C:\users\$ENV:USERNAME\.platformio\packages\tool-avrdude\avrdude.exe -c usbtiny -p m328p -C ../etc/avrdude.conf -U lfuse:w:0x62:m -U hfuse:w:0xDE:m -U efuse:w:0x05:m

   But use https://github.com/avrdudes/avrdude/releases
   Arduino atmega328p default fuses: -U lfuse:w:0xFF:m -U hfuse:w:0xDE:m -U efuse:w:0x05:m
   D:\tools\Avrdude\avrdude.exe -c usbtiny -p atmega328p -v

   Internal Clock 8Mhz
   D:\tools\Avrdude\avrdude.exe -c usbtiny -p atmega328p -U lfuse:w:0x62:m -U hfuse:w:0xDE:m -U efuse:w:0x05:m -U flash:w:.pio\build\ATmega328P\firmware.hex

   read FUSES
   Fuse calc
   https://eleccelerator.com/fusecalc/fusecalc.php?chip=atmega328p
   D:\tools\Avrdude\avrdude.exe -c usbtiny -p atmega328p -v -U lfuse:r:-:i -U hfuse:r:-:i


   Fluke J1 Connector
   J1 Pin

  Function

    2    -5V
    3    GND
    6     Batt Vref
    10   ST4
    11   ST3
    12   ST2
    13   ST1
    14   Z
    15   Y
    16   X
    17   W
    18   ST0
    20   HV
    22   DP
*/


/* Mapping Fluke J1 => Atmega328p 
  J1     ATMEGA      ArduinoPIN  AtmelPhysical PIN (DIP28) AtmelPhysical PIN (DIP28)

  10  -> PortD Pin0  0           2
  11  -> PortD Pin1  1           3
  12  -> PortD Pin2  2           4
  13  -> PortD Pin3  3           5
  14  -> PortD Pin4  4           6
  15  -> PortD Pin5  5           11
  16  -> PortD Pin6  6           12
  17  -> PortD Pin7  7           13
  18  -> PortB Pin0  8           14
  20  -> PortB Pin1  9           15
  22  -> PortB Pin2  10          16
*/


#include <TM1637TinyDisplay6.h>

// PIN ASSIGNMENTS https://docs.arduino.cc/retired/hacking/software/PortManipulation/
// https://docs.arduino.cc/retired/hacking/hardware/ATMEGA328P/
// Digital Pins
// PortD 0-7 Pin 0-7
// PortB 0-7 Pin 8-13
// PortC 0-5 Pin 14-19
// 8050A strobe lines
// Fluke Digit Count
//             _    _    _    _
//         |  | |  | |  | |  | | 
//     +/-     -    -    -    -
//         |  | |  | |  | |  | | 
//             ͞     ͞     ͞     ͞        
// Digit  0    1    2    3    4    
// Digit Select Input Ports => Interrupt for reading Datalines
#define PORTB_0 8
#define PIN_DIGIT0_ST0 PORTB_0
#define PIN_DIGIT1_ST1 PORTD3
#define PIN_DIGIT2_ST2 PORTD2
#define PIN_DIGIT3_ST3 PORTD1
#define PIN_DIGIT4_ST4 PORTD0


// 8050A Data/Scancode lines Input Pins
#define PIN_DATA_Z 4             // PORTD4 / Phys PIN 6 / Digit Value 2^0 Digit Bit 0
#define PIN_DATA_Y 5             // PORTD5 / Phys PIN 11 / Digit Value 2^1 Digit Bit 1
#define PIN_DATA_X 6             // PORTD6 / Phys PIN 12 / Digit Value 2^2 Digit Bit 2
#define PIN_DATA_W 7             // PORTD7 / Phys PIN 13 / Digit Value 2^3 Digit Bit 3
#define PIN_DATA_HV 9            // PORTB1 / Phys PIN 15
#define PIN_PORTB_HV PORTB1 // PORTB1 / Phys PIN 15
#define PIN_DATA_DP 10           // PORTB2 / Phys PIN 16
#define PIN_PORTB_DP PORTB2 // PORTB2 / Phys PIN 16

// CONSTANTS
// strobe lines (digit/number position) ST0 left (+/- 1) to right ST4
/*  
  example: -1.4567
  scanCodes[ST0] contains
  -1         .4   5   6   7
  ST0       ST1 ST2 ST3 ST4

  The point is to the left of a Digit means it is in the scan code of the next Digit => here ST1, Bit 
  ST5 contains

*/
// Data Bits
#define DATA_SCANCODE_Z_BIT0 0 // 2^0 Digit Bit 0
#define DATA_SCANCODE_Y_BIT1 1 // 2^1 Digit Bit 1
#define DATA_SCANCODE_X_BIT2 2 // 2^2 Digit Bit 2
#define DATA_SCANCODE_W_BIT3 3 // 2^3 Digit Bit 3
// #define DATA_SCANCODE_HV 4
#define DATA_SCANCODE_DP 5

#define SCANCODE_DB DATA_SCANCODE_Y_BIT1
#define SCANCODE_1 DATA_SCANCODE_Z_BIT0
#define SCANCODE_PLUS DATA_SCANCODE_W_BIT3
#define SCANCODE_MINUS DATA_SCANCODE_X_BIT2
#define SCANCODE_REL DATA_SCANCODE_DP

// Output LED Pins HV (HighValtage), db (Decibel), Rel (reltive measure)
#define PIN_PORTC_3 17    // PORTC3 Phy. PIN 26
#define PIN_LED_HV PIN_PORTC_3
#define PIN_PORTC_2 16    // PORTC2 Phy. PIN 25
#define PIN_LED_DB PIN_PORTC_2
#define PIN_PORTC_1 15    // PORTC1 Phy. PIN 24
#define PIN_LED_REL PIN_PORTC_1
// NN (not named LED) => currently not used
#define PIN_PORTC_0 14    // PORTC1 Phy. PIN 23
#define PIN_LED_NN PIN_PORTC_0


// Global variables

// Define TM1637 Pins => PortB Pin3 und 4 => 11, 12
#define PIN_CLK 18    // Atmega phys. PORTC4 PIN27
#define PIN_DIO 19    // Atmega phys. PORTC5 PIN28

// TM1637TinyDisplay6 display(PIN_CLK, PIN_DIO,0,0);
TM1637TinyDisplay6 display(PIN_CLK, PIN_DIO,0,0);

// number of digits in the display
#define NUM_DIGITS 5
// Point Bit 7 of the Scan code of a digit
// #define LED_SEGMENT_DP B10000000
// 5 Digits. Left to right
volatile uint8_t digits[ NUM_DIGITS ] = {
  0,
  0,
  0,
  0,
  0
}; 

// Measure +/-
volatile bool NegativeMeasure=false;
// HighVoltage
volatile bool HighVoltageRockNRoll=false;
// Decibalmeasure
volatile bool DecibalMeasure=false;
// Relative measure
volatile bool Relativemeasure=false;
String sOutputString="";

// Decimal Point Position
volatile uint8_t iDecimalPointPosition=0;

// update display flag (ISR)
volatile boolean displayUpdate = false;


// Digit 6 => 
// Pin Change Interrupts
// Enable pin change interrupts in the PCICR Pin Change Interrupt Control Register
// by setting the appropriate PCIEn Pin Change Interrupt Enable bit(s)
//   PCIE0 enables Arduino pins 8-13, mask bits PCINT0-5
//   PCIE1 enables Arduino pins A0-A5, mask bits PCINT8-13
//   PCIE2 enables Arduino pins 0-7, mask bits PCINT16-23
// Set the desired pin(s) in the appropriate PCMSKn Pin Change Mask Register
//   PCMSK0 handles bits PCINT0-5 PortB
//   PCMSK1 handles bits PCINT8-13 PortC
//   PCMSK2 handles bits PCINT16-23 PortD
//
// Control register enable interrupt enable bit => Port B and D
#define STROBE_INTERRUPT_ENABLE _BV( PCIE0 ) | _BV( PCIE2 );
// pin change mask register
// https://arduino.stackexchange.com/questions/79690/assign-an-interrupt-to-any-pin-of-the-atmega328-microcontroller
#define STROBE_INTERRUPT_MASK_REGISTER_PORTB PCMSK0
#define STROBE_INTERRUPT_MASK_REGISTER_PORTD PCMSK2
// Interrupt vectors/functions
// The ISR is called on the vector PCINTn_vect, corresponding PCIEn.
// ISR for PortB
#define DIGIT0_STROBE_INTERRUPT_VECTOR_PORTB PCINT0_vect 
// ISR for PortD
#define DIGIT1_TO_4_STROBE_INTERRUPT_VECTOR_PORTD PCINT2_vect



// FUNCTIONS
// pin change interrupt service routine
// Interrupt on Digit/Strobe 0  PortB0
ISR( DIGIT0_STROBE_INTERRUPT_VECTOR_PORTB )
{
  // Read IO Ports
  uint8_t ReadPortD= PIND ;
  uint8_t ReadPortB= PINB ;
  if(digitalRead(PIN_DIGIT0_ST0))
  {
    // The upper 4 Bit of PortD. Die lower 4 are the Digit selectors and not necessary here
      ReadPortD=(ReadPortD & 0xf0);
    // Right shift, to shift out Strobe 1-4 from PortD
    ReadPortD=(ReadPortD>>4);
    // Is voltage/current measure and negativ mumber?
    if (ReadPortD & (1<< SCANCODE_PLUS) && ReadPortD & (1<< SCANCODE_MINUS))
      NegativeMeasure=false;
    // resistor measure
    else if((ReadPortD & (1<< SCANCODE_PLUS))==0)  
      NegativeMeasure=false;
    else
      NegativeMeasure=true;
    // Leading 1 for example 12.345
    if(ReadPortD & (1<<SCANCODE_1))
    {
      digits[0]=1;
    } 
    else
      digits[0]=0;
    // Decibal measure?   
    if(ReadPortD & (1<<SCANCODE_DB))
      DecibalMeasure=true;
    else
      DecibalMeasure=false;      
    // HighVoltage  
    if(ReadPortB & (1<<PIN_PORTB_HV))
      HighVoltageRockNRoll=true;
    else
      HighVoltageRockNRoll=false;
    // Relative measure    
    if(ReadPortB & (1<<PIN_PORTB_DP))
       Relativemeasure=true;
    else
      Relativemeasure=false;       
  }  
}

// Interrupt for Digit/Strobe 1 - 4 selector
ISR( DIGIT1_TO_4_STROBE_INTERRUPT_VECTOR_PORTD )
{
  uint8_t DigitPosition=0;
  // Read IO Ports
  uint8_t ReadPortD= PIND ;
  uint8_t ReadPortB= PINB ;
  // Digit 1?
  if(ReadPortD & (1<<PIN_DIGIT1_ST1))
  {
    DigitPosition=1;
    digits[1] = (ReadPortD & B11110000) >>4; 
  }
  // Digit 2?
  else if(ReadPortD & (1<<PIN_DIGIT2_ST2))
  {
    DigitPosition=2;
    digits[2] = (ReadPortD & B11110000) >>4; 
  }
  // Digit 3?
  else if(ReadPortD &  (1<<PIN_DIGIT3_ST3))
  {
    DigitPosition=3;
    digits[3] = (ReadPortD & B11110000) >>4; 
  }
  // Digit 4?
  else if(ReadPortD & (1<<PIN_DIGIT4_ST4))
  {
    DigitPosition=4;
    digits[4] = (ReadPortD & B11110000) >>4; 
  }
  // Decimal Point set? The decimal point bit is set on at the digit after that point
  if(ReadPortB & (1<<PORTB2) && DigitPosition>=1 && DigitPosition<=4)
  {
    iDecimalPointPosition=5-DigitPosition;
  }
  // Last digit (4) read => Update display
  if(DigitPosition==4)
  {
    displayUpdate=true;
  }
}

void formatOutput()
{
  if(NegativeMeasure)
    sOutputString="-";
  else
    sOutputString=" "; 
  if(digits[0]==1)
    sOutputString+="1"; 
  else
    sOutputString+=" ";   
  // Digit 1-4 to output string  
  for ( uint8_t i = 1; i <= 4; i++ )
  {
    // Special case Resistormeasure, 7 Segment shows no number
    if(digits[i]==0xf)
      sOutputString+=String(" ");
    else
      // Number to outputstring
      sOutputString+=String(digits[i], DEC);
  }
  // Special case not valid measure range
  if(digits[0]==0 && digits[1]==0xf && digits[2]==0xf && digits[3]==0xf && digits[4]==0xf)
  {
    sOutputString=String("------");
    iDecimalPointPosition=0;
  }
  return;
}

// HV - HighVoltage
// db - decibel measure
// Rel- Relative measure
void setStateLEDs()
{
  // indicator LEDs dB, HV, REL
  digitalWrite(PIN_LED_DB, DecibalMeasure );
  // digitalWrite(PIN_LED_HV, HighVoltageRockNRoll); => Switched to blinking
  digitalWrite(PIN_LED_REL, Relativemeasure);
}


void updateDisplay()
{
  // displayUpdate flag is set in the ISR
  if ( displayUpdate )
  {
    displayUpdate = false;
    formatOutput();
    if(iDecimalPointPosition>=1)
      // With decimal point
      display.showString(sOutputString.c_str(),6,0,(1<<(iDecimalPointPosition+2)));
    else
      // No decimal point
      display.showString(sOutputString.c_str(),6,0);
    setStateLEDs();
    iDecimalPointPosition=0;
    // writeDigits();
  }
}  

// Blinking HV LED
void ChangeBlinkLEDState()
{
  if(HighVoltageRockNRoll)
    digitalWrite(PIN_LED_HV, !digitalRead(PIN_LED_HV));
  else
    digitalWrite(PIN_LED_HV,0);  
}

void setup() {
  // disable ADC (to reduce power)
  ADCSRA = 0; 
  // Input DIGIT 0 - 4 set/select
  pinMode(PIN_DIGIT0_ST0,INPUT );
  pinMode(PIN_DIGIT1_ST1,INPUT );
  pinMode(PIN_DIGIT2_ST2,INPUT );
  pinMode(PIN_DIGIT3_ST3,INPUT );
  pinMode(PIN_DIGIT4_ST4,INPUT );
  // Data Pins
  pinMode(PIN_DATA_Z,INPUT );
  pinMode(PIN_DATA_Y,INPUT );
  pinMode(PIN_DATA_X,INPUT );
  pinMode(PIN_DATA_W,INPUT );
  pinMode(PIN_DATA_HV,INPUT );
  pinMode(PIN_DATA_DP,INPUT );
  // LED Pins
  pinMode(PIN_LED_HV,OUTPUT);
  pinMode(PIN_LED_DB,OUTPUT);
  pinMode(PIN_LED_REL,OUTPUT);
  pinMode(PIN_LED_NN,OUTPUT);
  // Initialize Display => Test
  display.begin();
  // display.showString("000000");
  display.showString(" FLUHE");
  digitalWrite(PIN_LED_HV, HIGH);
  delay(50);
  // display.showString("111111");
  display.showString(" 8050A");
  digitalWrite(PIN_LED_HV, LOW);
  digitalWrite(PIN_LED_REL, HIGH);
  delay(50);
  // display.showString("222222");
  display.showString(" V1.00");
  digitalWrite(PIN_LED_REL, LOW);  
  digitalWrite(PIN_LED_DB, HIGH );
  delay(50);
  digitalWrite(PIN_LED_DB, LOW); 
  digitalWrite(PIN_LED_NN, HIGH); 
  delay(50);
  digitalWrite(PIN_LED_NN, LOW); 
  cli();
  // st = ST0; // initialize the strobe counter
  PCICR = STROBE_INTERRUPT_ENABLE;
  // STROBE_INTERRUPT_MASK_REGISTER_PORTB = StrobePin2PCINT[ st ];
  // Interrupt on Digit select / Strobe lines ST1 - S4
  STROBE_INTERRUPT_MASK_REGISTER_PORTD = (1<<PIN_DIGIT1_ST1) | (1<<PIN_DIGIT2_ST2) | (1<<PIN_DIGIT3_ST3) | (1<<PIN_DIGIT4_ST4);
  // Interrupt on Digit 0 select / Strobe lines ST0
  STROBE_INTERRUPT_MASK_REGISTER_PORTB = (1<<PORTB0);
  sei();
}
unsigned long BlinkingLastMills=0;
uint32_t BlinkLEDChangeEveryMilliseconds=25;
void loop() 
{
  // catch millis() overflow
  if(millis() < BlinkingLastMills)
    BlinkingLastMills=millis();

  if(BlinkingLastMills <= millis()-BlinkLEDChangeEveryMilliseconds)
  {
    ChangeBlinkLEDState();
    BlinkingLastMills=millis();
  }
  updateDisplay();
}

