// Special I2C parallel bus
// Datum:   02.12.2020
// Version: V0.0.0

/* PortRegisters

Pin  | Port | Bit
10  | 7    | 0
12  | 7    | 1
11  | 7    | 2
13  | 7    | 3    led
6   | 7    | 10
9   | 7    | 11
32  | 7    | 12
8   | 7    | 16
7   | 7    | 17
36  | 7    | 18
37  | 7    | 19
35  | 7    | 28
34  | 7    | 29

  
   
 */
#define CUBE_4
//#define CUBE_8 
 
//#define I2C_SPPED_FAST
#define I2C_USE_PORTMANIPULATION
#define DISABLE_DEBUG_PINS
#define I2C_IGNORE_ACK

#define USE_TIMER_ONE
#define USE_TIMERTOOL

#include "TeensyTimerTool.h"
using namespace TeensyTimerTool;



// Create an IntervalTimer object 
#ifndef USE_TIMER_ONE
#ifdef USE_TIMERTOOL
Timer         busTimer1;
#else
IntervalTimer busTimer;
#endif
#endif
//const unsigned int busTimeInterval = 1000; // us

// Bus stuff
// I2C R/W
#define I2C_READ                       1
#define I2C_WRITE                      0

#define clk_Bit CORE_PIN10_BIT
#define sda_0_Bit CORE_PIN12_BIT    // red 1
#define sda_1_Bit CORE_PIN11_BIT    // red 2
#define sda_2_Bit CORE_PIN13_BIT    // red 3
#define sda_3_Bit CORE_PIN6_BIT     // red 4
#define sda_4_Bit CORE_PIN9_BIT     // green 1
#define sda_5_Bit CORE_PIN32_BIT    // green 2
#define sda_6_Bit CORE_PIN8_BIT     // green 3
#define sda_7_Bit CORE_PIN7_BIT     // green 4
#define sda_8_Bit CORE_PIN36_BIT    // blue 1
#define sda_9_Bit CORE_PIN37_BIT    // blue 2
#define sda_10_Bit CORE_PIN35_BIT   // blue 3
#define sda_11_Bit CORE_PIN34_BIT   // blue 4

const byte redPortOffset   = 0;
const byte greenPortOffset = 4;
const byte bluePortOffset  = 8;

void interruptFunc();

// I2C Pins
const byte dataPorts  = 12;
//  CLK
const byte i2c_scl    = 10;
//  SDA's
const byte i2c_sda_0  = 12;
const byte i2c_sda_1  = 11;
const byte i2c_sda_2  = 13;
const byte i2c_sda_3  = 6;
const byte i2c_sda_4  = 9;
const byte i2c_sda_5  = 32;
const byte i2c_sda_6  = 8;
const byte i2c_sda_7  = 7;
const byte i2c_sda_8  = 36;
const byte i2c_sda_9  = 37;
const byte i2c_sda_10  = 35;
const byte i2c_sda_11  = 34;

const byte resetPin = 26;

volatile  byte bussy = 0;
const byte bufferSize = 50;

byte sendingData[dataPorts][bufferSize];
byte bitIndex = 7;
byte dataPosIndex = 0;
byte sendingDataSize = 0;
byte clkState = 1;

byte beginCondDone = 0;
byte endCondDone = 0;

byte busStep = 0;

void beginTransmission(byte address);
void beginTransmission(byte *address);
void write(byte data);
void write(byte *data);
void endTransmission();

#ifndef DISABLE_DEBUG_PINS
byte debugPin = 2;
byte debugPin2 = 5;
unsigned int dbg2Counter = 0;
byte errorPin = 3;
byte debugBytePin = 4;
#endif

// Layer Multiplexer
/*
Pin,Port,Bit
19  6 16
18  6 17
14  6 18
15  6 19
40  6 20
41  6 21
17  6 22
16  6 23
 */ 
Timer layerTimer;
void layerISR();
#define LAYER_MULTIPLEXER_ADDRESS_OFFSET 16
const byte layer_0_pin = 19;
const byte layer_1_pin = 18;
const byte layer_2_pin = 14;
const byte layer_3_pin = 15;
const byte layer_4_pin = 40;
const byte layer_5_pin = 41;
const byte layer_6_pin = 17;
const byte layer_7_pin = 16;

uint8_t  layerShiftData = B10000000;
uint32_t layerData      = uint32_t(~layerShiftData) << LAYER_MULTIPLEXER_ADDRESS_OFFSET;
uint32_t layerBitMask   = uint32_t(B11111111) << LAYER_MULTIPLEXER_ADDRESS_OFFSET;

const byte layerAmount = 4;
byte currentLayerIndex = 7;
byte internBrightnessList[layerAmount][dataPorts][16];

 //----------------------------------------------
 // I2C bus addresses (excludes the R/W bit)
#define ADDRESS                        0b1100000  // Assumes A0 to A3 are connected to ground
#define ALLCALL_ADDRESS                0b1101000
#define RESET_ADDRESS                  0b1101011

// I2C R/W
#define I2C_READ                       1
#define I2C_WRITE                      0

// Control register (three MSB control auto-increment)
#define NO_AUTO_INCREMENT              0b00000000
#define AUTO_INCREMENT_ALL_REGISTERS   0b10000000
#define AUTO_INCREMENT_BRIGHTNESS      0b10100000
#define AUTO_INCREMENT_CONTROL         0b11000000
#define AUTO_INCREMENT_BRIGHT_CONTROL  0b11100000

// TLC59116 registers
#define TLC59116_GRPPWM                0x12
#define TLC59116_LEDOUT0               0x14
#define TLC59116_LEDOUT1               0x15
#define TLC59116_LEDOUT2               0x16
#define TLC59116_LEDOUT3               0x17

// PWM registers
#define TLC_59116_PWM0                 0x02
#define TLC_59116_PWM1                 0x03
#define TLC_59116_PWM2                 0x04
#define TLC_59116_PWM3                 0x05
#define TLC_59116_PWM4                 0x06
#define TLC_59116_PWM5                 0x07
#define TLC_59116_PWM6                 0x08
#define TLC_59116_PWM7                 0x09
#define TLC_59116_PWM8                 0x0a
#define TLC_59116_PWM9                 0x0b
#define TLC_59116_PWM10                0x0c
#define TLC_59116_PWM11                0x0d
#define TLC_59116_PWM12                0x0e
#define TLC_59116_PWM13                0x0f
#define TLC_59116_PWM14                0x10
#define TLC_59116_PWM15                0x11


// LED output state for LEDOUT0 to LEDOUT3
#define LED_OUTPUT_OFF                 0b00
#define LED_OUTPUT_GROUP               0b11

volatile byte cubeStorageBusy = 0;
byte brightnessList[layerAmount][dataPorts][16];
float counter = 0;
byte counter2 = 0;

void setBrightness(byte layer, byte port,byte led, byte brightness);

void update();

 //----------------------------------------------
 struct Color
 {
	byte red;
	byte green;
	byte blue;
 };
 #ifdef CUBE_4
  const byte cubeSize = 4;
 #endif 
 #ifdef CUBE_8
  const byte cubeSize = 8;
 #endif
 Color cubeData[cubeSize][cubeSize][cubeSize];
 
void setColor(byte x, byte y, byte z, const Color &color);
void setColor(byte x, byte y, byte z, byte red, byte green, byte blue);


bool readSerialData();
 
void setup() {
  Serial.begin(1000000);
  pinMode(A9,INPUT);

  //Layer Multiplexer
  pinMode(layer_0_pin,OUTPUT);
  pinMode(layer_1_pin,OUTPUT);
  pinMode(layer_2_pin,OUTPUT);
  pinMode(layer_3_pin,OUTPUT);
  #ifdef CUBE_8
	  pinMode(layer_4_pin,OUTPUT);
	  pinMode(layer_5_pin,OUTPUT);
	  pinMode(layer_6_pin,OUTPUT);
	  pinMode(layer_7_pin,OUTPUT);
  #endif

  digitalWrite(layer_0_pin,1);
  digitalWrite(layer_1_pin,1);
  digitalWrite(layer_2_pin,1);
  digitalWrite(layer_3_pin,1);
  digitalWrite(layer_4_pin,1);
  digitalWrite(layer_5_pin,1);
  digitalWrite(layer_6_pin,1);
  digitalWrite(layer_7_pin,1);

  // I2C Pinmode
  pinMode(i2c_scl,OUTPUT);
  pinMode(i2c_sda_0,OUTPUT);
  pinMode(i2c_sda_4,OUTPUT);
  pinMode(i2c_sda_8,OUTPUT);
  #ifdef CUBE_8
	  pinMode(i2c_sda_1,OUTPUT);
	  pinMode(i2c_sda_2,OUTPUT);
	  pinMode(i2c_sda_3,OUTPUT);
	  pinMode(i2c_sda_5,OUTPUT);
	  pinMode(i2c_sda_6,OUTPUT);
	  pinMode(i2c_sda_7,OUTPUT);
	  pinMode(i2c_sda_9,OUTPUT);
	  pinMode(i2c_sda_10,OUTPUT);
	  pinMode(i2c_sda_11,OUTPUT);
  #endif
  
  

  pinMode(resetPin,OUTPUT);
  digitalWrite(resetPin,1);
  
  // I2C Init
  digitalWriteFast(i2c_scl,1);
  digitalWriteFast(i2c_sda_0,1);
  digitalWriteFast(i2c_sda_4,1);
  digitalWriteFast(i2c_sda_8,1);
  #ifdef CUBE_8
	  digitalWriteFast(i2c_sda_1,1);
	  digitalWriteFast(i2c_sda_2,1);
	  digitalWriteFast(i2c_sda_3,1);
	  digitalWriteFast(i2c_sda_5,1);
	  digitalWriteFast(i2c_sda_6,1);
	  digitalWriteFast(i2c_sda_7,1);
	  digitalWriteFast(i2c_sda_9,1);
	  digitalWriteFast(i2c_sda_10,1);
	  digitalWriteFast(i2c_sda_11,1);
  #endif
  
  

#ifndef DISABLE_DEBUG_PINS
  pinMode(debugPin,OUTPUT);
  pinMode(debugPin2,OUTPUT);
  pinMode(errorPin,OUTPUT);
  pinMode(debugBytePin,OUTPUT);
#endif
  //delay(100);
  uint32_t out = 0xff; 
  Serial.print("Timer: ");
  #ifndef USE_TIMER_ONE
  #ifdef USE_TIMERTOOL
  busTimer1.begin(interruptFunc, 2_MHz); 
  #else 
  Serial.println(busTimer.beginCycles(interruptFunc,10));
  busTimer.priority(10);
  #endif
  #else


// Setup Timer 1 for the i2cBus
//https://forum.pjrc.com/threads/59112-TeensyTimerTool/page2
  attachInterruptVector(IRQ_QTIMER1, interruptFunc);
  CCM_CCGR6 |= CCM_CCGR6_QTIMER1(CCM_CCGR_ON);
  TMR1_CTRL0 = 0x0000; // stop
  TMR1_LOAD0 = 0x0000; // start val after compare
  #ifdef I2C_SPPED_FAST
 // TMR1_COMP10  = 20; // count up to this val, interrupt,  and start again
 // TMR1_CMPLD10 = 20;
  TMR1_COMP10  = F_CPU_ACTUAL/30000000; // count up to this val, interrupt,  and start again
  TMR1_CMPLD10 = F_CPU_ACTUAL/30000000;
  TMR1_CTRL0 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(8) | TMR_CTRL_LENGTH ;  // prescale
  #else
  TMR1_COMP10  = 50; // count up to this val, interrupt,  and start again
  TMR1_CMPLD10 = 50;
  TMR1_CTRL0 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(8) | TMR_CTRL_LENGTH ;  // prescale
  #endif
  
  TMR1_CSCTRL0 &= ~(TMR_CSCTRL_TCF1); // clear
  TMR1_CSCTRL0 |= TMR_CSCTRL_TCF1EN;// enable interrupt
  NVIC_SET_PRIORITY(IRQ_QTIMER1, 255); // Top priority
  NVIC_ENABLE_IRQ(IRQ_QTIMER1);
  #endif
//-----------------------------------
  delay(100);
  layerTimer.begin(layerISR,400_Hz);
//-------------------------------------
//reset();        // added by aoc
  // Transmit to the TLC59116
  Serial.print("beginTransmission: ");
  Serial.println(ADDRESS,BIN);
  beginTransmission(ADDRESS);
  // Send the control register.  All registers will be written to, starting at register 0
  write(byte(AUTO_INCREMENT_ALL_REGISTERS));
  // Set MODE1: no sub-addressing
  write(byte(0));
  // Set MODE2: dimming
  write(byte(B00001000));
  // Set individual brightness control to maximum
  for (int i=0; i< 16; i++)
    write(byte(0x00));  //0XFF 
  // Set GRPPWM: Full brightness
  write(byte(0xff));  //0XFF
  // Set GRPFREQ: Not blinking, must be 0
  write(byte(0));
  // Set LEDs off for now
  for (int i=0; i< 4; i++)
    write(byte(0x11));
  // Set the I2C all-call and sub addresses (if needed)
  endTransmission();
  delay(100);
  for(byte z=0; z<layerAmount; z++)
  {
    for(byte i=0; i<16; i++)
    {
      for(byte j=0; j<dataPorts; j++)
      {
        setBrightness(z,j,i,0);
      }
    }
  }
  setLEDs(0xFFFF);
  delay(10);
  Serial.println("update");
  update();
  Serial.println("setup done");
  delay(1000);
  update();
}
void loop() {
  for(;;){
	if(Serial.available())
	{
		readSerialData();
		update();	
	}
	//delay(10);
	//setColor(3,2,3,255,0,0);
	//update();
  }
}
void layerISR()
{
  TMR2_CSCTRL0 &= ~TMR_CSCTRL_TCF1; // clear the timer flag
  //Serial.println("a");
  if(cubeStorageBusy)
    return;
  currentLayerIndex++;
  layerShiftData = layerShiftData << 1;
  if(!layerShiftData || layerAmount == currentLayerIndex)
  {
    layerShiftData = 0x01;
    currentLayerIndex = 0;
  }
    
   //PORTD |= B00000100;
  beginTransmission(ADDRESS);
  // Write to the GRPPWM register
  write(byte(AUTO_INCREMENT_BRIGHTNESS + TLC_59116_PWM0));
  byte *arr = new byte[dataPorts];
  for(byte i=0; i<16; i++)
  {
    
    for(byte j=0; j<dataPorts; j++)
    {
      arr[j] = internBrightnessList[currentLayerIndex][j][i];
    }
    write(arr);
    
  }
  delete[] arr;
  endTransmission();
  //digitalWrite(debugPin,LOW);
 // PORTD &= B11111011;
}
void interruptFunc()
{
 // Serial.println("b");
  #ifdef USE_TIMER_ONE
    TMR1_CSCTRL0 &= ~TMR_CSCTRL_TCF1; // clear the timer flag
  #endif
  #ifndef DISABLE_DEBUG_PINS
  digitalWriteFast(debugPin2,!digitalRead(debugPin2));
  #endif
  //digitalWriteFast(debugPin,!digitalReadFast(debugPin));
  if(!bussy) 
  {
    return;
  }
   
  if(busStep == 0)
  {
    // make start condition
    //busState &= ~uint32_t(1<<sda1Bit); //sda bit ausschalten
   

    #ifndef I2C_USE_PORTMANIPULATION
		digitalWriteFast(i2c_sda_0,0);
		digitalWriteFast(i2c_sda_4,0);
		digitalWriteFast(i2c_sda_8,0);
	#ifdef CUBE_8
		digitalWriteFast(i2c_sda_1,0);
		digitalWriteFast(i2c_sda_2,0);
		digitalWriteFast(i2c_sda_3,0);
		digitalWriteFast(i2c_sda_5,0);
		digitalWriteFast(i2c_sda_6,0);
		digitalWriteFast(i2c_sda_7,0);
		digitalWriteFast(i2c_sda_9,0);
		digitalWriteFast(i2c_sda_10,0);
		digitalWriteFast(i2c_sda_11,0);
	#endif
    
    
    #else
      GPIO7_DR &= ~uint32_t((1<< sda_0_Bit) |
							(1<< sda_4_Bit) |
							(1<< sda_8_Bit) 
#ifdef CUBE_8
						   |(1<< sda_1_Bit) |
                            (1<< sda_2_Bit) |
                            (1<< sda_3_Bit) |
                            (1<< sda_5_Bit) |
                            (1<< sda_6_Bit) |
                            (1<< sda_7_Bit) |
                            (1<< sda_9_Bit) |
                            (1<< sda_10_Bit) |
                            (1<< sda_11_Bit)
#endif  
                            );
    #endif
    GPIO6_DR = GPIO6_DR | layerBitMask;
    busStep = 1;
  }else if(busStep == 1)
  {
    // lower clock
    //GPIO7_DR &= ~uint32_t(1<<clkBit); // clk auf 0 setzen
     #ifndef DISABLE_DEBUG_PINS
    digitalWriteFast(debugPin,1);
    #endif
    #ifndef I2C_USE_PORTMANIPULATION
    digitalWriteFast(i2c_scl,0);
    #else
    GPIO7_DR &= ~uint32_t(1<<clk_Bit);
    #endif
    #ifndef DISABLE_DEBUG_PINS
    digitalWriteFast(debugBytePin,0);
    #endif
    if(dataPosIndex == sendingDataSize)
    {
      // make stop condition
      busStep = 4;
    }
    else
    {
      busStep = 2;
    }
  }else if(busStep == 2)
  {
    // set data
    //busState = (busState&~uint32_t(1<<sda1Bit)) | uint32_t(((sendingData[0][dataPosIndex] & (1<<bitIndex))>>bitIndex)<<sda1Bit);
    //GPIO7_DR = (GPIO7_DR & ~uint32_t(1<<sda1Bit)) | busState;
    #ifndef I2C_USE_PORTMANIPULATION
	digitalWriteFast(i2c_sda_0,(sendingData[0][dataPosIndex] & (1<<bitIndex))>>bitIndex);
	digitalWriteFast(i2c_sda_4,(sendingData[4][dataPosIndex] & (1<<bitIndex))>>bitIndex);
	digitalWriteFast(i2c_sda_8,(sendingData[8][dataPosIndex] & (1<<bitIndex))>>bitIndex);
	#ifdef CUBE_8
		digitalWriteFast(i2c_sda_1,(sendingData[1][dataPosIndex] & (1<<bitIndex))>>bitIndex);
		digitalWriteFast(i2c_sda_2,(sendingData[2][dataPosIndex] & (1<<bitIndex))>>bitIndex);
		digitalWriteFast(i2c_sda_3,(sendingData[3][dataPosIndex] & (1<<bitIndex))>>bitIndex);
		digitalWriteFast(i2c_sda_5,(sendingData[5][dataPosIndex] & (1<<bitIndex))>>bitIndex);
		digitalWriteFast(i2c_sda_6,(sendingData[6][dataPosIndex] & (1<<bitIndex))>>bitIndex);
		digitalWriteFast(i2c_sda_7,(sendingData[7][dataPosIndex] & (1<<bitIndex))>>bitIndex);
		digitalWriteFast(i2c_sda_9,(sendingData[9][dataPosIndex] & (1<<bitIndex))>>bitIndex);
		digitalWriteFast(i2c_sda_10,(sendingData[10][dataPosIndex] & (1<<bitIndex))>>bitIndex);
		digitalWriteFast(i2c_sda_11,(sendingData[11][dataPosIndex] & (1<<bitIndex))>>bitIndex);
	#endif
    
    
    #else
    GPIO7_DR  = (GPIO7_DR & ~uint32_t((1<< sda_0_Bit) | 
									  (1<< sda_4_Bit) |
									  (1<< sda_8_Bit) 
#ifdef CUBE_8
									 |(1<< sda_1_Bit) |
                                      (1<< sda_2_Bit) |
                                      (1<< sda_3_Bit) |
                                      (1<< sda_5_Bit) |
                                      (1<< sda_6_Bit) |
                                      (1<< sda_7_Bit) |
                                      (1<< sda_9_Bit) |
                                      (1<< sda_10_Bit) |
                                      (1<< sda_11_Bit)
#endif
									   ))|  
                uint32_t(((sendingData[0][dataPosIndex] & (1<<bitIndex))>>bitIndex) << sda_0_Bit) | 
				uint32_t(((sendingData[4][dataPosIndex] & (1<<bitIndex))>>bitIndex) << sda_4_Bit) |
				uint32_t(((sendingData[8][dataPosIndex] & (1<<bitIndex))>>bitIndex) << sda_8_Bit) 
#ifdef CUBE_8
			  | uint32_t(((sendingData[1][dataPosIndex] & (1<<bitIndex))>>bitIndex) << sda_1_Bit) |
                uint32_t(((sendingData[2][dataPosIndex] & (1<<bitIndex))>>bitIndex) << sda_2_Bit) |
                uint32_t(((sendingData[3][dataPosIndex] & (1<<bitIndex))>>bitIndex) << sda_3_Bit) |
                uint32_t(((sendingData[5][dataPosIndex] & (1<<bitIndex))>>bitIndex) << sda_5_Bit) |
                uint32_t(((sendingData[6][dataPosIndex] & (1<<bitIndex))>>bitIndex) << sda_6_Bit) |
                uint32_t(((sendingData[7][dataPosIndex] & (1<<bitIndex))>>bitIndex) << sda_7_Bit) |
                uint32_t(((sendingData[9][dataPosIndex] & (1<<bitIndex))>>bitIndex) << sda_9_Bit) |
                uint32_t(((sendingData[10][dataPosIndex] & (1<<bitIndex))>>bitIndex) << sda_10_Bit) |
                uint32_t(((sendingData[11][dataPosIndex] & (1<<bitIndex))>>bitIndex) << sda_11_Bit)
#endif  
				;
    #endif

   
    
    if(bitIndex == 0)
    {
      bitIndex = 7; // wait for Acknowledge
      dataPosIndex++;
      busStep = 8;
      
    }
    else
    {
       bitIndex -= 1;
       busStep = 3;
    }
    
  }else if(busStep == 3)
  {
    // rise clock
   // GPIO7_DR |= uint32_t(1<<clkBit); // clk setzen
    #ifndef I2C_USE_PORTMANIPULATION
    digitalWriteFast(i2c_scl,1);
    #else
    GPIO7_DR |= uint32_t(1<<CORE_PIN10_BIT);
    #endif
    busStep = 1;
  }else if(busStep == 4)
  {
    // set data low
    //busState &= ~uint32_t(1<<sda1Bit); //sda bit ausschalten
    #ifndef I2C_USE_PORTMANIPULATION
	digitalWriteFast(i2c_sda_0,0);
	digitalWriteFast(i2c_sda_4,0);
	digitalWriteFast(i2c_sda_8,0);
	#ifdef CUBE_8
		digitalWriteFast(i2c_sda_1,0);
		digitalWriteFast(i2c_sda_2,0);
		digitalWriteFast(i2c_sda_3,0);
		digitalWriteFast(i2c_sda_5,0);
		digitalWriteFast(i2c_sda_6,0);
		digitalWriteFast(i2c_sda_7,0);
		digitalWriteFast(i2c_sda_9,0);
		digitalWriteFast(i2c_sda_10,0);
		digitalWriteFast(i2c_sda_11,0);
	#endif
   
    
    #else
      GPIO7_DR &= ~uint32_t((1<< sda_0_Bit) | 
						    (1<< sda_4_Bit) |
							(1<< sda_8_Bit) 
  #ifdef CUBE_8
					      | (1<< sda_1_Bit) |
                            (1<< sda_2_Bit) |
                            (1<< sda_3_Bit) |
                            (1<< sda_5_Bit) |
                            (1<< sda_6_Bit) |
                            (1<< sda_7_Bit) |
                            (1<< sda_9_Bit) |
                            (1<< sda_10_Bit) |
                            (1<< sda_11_Bit)
  #endif
                            );
    #endif
    busStep = 5;
  }else if(busStep == 5)
  {
    // last clock
    // rise clock
    //GPIO7_DR |= uint32_t(1<<clkBit); // clk setzen
    #ifndef I2C_USE_PORTMANIPULATION
    digitalWriteFast(i2c_scl,1);
    #else
    GPIO7_DR |= uint32_t(1<<clk_Bit);
    #endif
    busStep = 6;
  }else if(busStep == 6)
  {
    // make stop condition
    //GPIO7_DR |= uint32_t(1<<sda1Bit); //sda bit einschalten
    #ifndef I2C_USE_PORTMANIPULATION
    digitalWriteFast(i2c_sda_0,1);
	digitalWriteFast(i2c_sda_4,1);
	digitalWriteFast(i2c_sda_8,1);
	#ifdef CUBE_8
		digitalWriteFast(i2c_sda_1,1);
		digitalWriteFast(i2c_sda_2,1);
		digitalWriteFast(i2c_sda_3,1);
		digitalWriteFast(i2c_sda_5,1);
		digitalWriteFast(i2c_sda_6,1);
		digitalWriteFast(i2c_sda_7,1);
		digitalWriteFast(i2c_sda_9,1);
		digitalWriteFast(i2c_sda_10,1);
		digitalWriteFast(i2c_sda_11,1);
	#endif
    
    #else
      GPIO7_DR |=  uint32_t((1<< sda_0_Bit) | 
							(1<< sda_4_Bit) |
							(1<< sda_8_Bit) 
  #ifdef CUBE_8
						  | (1<< sda_1_Bit) |
                            (1<< sda_2_Bit) |
                            (1<< sda_3_Bit) |
                            (1<< sda_5_Bit) |
                            (1<< sda_6_Bit) |
                            (1<< sda_7_Bit) |
                            (1<< sda_9_Bit) |
                            (1<< sda_10_Bit) |
                            (1<< sda_11_Bit)
  #endif  
							);
    #endif
    //GPIO7_DR = (GPIO7_DR & ~uint32_t(1<<sda1Bit)) | busState;
    busStep = 7;
  }else if(busStep == 7)
  {
    // resetBus

    // switch layer
    busStep = 14;
    
    #ifndef DISABLE_DEBUG_PINS
    digitalWriteFast(errorPin,0);
    digitalWriteFast(debugPin,0);
    #endif
   // TMR1_CTRL0 = 0x0000; // stop
  }else if(busStep == 8)
  {
    // rise clock and than wait for Acknowledge
   // GPIO7_DR |= uint32_t(1<<clkBit); // clk setzen
    #ifndef I2C_USE_PORTMANIPULATION
    digitalWriteFast(i2c_scl,1);
    #else
    GPIO7_DR |= uint32_t(1<<clk_Bit);
    #endif
    #ifndef DISABLE_DEBUG_PINS
    digitalWriteFast(debugBytePin,1);
    #endif
    busStep = 9;
  }else if(busStep == 9)
  {
    // rise clock and than wait for Acknowledge
   // GPIO7_DR |= uint32_t(1<<clkBit); // clk setzen
    #ifndef I2C_USE_PORTMANIPULATION
    digitalWriteFast(i2c_scl,0);
    digitalWriteFast(i2c_sda_0,0);
	digitalWriteFast(i2c_sda_4,0);
	digitalWriteFast(i2c_sda_8,0);
	#ifdef CUBE_8
		digitalWriteFast(i2c_sda_1,0);
		digitalWriteFast(i2c_sda_2,0);
		digitalWriteFast(i2c_sda_3,0);
		digitalWriteFast(i2c_sda_5,0);
		digitalWriteFast(i2c_sda_6,0);
		digitalWriteFast(i2c_sda_7,0);
		digitalWriteFast(i2c_sda_9,0);
		digitalWriteFast(i2c_sda_10,0);
		digitalWriteFast(i2c_sda_11,0);
	#endif
    
    #else
   // GPIO7_DR &= ~uint32_t(1<<clk_Bit) | uint32_t(1<<sda_0_Bit) | uint32_t(1<<sda_1_Bit);

    GPIO7_DR &=   uint32_t(	(1<< sda_0_Bit) | 
							(1<< sda_4_Bit) |
							(1<< sda_8_Bit) 
	#ifdef CUBE_8
						  | (1<< sda_1_Bit) |
                            (1<< sda_2_Bit) |
                            (1<< sda_3_Bit) |
                            (1<< sda_5_Bit) |
                            (1<< sda_6_Bit) |
                            (1<< sda_7_Bit) |
                            (1<< sda_9_Bit) |
                            (1<< sda_10_Bit) |
                            (1<< sda_11_Bit) 
	#endif
                           )| ~uint32_t(1<<clk_Bit);
    #endif
	#ifndef I2C_IGNORE_ACK
    pinMode(i2c_sda_0,INPUT_PULLUP); // to read ack
	pinMode(i2c_sda_4,INPUT_PULLUP); // to read ack
	pinMode(i2c_sda_8,INPUT_PULLUP); // to read ack
	#ifdef CUBE_8
		pinMode(i2c_sda_1,INPUT_PULLUP); // to read ack
		pinMode(i2c_sda_2,INPUT_PULLUP); // to read ack
		pinMode(i2c_sda_3,INPUT_PULLUP); // to read ack
		pinMode(i2c_sda_5,INPUT_PULLUP); // to read ack
		pinMode(i2c_sda_6,INPUT_PULLUP); // to read ack
		pinMode(i2c_sda_7,INPUT_PULLUP); // to read ack
		pinMode(i2c_sda_9,INPUT_PULLUP); // to read ack
		pinMode(i2c_sda_10,INPUT_PULLUP); // to read ack
		pinMode(i2c_sda_11,INPUT_PULLUP); // to read ack
	#endif
	#endif
    
    busStep = 10;
  }else if(busStep == 10)
  {
    // wait for equal spaced clk signal
    busStep = 11;
  }else if(busStep == 11)
  {
    // rise clock and than wait for Acknowledge
   // GPIO7_DR |= uint32_t(1<<clkBit); // clk setzen
    #ifndef I2C_USE_PORTMANIPULATION
    digitalWriteFast(i2c_scl,1);
    #else
    GPIO7_DR |= uint32_t(1<<clk_Bit);
    #endif
    busStep = 12;
  }else if(busStep == 12)
  {
    // rise clock and than wait for Acknowledge
   // GPIO7_DR |= uint32_t(1<<clkBit); // clk setzen
   #ifndef I2C_IGNORE_ACK
    byte ack_0 = digitalReadFast(i2c_sda_0);
    byte ack_4 = digitalReadFast(i2c_sda_4);
    byte ack_8 = digitalReadFast(i2c_sda_8);
  pinMode(i2c_sda_0,OUTPUT); 
	pinMode(i2c_sda_4,OUTPUT); 
	pinMode(i2c_sda_8,OUTPUT); 
	#ifdef CUBE_8
		pinMode(i2c_sda_1,OUTPUT); 
		pinMode(i2c_sda_2,OUTPUT); 
		pinMode(i2c_sda_3,OUTPUT); 
		pinMode(i2c_sda_5,OUTPUT); 
		pinMode(i2c_sda_6,OUTPUT); 
		pinMode(i2c_sda_7,OUTPUT); 
		pinMode(i2c_sda_9,OUTPUT); 
		pinMode(i2c_sda_10,OUTPUT); 
		pinMode(i2c_sda_11,OUTPUT); 
	#endif
	#endif
    

    // lower clock
    //GPIO7_DR &= ~uint32_t(1<<clkBit); // clk auf 0 setzen
    #ifndef I2C_USE_PORTMANIPULATION
    digitalWriteFast(i2c_scl,0);
    #else
    GPIO7_DR &= ~uint32_t(1<<clk_Bit);
    #endif
    #ifndef DISABLE_DEBUG_PINS
    digitalWriteFast(debugBytePin,0);
    #endif
    
   // busStep = 13;
   #ifndef I2C_IGNORE_ACK
    if(!(ack_0 + ack_4 + ack_8))
		#endif
    {
      /*Serial.print("!ACK");
      Serial.print(ack_0);
      Serial.print(ack_4);
      Serial.println(ack_8);*/
      busStep = 13;
      //digitalWriteFast(ackErrorPin,1);
    }  
    /*else
    {
     // digitalWriteFast(i2c_scl,1);
     // digitalWriteFast(i2c_sda_0,1);
     // digitalWriteFast(i2c_sda_1,1);
      busStep = 6;
      #ifndef DISABLE_DEBUG_PINS
      digitalWriteFast(errorPin,1);
      #endif
    }*/
  }else if(busStep == 13)
  {
    if(dataPosIndex == sendingDataSize)
    {
      // make stop condition
      busStep = 4;
    }
    else
    {
      busStep = 2;
    }
  }else if(busStep == 14)
  {

    
    layerData = uint32_t(~layerShiftData) << LAYER_MULTIPLEXER_ADDRESS_OFFSET;
    GPIO6_DR  = (GPIO6_DR & ~layerBitMask) | layerData;
    
    
    busStep = 0;
    bussy   = 0;
  }
 
}

void beginTransmission(byte address)
{
  byte *arrAddress = new byte[dataPorts];
  for(int i=0; i<dataPorts; i++)
  {
    arrAddress[i] = address;
  }
  beginTransmission(arrAddress);
  delete[] arrAddress;
}
void beginTransmission(byte *address)
{
  while(bussy){
    //delay(1);
    /*Serial.println("bussy");*/ /*delay(10);*/}
  dataPosIndex = 0;
  sendingDataSize = 0;
  /*for(int i=0; i<dataPorts; i++)
  {
    for(int j=0; j<bufferSize; j++)
    {
      sendingData[i][j] = 0; // Clear the buffer
    }
  }*/
  for(int i=0; i<dataPorts; i++)
  {
    sendingData[i][dataPosIndex] = (address[i]<<1) | I2C_WRITE; 
  }
  dataPosIndex++;
  sendingDataSize++;
}
void write(byte data)
{
  byte *arrData = new byte[dataPorts];
  for(int i=0; i<dataPorts; i++)
  {
    arrData[i] = data;
  }
  write(arrData);
  delete[] arrData;
}
void write(byte *data)
{
  while(bussy){
   /* Serial.println("bussy");*/ /*delay(10);*/}
  for(int i=0; i<dataPorts; i++)
  {
    sendingData[i][dataPosIndex] = data[i];
  }
  dataPosIndex++;
  if(dataPosIndex >= bufferSize)
  {
    dataPosIndex = 0;
    Serial.println("Buffer overflow");
  }
  sendingDataSize++;
}
void endTransmission()
{
  while(bussy){
    
    /*Serial.println("bussy");*/ /*delay(10);*/}
  dataPosIndex = 0;
  /*Serial.println("Datastream: ");
  //char line[10*dataPorts];
  for(int j=0; j<bufferSize; j++)
  {
    for(int i=0; i<dataPorts; i++)
    {
      String binStr(sendingData[i][j],BIN);
      if(binStr.length()<8)
        binStr+="\t"; 
      Serial.print(binStr); // Clear the buffer
      Serial.print(" \t");
    }
    Serial.print("\n");
  }
  Serial.println("");*/
  bussy = 1;
 // TMR1_CSCTRL0 |= TMR_CSCTRL_TCF1EN;// enable interrupt
}




//------------------------------------------------------------------
void setLEDs(int LEDs)
{
  int registerVal=0;
  int registerIncrement = LED_OUTPUT_GROUP;
  Serial.println("Set Leds");
  beginTransmission(ADDRESS);
  // Write to consecutive registers, starting with LEDOUT0
  write(byte(AUTO_INCREMENT_ALL_REGISTERS + TLC59116_LEDOUT0));
  
  // Write the value for LEDs
  for (int i=0; i< 16; i++) {
    if (LEDs & 0x01)
      registerVal += registerIncrement;
    // Move to the next LED
    LEDs >>= 1;
    // Are 4 LED values in the register now?
    if (registerIncrement == LED_OUTPUT_GROUP<<6) {
      // The register can be written out now
      write((byte) registerVal);
      registerVal = 0;
      registerIncrement = LED_OUTPUT_GROUP;
    }
    else {
      // Move to the next increment
      registerIncrement <<= 2;
    }
  }
  endTransmission();
}


void setColor(byte x, byte y, byte z, const Color &color)
{
	cubeData[x][y][z] = color;
}
void setColor(byte x, byte y, byte z, byte red, byte green, byte blue)
{
	cubeData[x][y][z].red   = red;
	cubeData[x][y][z].green = green;
	cubeData[x][y][z].blue  = blue;
}
void setBrightness(byte layer,byte port, byte led, byte brightness)
{
  
  brightnessList[layer][port][led] = brightness;
  
}

void update()
{
  cubeStorageBusy = true;
  //Serial.println("update:");
  for(byte z=0; z<layerAmount; z++)
  {
	#ifdef CUBE_4
	byte pos = 0;
	/*Serial.print("Layer: ");
	Serial.print(z);
	Serial.print(" ");*/
	for(byte x=0; x<cubeSize; x++)
	{
		for(byte y=0; y<cubeSize; y++)
		{
			internBrightnessList[z][redPortOffset][pos]   = cubeData[x][y][z].red;
			internBrightnessList[z][greenPortOffset][pos] = cubeData[x][y][z].green;
			internBrightnessList[z][bluePortOffset][pos]  = cubeData[x][y][z].blue;
			
		/*	Serial.print(internBrightnessList[z][redPortOffset][pos] );
			Serial.print(" ");
			Serial.print(internBrightnessList[z][greenPortOffset][pos] );
			Serial.print(" ");
			Serial.print(internBrightnessList[z][bluePortOffset][pos] );
			Serial.print("  ");*/
			
			pos++;
		}
	}
	//Serial.println();
	
	#endif
	
	#ifdef CUBE_8
	 notDefined
	#endif
  }
 // Serial.println("\nupdate end");
  cubeStorageBusy = false;
}
double mapF(double in, double inMin, double inMax, double outMin, double outMax)
{ 
	double out; 
	out = (in-inMin)/(inMax-inMin)*(outMax-outMin) + outMin; 
	return out; 
}

bool readSerialData()
{
	size_t arraySize = cubeSize*2 + cubeSize*cubeSize*cubeSize*3 + 1 +50;
	
	for(byte z=0; z<cubeSize; z++)
	{
		
		byte layerIndex;
		if(Serial.read() == 'L')
		{
			layerIndex = Serial.read()-1;
			
			/*Serial.print("L");
			Serial.print(layerIndex);
			Serial.print(" ");*/
			for(byte x=0; x<cubeSize; x++)
			{
				for(byte y=0; y<cubeSize; y++)
				{
					//if(!Serial.available())
						//return false;
					
					cubeData[x][y][layerIndex].red   = Serial.read()-1;//buffer[iterator];  	iterator++;
					cubeData[x][y][layerIndex].green = Serial.read()-1;//buffer[iterator];  	iterator++;
					cubeData[x][y][layerIndex].blue  = Serial.read()-1;//buffer[iterator];  	iterator++;
		
					/*
					Serial.print(cubeData[x][y][layerIndex].red  );
					Serial.print(" ");
					Serial.print(cubeData[x][y][layerIndex].green);
					Serial.print(" ");
					Serial.print(cubeData[x][y][layerIndex].blue );
					Serial.print("  ");*/
					
				}
			}
			//Serial.print("\n");
		}
	}
	//Serial.println("finish");

	return true;
}
