/*
 
    Interfacing Gameboy Camera with Addafruit Feather 32u4, 
    created by goatspit, March 17, 2017.
                                               
    This was forked from https://github.com/shimniok/avr-gameboy-cam,
    original Gameboy Camera Code by Laurent Saint-Marcel (lstmarcel@yahoo.fr).
    
    Modified and adapted for the Adafruit Feather 32u4 board by goatspit.
    
*/

/* ------------------------------------------------------------------------ */
/* HOW TO CONNECT THE GAME BOY CAMERA                                       */
/* ------------------------------------------------------------------------ */
/*

  READ  -- 11, PB7
  XCK   -- 15, PB1 
  XRST  -- 16, PB2
  LOAD  -- 14, PB3
  SIN   -- 10, PB6
  START --  9, PB5
  VOUT  -- A3, PF4
  
 */

/* ------------------------------------------------------------------------ */
/* INCLUDES                                                                 */
/* ------------------------------------------------------------------------ */
 
#include <avr/interrupt.h>
#include <avr/io.h>
#include <compat/deprecated.h>

#include "cam.h"
#include "camCom.h"

/* ------------------------------------------------------------------------ */
/* DEFINES                                                                  */
/* ------------------------------------------------------------------------ */

#define CAM_DATA_PORT     PORTB
#define CAM_DATA_DDR      DDRB
#define CAM_READ_PIN      11

// CAM_DATA_PORT
#define CAM_READ_BIT  7
#define CAM_SIN_BIT   6
#define CAM_START_BIT 5
#define CAM_LOAD_BIT  3
#define CAM_RESET_BIT 2
#define CAM_CLOCK_BIT 1
#define CAM_ADC_PIN   A3

// Define end of row of obj/img buffers for pointer math
#define THEEND (2*128-1)

#define FALSE 0
#define TRUE 1

/* ------------------------------------------------------------------------ */
/* GLOBALS                                                                  */
/* ------------------------------------------------------------------------ */

unsigned char camReg[8]={ 0x80, 0x03, 0x00, 0x30, 0x01, 0x00, 0x01, 0x21 };

unsigned char camMode       = CAM_MODE_STANDARD;
unsigned char camClockSpeed = 0x0A;
unsigned char camPixMin =0;
unsigned char camPixMax =0xFF;


/* ------------------------------------------------------------------------ */
/* MACROS                                                                   */
/* ------------------------------------------------------------------------ */

#define Serialwait()   while (Serial.available() == 0) ;

int dataIn;
int dataOut;
boolean dataReady;
unsigned char reg;


/* ------------------------------------------------------------------------ */
/* Initialize all components                                                */
/* ------------------------------------------------------------------------ */
void setup()
{
  dataReady = false;
	Serial.begin(115200);
	camInit();
	/* enable interrupts */
	sei();
}

/* ------------------------------------------------------------------------ */
/* Program entry point                                                      */
/* ------------------------------------------------------------------------ */
void loop()
{
  while (Serial.available() == 0)
    delay(50);
  dataIn = Serial.read();
  switch (dataIn) {
    case CAM_COM_PING:
      dataOut = CAM_COM_PONG;
      Serial.write(CAM_COM_PONG);
      break;
    case CAM_COM_TAKE_PICTURE:
      // Send 8 camera registers
      camReset();
      camSetRegisters();
      camReadPicture(true);
      camReset();
      break;
    case CAM_COM_SET_REGISTERS:
      Serialwait();
      camStoreReg(0, Serial.read());
      Serialwait();
      camStoreReg(1, Serial.read());
      Serialwait();
      camStoreReg(2, Serial.read());
      Serialwait();
      camStoreReg(3, Serial.read());
      Serialwait();
      camStoreReg(4, Serial.read());
      Serialwait();
      camStoreReg(5, Serial.read());
      Serialwait();
      camStoreReg(6, Serial.read());
      Serialwait();
      camStoreReg(7, Serial.read());
      Serialwait();
      Serial.write(CAM_COM_GOT_REGISTERS);
      break;
    default:
      Serial.write(CAM_COM_ERROR);
      break;
  } // switch-case
  
} // loop


// Delay used between each signal sent to the AR (four per xck cycle).
void camStepDelay() {
  	unsigned char u=camClockSpeed;
	while(u--) {__asm__ __volatile__ ("nop");}
}
// Set the clock signal Low
inline void camClockL()
{
	cbi(CAM_DATA_PORT, CAM_CLOCK_BIT);
}
// Set the clock signal High
inline void camClockH()
{
	sbi(CAM_DATA_PORT, CAM_CLOCK_BIT);
}


// Initialise the IO ports for the camera
void camInit()
{
  pinMode(11, INPUT);   // READ
  pinMode(15, OUTPUT);  // XCK
  pinMode(16, OUTPUT);  // XRST
  pinMode(14, OUTPUT); // LOAD
  pinMode(10, OUTPUT); // SIN
  pinMode(9, OUTPUT); // START
  
  cbi(CAM_DATA_PORT, CAM_CLOCK_BIT);
  sbi(CAM_DATA_PORT, CAM_RESET_BIT);  // Reset is active low
  cbi(CAM_DATA_PORT, CAM_LOAD_BIT);
  cbi(CAM_DATA_PORT, CAM_START_BIT);
  cbi(CAM_DATA_PORT, CAM_SIN_BIT);
}


// Sends a 'reset' pulse to the AR chip.
// START:  XCK Rising Edge 
// FINISH: XCK Just before Falling Edge
void camReset()
{
  camClockH(); // clock high
  camStepDelay();
  camStepDelay();
 
  camClockL(); // clock low
  camStepDelay();
  cbi(CAM_DATA_PORT, CAM_RESET_BIT);
  camStepDelay();

  camClockH(); // clock high
  camStepDelay();
  sbi(CAM_DATA_PORT, CAM_RESET_BIT);
  camStepDelay();
}


// locally set the value of a register but do not set it in the AR chip. You 
// must run camSendRegisters1 to write the register value in the chip
void camStoreReg(unsigned char reg, unsigned char data) 
{
  camReg[reg] = data;
}

// Reset the camera and set the camera's 8 registers
// from the locally stored values (see camStoreReg)
void camSetRegisters(void)
{
  for(reg=0; reg<8; ++reg) {
    camSetReg(reg, camReg[reg]);
  }
}

// Sets one of the 8 8-bit registers in the AR chip.
// START:  XCK Falling Edge 
// FINISH: XCK Just before Falling Edge
void camSetReg(unsigned char regaddr, unsigned char regval)
{
  unsigned char bitmask;

  // Write 3-bit address.
  for(bitmask = 0x4; bitmask >= 0x1; bitmask >>= 1){
    camClockL();
    camStepDelay();
    // ensure load bit is cleared from previous call
    cbi(CAM_DATA_PORT, CAM_LOAD_BIT);
    // Set the SIN bit
    if(regaddr & bitmask)
      sbi(CAM_DATA_PORT, CAM_SIN_BIT);
    else
      cbi(CAM_DATA_PORT, CAM_SIN_BIT);
    camStepDelay();

    camClockH();
    camStepDelay();
    // set the SIN bit low
    cbi(CAM_DATA_PORT, CAM_SIN_BIT);
    camStepDelay();
  }

  // Write 7 most significant bits of 8-bit data.
  for(bitmask = 128; bitmask >= 1; bitmask>>=1){
    camClockL();
    camStepDelay();
    // set the SIN bit
    if(regval & bitmask)
      sbi(CAM_DATA_PORT, CAM_SIN_BIT);
    else
      cbi(CAM_DATA_PORT, CAM_SIN_BIT);
    camStepDelay();
    // Assert load at rising edge of xck
    if (bitmask == 1)
      sbi(CAM_DATA_PORT, CAM_LOAD_BIT);
    camClockH();
    camStepDelay();
    // reset the SIN bit
    cbi(CAM_DATA_PORT, CAM_SIN_BIT);
    camStepDelay();
  }
}

void printHeaderToSerial() {
  Serial.println("P2");
  Serial.println("128 128");
  Serial.println("255");
}



// Take a picture, read it and send it through the serial port. 

// START:  XCK Falling Edge 
// FINISH: XCK Just before Rising Edge
void camReadPicture(boolean getPixels)
{
  // Camera START sequence
  camClockL();
  camStepDelay();
  // ensure load bit is cleared from previous call
  cbi(CAM_DATA_PORT, CAM_LOAD_BIT);
  // START rises before xck
  sbi(CAM_DATA_PORT, CAM_START_BIT);
  camStepDelay();

  camClockH();
  camStepDelay();
  // start valid on rising edge of xck, so can drop now
  cbi(CAM_DATA_PORT, CAM_START_BIT);
  camStepDelay();

  camClockL();
  camStepDelay();
  camStepDelay();
 
  // Wait for READ to go high
  while (1) {
    camClockH();
    camStepDelay();
    // READ goes high with rising XCK
    if (digitalRead(CAM_READ_PIN) == HIGH)
      break;
    camStepDelay();

    camClockL();
    camStepDelay();
    camStepDelay();
  }
  
  camStepDelay();
   
  // Read pixels from cam until READ signal is low again

  // Set registers while reading the first 11-ish pixels
  // The camera seems to be spitting out 128x128 even though the final 5 rows are junk
  // printHeaderToSerial();
  
  for (size_t y = 0; y < 123; y++) {
    for (size_t x = 0; x < 128; x++) {

      camClockL();
      camStepDelay();
      // get the next pixel, buffer it, and send it out over serial
      uint8_t pixel = analogRead(CAM_ADC_PIN) >> 2;
      if (getPixels) {
        Serial.write(pixel);
      }
      camClockH();
      camStepDelay();
    } // end for x
  } // end for y

  // Go through the remaining rows
  while ( digitalRead(CAM_READ_PIN) == HIGH ) { 
    camClockL();
    camStepDelay();
    camStepDelay();
    camClockH();
    camStepDelay();
    camStepDelay();
  }

  camClockL();
  camStepDelay();
  camStepDelay();
  Serial.print('\0');
}

