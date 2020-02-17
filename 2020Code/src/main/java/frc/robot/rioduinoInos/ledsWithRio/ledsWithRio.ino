#include <Adafruit_NeoPixel.h>  // for the NeoPixel array
#include "SPI.h"                // the following three headers are all required
#include "Adafruit_GFX.h"       // to drive the TFT
#include "Adafruit_ILI9341.h"

// Bling control 
//
// This program uses code from the Arduino "Serial Event" example as well as
// code from the Adafruit "strandtest" and "graphicstest"examples. It implements 
// a serial protocol for sending commands on the serial line from a roboRIO to 
// this program to control the LEDs on an Adafruit neopixel strip. The protocol is 
// quite simple and is required because the NeoPixel "show" method locks interrupts
// for a long enough period (and there are some lengthy delay loops in other 
// places) that incomming characters can be dropped. To get around this the
// code:
//  - runs a function to display a selected pattern on the LED array,
//  - polls the serial line for an "I" character (a command "Interrupt"
//    from the roboRIO). 
//  - repeat ad infinitum
// When it sees the comand character all pixel processing stops,
// a "ready" character ("R") is sent back to the roboRIO, and the serial line
// is then monitored for a command string of one or more commands in the form:
//   <cmd char><cmd_val>
// the last of which is terminated with a "Z". For example:
//   F2C255D25Z - set the color to 255 and the delay to 25 for function 2
//   E1Z        - start executing function 1
// Once received, the string is parsed, the commands executed, and the code
// goes back into the simple display/serial poll loop described above. The
// currently supported command characters are documented in processCommand
// (below). The protocl looks like this:
//
//    Arduino                   roboRIO
//       |                        |
//       |<-----------------------I
//       |                        |
//       R----------------------->|
//       |                        |
//       |<---------------------<cmd>
//       .                        .
//       .                        .
//       R----------------------->|  (roboRIO waits for command execution to
//       .                        .   complete)
//                  or
//  
//       R----------------------->|
//       |                        |
//       |<---------------------<cmd>
//       |                        |
//       |<-----------------------I  (roboRIO interrupts current command to
//       |                        |   begin executing a new one)
//       R----------------------->| 
//       .                        .
//
#define CMD_CHAR 'I'      // The character the roboRIO sends us to indicate it wants
                          // to send a command
#define RESPONSE_CHAR 'R' // The character we send to the roboRIO to let it know
#define RESPONSE_STR  "R" // we are ready to receive a command

// ToDo : add blink and rainbow blink
//      : more comments on debug usage
#define STATUS_PIN  13 // The status LED is off-board

#define STRIP_PIN    6 // The Arduino digital IO pin used to send data to the LED array
#define STRIP_LEN  150 // The number of pixels in the LED strip
#define MAX_Q      150 // the last LED to use. Shold not be greater than STRIP_LEN
#define DEF_Q      110 // need to count this on competition bot

// NeoPixel object construction
// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(MAX_Q, STRIP_PIN, NEO_GRB + NEO_KHZ800);

// ToDo: These no longer need to be globals. They should be declared in serialEvent
char     commandChar = ' ';       // the last command from the roboRIO
uint32_t commandVal  = 0;

// A structure to contain the configuration data for a specific bling function
typedef struct {
  uint32_t color;
  uint32_t delay;
  byte     brightness;
  uint16_t pixelStart;
  uint16_t pixelEnd;
  uint32_t repeat;
} blingParms_t;

// The bling functions that can be configured and run via the serial command interface
typedef enum {
  CLEAR,                 // 0
  COLOR_WIPE,            // 1
  COLOR_WIPE_WITH_BLANK, // 2
  THEATRE_CHASE,         // 3
  RAINBOW,               // 4
  THEATRE_CHASE_RAINBOW, // 5
  COLOR_BAR,             // 6
  COLOR_BAR_FLASH,       // 7
  BOUNCE,                // 8
  BOUNCE_WIPE,           // 9
  MULTI_BOUNCE,          // 10
  MULTI_BOUNCE_WIPE,     // 11
  MULTI_COLOR_WIPE,      // 12
  NUM_FUNCTIONS     
} function_t;

// Global variables
boolean      commandFlag         = false; // a flag indicating that there is a command req from the roboRIO
boolean      serialDebugEnabled  = true;  // set this to true to see a bunch of debug stuff on the serial output
boolean      doneSent            = false; // a flag to indicate that we have sent the done signal to the roboRIO
function_t   configFunction;              // the bling func we are currently configuring from received commands
function_t   runningFunction;             // the bling function we are currently using on the LED array
uint32_t     repeatCount;                 // the number of remaining times we will execute the current bling func
int          ledState            = LOW;   // ledState used to set the status LED
uint32_t     prevLedMillis       = 0;     // will store last time status LED was updated
uint32_t     prevShowMillis      = 0;     // will store the last time the debug status was output

// the following variables are longs because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long ledInterval  =  500;                 // interval at which to blink (milliseconds)

#define BLU 0x0000FF
#define GRN 0x00FF00
#define RED 0xFF0000
#define YEL 0xE09E1C
#define PUR 0x3B0966
#define WHT 0xFFFFFF
#define TUR 0x00FFFF
#define PNK 0xD60C9F

// This table holds configuration data for each bling function we support. It can be modified by sending
// the "F" command to select the function to configure followed by other commands to modify specific
// values for the selected function.

#define DNT 0 // Do not change these
#define UN  0 // Unused - changing these won't make any difference

blingParms_t blingParmsTable [NUM_FUNCTIONS] = {
//   C    D    B    P    Q    R
  {DNT,  UN,  UN,   0, MAX_Q,   1}, // 0 - clear
  {PNK,  25,  64,   0, DEF_Q,   1}, // 1 - colorWipe
  {GRN,  25,  64,   0, DEF_Q,   5}, // 2 - colorWipeWithBlank
  {TUR,  25,  64,   0, DEF_Q,   5}, // 3 - theatreChase
  { UN,  25,  64,   0, DEF_Q,   5}, // 4 - rainbow
  {YEL,  25,  64,   0, DEF_Q,   5}, // 5 - theatreChaseRainbow
  {PUR,  25,  64,   0, DEF_Q,   1}, // 6 - colorBar
  {YEL, 250,  64,   0, DEF_Q,  50}, // 7 - colorBarFlash
  {BLU,  50,  64,   0, DEF_Q,  10}, // 8 - bounce
  {PUR,  25,  64,   0, DEF_Q,   5},  // 9 - bounceWipe
  {PUR,  25,  64,   0, DEF_Q,   5}, // 10 - MultiBounce
  {PUR,  25,  64,   0, DEF_Q,   5}, // 11 - MultiBounceWipe
  {PUR,  25,  64,   0, DEF_Q,   1}  // 12 - multiZoneBar
  
};

typedef enum {zone0, zone1, zone2, NUM_ZONES};

int zone_starts[NUM_ZONES] = {0, 40, 80};
int zone_ends[NUM_ZONES] = {39, 79, 119};
 

// ----------------------------------------------------------------------------- //
// setup
// This runs once at reset time
// ----------------------------------------------------------------------------- //

void setup() {
  // initialize serial:
  Serial.begin(9600);
  
  // Initialize the I/O pin used for the status LED
  pinMode(STATUS_PIN, OUTPUT);
  
  // Initialize the neopixel array
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  // Global variable initialization
  runningFunction = CLEAR;
  configFunction  = CLEAR;
  repeatCount     = blingParmsTable[CLEAR].repeat;
  
}

// ----------------------------------------------------------------------------- //
// loop
// This, and serialEvent below, are run one after another in an endless loop
// ----------------------------------------------------------------------------- //

void loop() {
  // Give the outside world some idea of what we are up to ...
  doBlink();
    
  if ((!commandFlag) && (repeatCount > 0)) {
  // No command reception/processing is occurring so we are OK to go ahead
  // and run a bling function
      doBling();
      repeatCount--;
  } else {
      if (!doneSent) {
        if (serialDebugEnabled) {
          Serial.println("Ready");
        } else {
          Serial.print(RESPONSE_STR);
        }
        doneSent = true; 
      }
  } // end !commandFlag ...
} 

// ----------------------------------------------------------------------------- //
// serialEvent
// A serial event occurs whenever new data comes in the
// hardware serial RX.  This routine is run between each
// time loop() runs, so using delay inside loop can delay
// response.  Multiple bytes of data may be available.
// ----------------------------------------------------------------------------- //

void serialEvent() { 
  while (Serial.available()) {
     commandChar = Serial.read();   
  
     if (CMD_CHAR == commandChar) {
       commandFlag = true;
       
       if (serialDebugEnabled) {
         Serial.println("Ready");
       } else {
         Serial.print(RESPONSE_STR);
       }

       doneSent  = true;
       continue;
     }   
     
     // ToDo: shorten the timeout that parseInt uses so that
     // we don't wait so long for the end of a number string if
     // a human tester forgets to put in a termination char
     commandVal  = Serial.parseInt();
     processCommand(commandChar, commandVal);
     
     // Peek at the next character. If it is a "Z" we are
     // done.
     // ToDo: we can make this protocol more robust by assuming that
     // any noncommand char is a termination char
     char foo = Serial.peek();

     if ('Z' == foo)
      {
        // We need to read the char to get it out of the buffer *and*
        // continue the loop so that Serial.available runs and returns
        // 0 (otherwise the "Z" character will still be the first 
        // char in the buffer then next time we get into this routine -
        // despite what the docs say)
        Serial.read();
        commandFlag = false;
      } // end if "end" char
  } // end while Serial.available 
}

// ----------------------------------------------------------------------------- //
// doBling
// This function simply looks at the currentRunning function (as set by the most 
// recent "E" command and executes the approriate function
// ----------------------------------------------------------------------------- //

void doBling() {
  
  serialStatusShow(RESPONSE_CHAR);
  // Set the staus LED on to indicate that we are running a bling function
  
  digitalWrite(STATUS_PIN, HIGH);
  
  // Set the brightness for the function about to be called. We don't apply
  // brightness changes for CLEAR becaue we may not be clearing all of the
  // pixels so we don't want to effect the brightness of the ones left uncleared
  if (CLEAR != runningFunction) {
    strip.setBrightness(blingParmsTable[runningFunction].brightness);
  }

  // Always start with a blank LED strip
  colorBar (0, 0, MAX_Q);
  
  switch (runningFunction)
  {
    case COLOR_WIPE:
       colorWipe(blingParmsTable[runningFunction].color, 
                 blingParmsTable[runningFunction].delay,
                 blingParmsTable[runningFunction].pixelStart,
                 blingParmsTable[runningFunction].pixelEnd);
       break;
    case COLOR_WIPE_WITH_BLANK:
       colorWipeWithBlank(blingParmsTable[runningFunction].color, 
                          blingParmsTable[runningFunction].delay,
                          blingParmsTable[runningFunction].pixelStart,
                          blingParmsTable[runningFunction].pixelEnd);
       break;
    case THEATRE_CHASE:
       theatreChase(blingParmsTable[runningFunction].color,
                    blingParmsTable[runningFunction].delay,
                    blingParmsTable[runningFunction].pixelStart,
                    blingParmsTable[runningFunction].pixelEnd);
       break;
    case RAINBOW:
       rainbow(blingParmsTable[runningFunction].delay,
               blingParmsTable[runningFunction].pixelStart,
               blingParmsTable[runningFunction].pixelEnd);
       break;       
    case THEATRE_CHASE_RAINBOW:
       theatreChaseRainbow(blingParmsTable[runningFunction].delay,
                           blingParmsTable[runningFunction].pixelStart,
                           blingParmsTable[runningFunction].pixelEnd);  
       break;
    case CLEAR:
    case COLOR_BAR:
       colorBar(blingParmsTable[runningFunction].color,
                blingParmsTable[runningFunction].pixelStart,
                blingParmsTable[runningFunction].pixelEnd);
       break; 
    case COLOR_BAR_FLASH:
       colorBarFlash(blingParmsTable[runningFunction].color,
                     blingParmsTable[runningFunction].pixelStart,
                     blingParmsTable[runningFunction].pixelEnd,
                     blingParmsTable[runningFunction].delay);
       break; 
    case BOUNCE:
       bounce(blingParmsTable[runningFunction].color,
              blingParmsTable[runningFunction].pixelStart,
              blingParmsTable[runningFunction].pixelEnd,
              blingParmsTable[runningFunction].delay);
       break; 
    case BOUNCE_WIPE:
      bounceWipe(blingParmsTable[runningFunction].color,
              blingParmsTable[runningFunction].pixelStart,
              blingParmsTable[runningFunction].pixelEnd,
              blingParmsTable[runningFunction].delay);
              
       break;
    case MULTI_BOUNCE:
      multiBounce(blingParmsTable[runningFunction].color,
              blingParmsTable[runningFunction].pixelStart,
              blingParmsTable[runningFunction].pixelEnd,
              blingParmsTable[runningFunction].delay);
              
       break;
    case MULTI_BOUNCE_WIPE:
      multiBounceWipe(blingParmsTable[runningFunction].color,
              blingParmsTable[runningFunction].pixelStart,
              blingParmsTable[runningFunction].pixelEnd,
              blingParmsTable[runningFunction].delay);

       break;
    case MULTI_COLOR_WIPE:
      multiZoneBar(blingParmsTable[runningFunction].color,
              blingParmsTable[runningFunction].pixelStart,
              blingParmsTable[runningFunction].pixelEnd,
              blingParmsTable[runningFunction].delay);
    default:
       break;
  } // end switch
}

// ----------------------------------------------------------------------------- //
// processCommand
// Handle a single command/value pair received on the serial line
// ----------------------------------------------------------------------------- //

void processCommand(char cmdChar, uint32_t cmdVal) {
  // Debug
  // Serial.print(cmdChar);
  // Serial.print(" ");
  // Serial.println(cmdVal);
  
  switch (cmdChar) {
    case 'B':
      // Set the brightness for the function being configured
      blingParmsTable[configFunction].brightness = cmdVal;
      break;
    case 'C':
      // Set the color for the function being configured
      // (not all functions actually use this parameter)
      blingParmsTable[configFunction].color = cmdVal;
      break;
    case 'D':
      // Set the delay for the function being configured
      // (not all functions actually use this parameter)
      blingParmsTable[configFunction].delay = cmdVal;
      break;
    case 'E':
      // Select the bling function to execute
      runningFunction = (function_t)cmdVal;
      repeatCount = blingParmsTable[runningFunction].repeat;
      break;
    case 'F':
      // Select the bling function to configure with any following
      // configurations commands (B, C, D, etc.)
      configFunction = (function_t)cmdVal;
      break;
    case 'P':
      // Set the pixel count for the function being configured
      // (not all functions actually use this parameter)
      if (cmdVal > STRIP_LEN) {
        cmdVal = STRIP_LEN;
      }
      blingParmsTable[configFunction].pixelStart = cmdVal;
      break;
    case 'Q':
      // Set the pixel count for the function being configured
      // (not all functions actually use this parameter)
      if (cmdVal > STRIP_LEN) {
        cmdVal = STRIP_LEN;
      }
      blingParmsTable[configFunction].pixelEnd = cmdVal;
      break;
    case RESPONSE_CHAR:
      // Set the repeat count for the function being configured
      blingParmsTable[configFunction].repeat = cmdVal;
      break;
    default: 
      if (serialDebugEnabled) {
        Serial.print("Unrecognized cmdChar: <");
        Serial.print(cmdChar);
        Serial.println(">");
      }
  } // end switch(cmdChar)
  
    serialStatusShow('C');
    doneSent = false;
}

// ToDo: proper head comment block
void doBlink() {
  // check to see if it's time to blink the LED; that is, if the 
  // difference between the current time and last time you blinked 
  // the LED is bigger than the interval at which you want to 
  // blink the LED.
  unsigned long currentMillis = millis();
 
  if (currentMillis - prevLedMillis > ledInterval) {
    // save the last time you blinked the LED 
    prevLedMillis = currentMillis;   

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
      
    } else {
      
      ledState = LOW;
    }
    // set the LED with the ledState of the variable:
    digitalWrite(STATUS_PIN, ledState);
  }
}

// ----------------------------------------------------------------------------- //
// serialStatusShow
// ToDo - clean up these comments
// This is an all-purpose status output routine to let the outside world know 
// what we are doing. With debugEnabled == true, it signals the outside world 
// via an LED on pin 8 as follows:
//  - runing bling function: LED is ON
//  - awaiting a command string on the serial line: flashing once per second
// With debugEnabled == true, the settings of all of the configurable parameters 
// are output to the serial console once per second
// ----------------------------------------------------------------------------- //

void serialStatusShow (const char prefix)
{      
    if (serialDebugEnabled) {
      // Show the current running function and its configuration
      Serial.print(prefix);
      Serial.print(": E=");
      Serial.print(runningFunction);
      Serial.print(" B=");
      Serial.print(blingParmsTable[runningFunction].brightness);
      Serial.print(" C=");
      Serial.print(blingParmsTable[runningFunction].color);
      Serial.print(" D=");
      Serial.print(blingParmsTable[runningFunction].delay);
      Serial.print(" P=");
      Serial.print(blingParmsTable[runningFunction].pixelStart);
      Serial.print(" Q=");
      Serial.print(blingParmsTable[runningFunction].pixelEnd);
      Serial.print(" R=");
      Serial.print(blingParmsTable[runningFunction].repeat);
      Serial.print(" ");
      Serial.println(repeatCount);
      
      // Next line ...
      Serial.print("   F=");
      Serial.print(configFunction);
      Serial.print(" B=");
      Serial.print(blingParmsTable[configFunction].brightness);
      Serial.print(" C=");
      Serial.print(blingParmsTable[configFunction].color);
      Serial.print(" D=");
      Serial.print(blingParmsTable[configFunction].delay);
      Serial.print(" P=");
      Serial.print(blingParmsTable[configFunction].pixelStart);
      Serial.print(" Q=");
      Serial.print(blingParmsTable[configFunction].pixelEnd);
      Serial.print(" R=");
      Serial.println(blingParmsTable[configFunction].repeat);
    }
  }

// ----------------------------------------------------------------------------- //
// LCDStatusShow
// ToDo - clean up these comments
// This is an all-purpose status output routine to let the outside world know 
// what we are doing. It is nearly identical to serialStatusShow - it has slight 
// format differences and outputs to an attached TFT LCD touchscreen rather than
// to the serial port.
// With debugEnabled == true, it signals the outside world 
// via an LED on pin 8 as follows:
//  - runing bling function: LED is ON
//  - awaiting a command string on the serial line: flashing once per second
// With debugEnabled == true, the settings of all of the configurable parameters 
// are output to the serial console once per second
// ----------------------------------------------------------------------------- //

// ----------------------------------------------------------------------------- //
// colorWipe
// Fill the dots one after the other with a color but use a breakable
// delay to enable faster response to command requests from the roboRIO
// ----------------------------------------------------------------------------- //

void colorWipe(uint32_t c, uint8_t wait, uint16_t pixelStart, uint16_t pixelEnd){
  for(uint16_t i=pixelStart; i<pixelEnd; i++) {
      strip.setPixelColor(i, c);
      strip.show();
      if (delayWithBreak (wait)) {
        return;
      }
  } // for
  return;
}

// ----------------------------------------------------------------------------- //
// colorWipeWithBlank
// Clear the strip then...
// Fill the dots one after the other with a color but use a breakable
// delay to enable faster response to command requests from the roboRIO
// ----------------------------------------------------------------------------- //

void colorWipeWithBlank(uint32_t c, uint8_t wait, uint16_t pixelStart, uint16_t pixelEnd) {
  for(uint16_t i=pixelStart; i<pixelEnd; i++) {
      strip.setPixelColor(i, 0);
  } // end for i
  strip.show();

  
  for(uint16_t i=pixelStart; i<pixelEnd; i++) {
      strip.setPixelColor(i, c);
      strip.show();
      if (delayWithBreak (wait)) {
        return;
      }
  } // end for i
  return;
}

// ----------------------------------------------------------------------------- //
// theatreChase
// Theatre-style crawling lights
// delay to enable faster response to command requests from the roboRIO 
// ----------------------------------------------------------------------------- //

void theatreChase(uint32_t c, uint8_t wait, uint16_t pixelStart, uint16_t pixelEnd) {
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (int i=pixelStart; i < pixelEnd; i=i+3) {
        strip.setPixelColor(i+q, c);    //turn every third pixel on
      }
      strip.show();
      if (delayWithBreak (wait)) {
        return;
      }
     
      for (int i=pixelStart; i < pixelEnd; i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    } // end for q
  } // end for j
}

// ----------------------------------------------------------------------------- //
// rainbow
// *Insert discription of rainbow*
// ----------------------------------------------------------------------------- //

void rainbow(uint8_t wait, uint16_t pixelStart, uint16_t pixelEnd) {
  uint16_t i, j;

  for(j=0; j<1256; j++) {
    for(i=pixelStart; i<pixelEnd; i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    if (delayWithBreak (wait)) {
      return;
    }
  } // end for j
}

// ----------------------------------------------------------------------------- //
// Wheel
// ----------------------------------------------------------------------------- //

uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else if(WheelPos < 170) {
    WheelPos -= 85;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}

// ----------------------------------------------------------------------------- //
// theatreChaseRainbow
// Theatre-style crawling lights with rainbow effect
// delay to enable faster response to command requests from the roboRIO 
// ----------------------------------------------------------------------------- //

void theatreChaseRainbow(uint8_t wait, uint16_t pixelStart, uint16_t pixelEnd) {
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
        for (int i=pixelStart; i < pixelEnd; i=i+3) {
          strip.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
        }
        strip.show();
        if (delayWithBreak (wait)) {
          return;
        }
       
        for (int i=pixelStart; i < pixelEnd; i=i+3) {
          strip.setPixelColor(i+q, 0);        //turn every third pixel off
        }
    }
  }
}

// ----------------------------------------------------------------------------- //
// colorBar
// ----------------------------------------------------------------------------- //

void colorBar(uint32_t c, uint16_t pixelStart, uint16_t pixelEnd) {
  for(uint16_t i=pixelStart; i<pixelEnd; i++) {
      strip.setPixelColor(i, c);
  } 
  strip.show();
  return;
}

// ----------------------------------------------------------------------------- //
// colorBarFlash
// ----------------------------------------------------------------------------- //

void colorBarFlash(uint32_t c, uint16_t pixelStart, uint16_t pixelEnd, uint16_t wait) {
  
  uint32_t color;
  
  for(uint16_t j=0; j<100; j++) {
    if(j%2) {
      color = c;
    } else {
      color = 0;
    }
    
    for(uint16_t i=pixelStart; i<pixelEnd; i++) {
        strip.setPixelColor(i, color);
    } 
    
    if (delayWithBreak(wait)) {
      break;
    }
    strip.show();
  }
  return;
}
// ----------------------------------------------------------------------------- //
// bounce
// ----------------------------------------------------------------------------- //

void bounce(uint32_t color, uint16_t pixelStart, uint16_t pixelEnd, uint16_t wait) {
  uint16_t a, b, c, d = 0;
  
  a = pixelStart;
  
  if ((pixelEnd - pixelStart)%2) {
    d = pixelEnd;
  } else {
    d = pixelEnd - 1;
  }
    
  uint16_t count = (d - a)/2  + 1;
  
  
  for (uint16_t i = 0; i < count; i++) {
    strip.setPixelColor(a, color);
    strip.setPixelColor(d, color);
    strip.show();

    if (delayWithBreak(wait)) {
      break;
    }
    
    strip.setPixelColor(a, 0);
    strip.setPixelColor(d, 0);
    strip.show();

    a++;
    d--;
   
  }
  
  for (uint16_t i = 0; i < count; i++) {
    a--;
    d++;
    strip.setPixelColor(a, color);
    strip.setPixelColor(d, color);
    strip.show();

    if (delayWithBreak(wait)) {
      break;
    }
    strip.setPixelColor(a, 0);
    strip.setPixelColor(d, 0);
    strip.show();
  }

}

// ----------------------------------------------------------------------------- //
// bounceWipe
// ----------------------------------------------------------------------------- //

void bounceWipe(uint32_t color, uint16_t pixelStart, uint16_t pixelEnd, uint16_t wait) {
  uint16_t a, b, c, d = 0;
  
  a = pixelStart;
  
  if ((pixelEnd - pixelStart)%2) {
    d = pixelEnd;
  } else {
    d = pixelEnd - 1;
  }
    
  uint16_t count = (d - a)/2  + 1;
  
  
  for (uint16_t i = 0; i < count; i++) {
    strip.setPixelColor(a, color);
    strip.setPixelColor(d, color);
    strip.show();

    if (delayWithBreak(wait)) {
      break;
    }
    
    strip.setPixelColor(a, 0);
    strip.setPixelColor(d, 0);
    strip.show();

    a++;
    d--;
   
  }
  
  for (uint16_t i = 0; i < count; i++) {
    a--;
    d++;
    strip.setPixelColor(a, color);
    strip.setPixelColor(d, color);
    strip.show();

    if (delayWithBreak(wait)) {
      break;
    }
    strip.setPixelColor(a, color);
    strip.setPixelColor(d, color);
    strip.show();
  }

}


// ----------------------------------------------------------------------------- //
// multiBounce
// ----------------------------------------------------------------------------- //

void multiBounce(uint32_t color, uint16_t pixelStart, uint16_t pixelEnd, uint16_t wait) {
  uint16_t a, b, c, d = 0;
  uint16_t a1, b1, c1, d1 = 0;
  uint16_t a2, b2, c2, d2 = 0;
 
  //uint16_t count = (d - a)/2  + 1;
  uint16_t count = (zone_ends[0] - zone_starts[0])/2 + 1;
  
  for (uint16_t i = 0; i < count; i++) {
    for (uint16_t j = 0; j < NUM_ZONES; j++) {
       strip.setPixelColor(zone_starts[j] + i, color);
       strip.setPixelColor(zone_ends[j] - i, color);
    }
  
    strip.show();

    if (delayWithBreak(wait)) {
      break;
    }

    for (uint16_t j = 0; j < NUM_ZONES; j++) {
       strip.setPixelColor(zone_starts[j] + i, 0);
       strip.setPixelColor(zone_ends[j] - i, 0);
    }
   
    strip.show();
  }
  
  for (uint16_t i = count; i > 0; --i) {
    for (uint16_t j = 0; j < NUM_ZONES; j++) {
       strip.setPixelColor(zone_starts[j] + i, color);
       strip.setPixelColor(zone_ends[j] - i, color);
    }
    
    strip.show();

    if (delayWithBreak(wait)) {
      break;
    }

    for (uint16_t j = 0; j < NUM_ZONES; j++) {
       strip.setPixelColor(zone_starts[j] + i, 0);
       strip.setPixelColor(zone_ends[j] - i, 0);
    }

    strip.show();
  }
}

// ----------------------------------------------------------------------------- //
// multiBounceWipe
// ----------------------------------------------------------------------------- //

void multiBounceWipe(uint32_t color, uint16_t pixelStart, uint16_t pixelEnd, uint16_t wait) {

  uint16_t count = (zone_ends[0] - zone_starts[0])/2 + 1;
  
  for (uint16_t i = 0; i < count; i++) {

    for (uint16_t j = 0; j < NUM_ZONES; j++) {
         strip.setPixelColor(zone_starts[j] + i, color);
         strip.setPixelColor(zone_ends[j] - i, color);
    }

    strip.show();

    if (delayWithBreak(wait)) {
      break;
    }
  }
  
  for (uint16_t i = count; i > 0; --i) {

    for (uint16_t j = 0; j < NUM_ZONES; j++) {
       strip.setPixelColor(zone_starts[j] + i, 0);
       strip.setPixelColor(zone_ends[j] - i, 0);
    }

    strip.show();
    

    if (delayWithBreak(wait)) {
      break;
    }
  }
}

// ----------------------------------------------------------------------------- //
// multiZoneBar
// Fill from the edges to the center of each zone and then clear out again in
// the opposite direction (from center to edges)
// ----------------------------------------------------------------------------- //

void multiZoneBar(uint32_t c, uint16_t pixelStart, uint16_t pixelEnd, uint8_t wait){
  // Clear
  for(uint16_t i=pixelStart; i<pixelEnd; i++) {
      strip.setPixelColor(i, 0);
  } // end for i
  strip.show();
  
  // We are interpreting the wait value as a percentage (0 - 100)
  // and we will fill that percentage of the zone. The  following 
  // computation assumes that all zones are the same size
  uint16_t delta = (zone_ends[0] - zone_starts[0] + 1);
  uint16_t numPixels = delta * wait / 100;

  if ((numPixels * 100) != (delta * wait)) {
    numPixels++;
  }
  
  for (uint16_t j = 0; j < NUM_ZONES; j++) {
    for(uint16_t i=zone_starts[j]; i<zone_starts[j] + numPixels; i++) {
        strip.setPixelColor(i, c);
    }
  } 
  strip.show();
 
}
// ----------------------------------------------------------------------------- //
// delayWithBreak
// A delay function which checks the serial device every millisecond to
// see if there is a command request character ("X"). If one is seen
// we exit the delay and signal the caller that we have ended early
// ----------------------------------------------------------------------------- //

boolean delayWithBreak (uint32_t count)
{
  uint32_t localCount = 0;
  
  for (localCount = 0; localCount < count; localCount++) {
    delay (1);
    
    // Note that we peek here (not read) so that the char is still
    // in the serial buffer and can be seen and read as usual by
    // the serialEvent routine
    char tmp = Serial.peek();

     if (CMD_CHAR == tmp)
      {
        return true;
      }
  } // end for
  return false;
}
