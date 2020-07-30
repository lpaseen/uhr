/****************************************************************
    uhr box
    Electronic simulation of the uhr box that luftwaffe used with their enigma machine

    Copyright: Peter Sjoberg <peters-enigma AT techwiz DOT ca>+
    License: GPLv3
      This program is free software: you can redistribute it and/or modify
      it under the terms of the GNU General Public License version 3 as
      published by the Free Software Foundation.

      This program is distributed in the hope that it will be useful,
      but WITHOUT ANY WARRANTY; without even the implied warranty of
      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
      GNU General Public License for more details.

      You should have received a copy of the GNU General Public License
      along with this program.  If not, see <http://www.gnu.org/licenses/>.


  History:
    v0.00 - a first version to test the basic concept
    v0.02 - added code for a dual 7seg display
    v0.03 - added code for encoder


  Status:
  Done:
    led and encoder works
    plugboard connection is tested and works
  ToDo:
    uhr mapping does not work correctly, need to get the mapping
    between in and out corrected, probably need to add some some more lookup tables

*/


#include <SPI.h>
#include <Wire.h>


/****************************************************************/
// from https://github.com/PaulStoffregen/TimerOne
// also at https://www.pjrc.com/teensy/td_libs_TimerOne.html
#include <TimerOne.h>

uint8_t LEDpattern[] = {
  //    .abcdefg
  0b01111110,  // 0
  0b00110000,  // 1
  0b01101101,  // 2
  0b01111001,  // 3
  0b00110011,  // 4
  0b01011011,  // 5
  0b01011111,  // 6
  0b01110010,  // 7
  0b01111111,  // 8
  0b01111011,  // 9
  // #+10 => decimal point
  0b11111110,  // 0.
  0b10110000,  // 1.
  0b11101101,  // 2.
  0b11111001,  // 3.
  0b10110011,  // 4.
  0b11011011,  // 5.
  0b11011111,  // 6.
  0b11110010,  // 7.
  0b11111111,  // 8.
  0b11111011,  // 9.
  //
  0b00000000,   // blank (20)
  0b10000000  // . (21)
};


#define CCD 

#define MAXLED 2
static uint8_t LEDBuffer[MAXLED] = {20, 20};
static uint8_t LEDassembly[MAXLED] = {20, 20};

#ifdef CC
//common cathode - no driver
static uint8_t CommonActive = LOW;
static uint8_t CommonInactive = HIGH;
static uint8_t SegmentActive = HIGH;
static uint8_t SegmentInactive = LOW;
#elif defined(CCD)
//common cathode - transistor driver
static uint8_t CommonActive = HIGH;
static uint8_t CommonInactive = LOW;
static uint8_t SegmentActive = HIGH;
static uint8_t SegmentInactive = LOW;
#elif defined(CA)
//common anode
static uint8_t CommonActive = HIGH;
static uint8_t CommonInactive = LOW;
static uint8_t SegmentActive = LOW;
static uint8_t SegmentInactive = HIGH;
#else
#error need CC or CA defined
#endif

//How the led display is connected
//                    digit0, digit1
const uint8_t Common[] = {A3, A2};
//                           dp, a   b   c   d   e   f   g
const uint8_t Segment[] = {   6, 13, 12, 11, 10,  9,  8,  7};

/****************************************************************/
//static const uint8_t encoderPins[] = {2,3}; // external interrupt is only available on 2 and 3
static const uint8_t encoderPins[] = {4, 5}; //
volatile uint8_t encoderState = 0xff;
volatile unsigned long encoderChange = 0;
volatile boolean encoderMoved = false;

/****************************************************************/
//  MCP23017 registers, all as seen from bank0
//
#define mcp_address 0x20 // I2C Address of MCP23017
#define IODIRA    0x00 // IO Direction Register Address of Port A
#define IODIRB    0x01 // IO Direction Register Address of Port B
#define IPOLA     0x02 // Input polarity port register 
#define IPOLB     0x03 // 
#define GPINTENA  0x04 // Interrupt on change
#define GPINTENB  0x05 // 
#define DEFVALA   0x06 // Default value register
#define DEFVALB   0x07 // 
#define INTCONA   0x08 // Interrupt on change control register
#define INTCONB   0x09 // 
#define IOCON     0x0A // Control register
#define IOCON     0x0B // 
#define GPPUA     0x0C // GPIO Pull-ip resistor register
#define GPPUB     0x0D // 
#define INTFA     0x0E // Interrupt flag register
#define INTFB     0x0F // 
#define INTCAPA   0x10 // Interrupt captred value for port register
#define INTCAPB   0x11 // 
#define GPIOA     0x12 // General purpose io register
#define GPIOB     0x13 // 
#define OLATA     0x14 // Output latch register
#define OLATB     0x15 // 

#define BIT(val, i) ((val >> i) & 1)

//static const uint8_t pbLookup[26][3] PROGMEM = {
static const uint8_t pbLookup[32][3] = { // could make it PROGMEM but then the code below gets bigger instead
  //mcp=i2c address,port=0 or 1,bit=0 to 7
  {mcp_address, 0, 0},
  {mcp_address, 0, 1},
  {mcp_address, 0, 2},
  {mcp_address, 0, 3},
  {mcp_address, 0, 4},
  {mcp_address, 0, 5},
  {mcp_address, 0, 6},
  {mcp_address, 0, 7},
  {mcp_address, 1, 0},
  {mcp_address, 1, 1},
  {mcp_address, 1, 2},
  {mcp_address, 1, 3},
  {mcp_address, 1, 4},
  {mcp_address, 1, 5},
  {mcp_address, 1, 6},
  {mcp_address, 1, 7},
  {mcp_address + 1, 0, 0},
  {mcp_address + 1, 0, 1},
  {mcp_address + 1, 0, 2},
  {mcp_address + 1, 0, 3},
  {mcp_address + 1, 0, 4},
  {mcp_address + 1, 0, 5},
  {mcp_address + 1, 0, 6},
  {mcp_address + 1, 0, 7},
  {mcp_address + 1, 1, 0},
  {mcp_address + 1, 1, 1},
  {mcp_address + 1, 1, 2},
  {mcp_address + 1, 1, 3},
  {mcp_address + 1, 1, 4},
  {mcp_address + 1, 1, 5},
  {mcp_address + 1, 1, 6},
  {mcp_address + 1, 1, 7},
};

uint8_t valA[4];

//How the plugboard is mapped
//[0] is first input port on first mcp23017
// (char)pgm_read_byte(&steckerbrett[0]+key-1)
//with steckerbrett as A-Z the plugboard ends up as
// ABCDEFGHI
//  JCKLMNOPQ
// QRSTUVWXYZ
// or
//  Q   W   E   R   T   Z   U   I   O
//  0   1   2   3   4   4   6   7   8
//
//    A   S   D   F   G   H   J   K
//    9  10  11  12  13  14  15  16
//
//  P   Y   X   C   V   B   N   M   L
// 17  18  19  20  21  22  23  24  25
//
//
//                                     00000000001111111111222222
//                                     01234567890123456789012345
const byte steckerbrett[] PROGMEM =   "QWERTZUIOASDFGHJKPYXCVBNML"; //
//const byte steckerbrett[] PROGMEM = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"; //

// naming convention:
//  red/outer are the red plugs that are connected to the outer ring
//  white/inner are the white (or black) plugs connected to the inner ring
//  the logic see the signal flow to go from left to right, from red to white so "IN"=red and "OUT" = white, even if the active signal actually is on the right/"IN" side


// Uhrkontakt (r) 00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39
// Uhrkontakt (w) 26 11 24 21 02 31 00 25 30 39 28 13 22 35 20 37 06 23 04 33 34 19 32 09 18 07 16 17 10 03 08 01 38 27 36 29 14 15 12 05

// Red plugs (a) goes to outer ring
// white plugs (b) goes to inner ring

//
//                                      0   1   2   3   4   5   6   7   8   9
const uint8_t UHRIN[]      PROGMEM = {  0,  4,  8, 12, 16, 20, 24, 28, 32, 36}; // the red "a" plugs
const uint8_t UHROUT[]     PROGMEM = {  4, 16, 28, 36, 24, 12,  0,  8, 20, 32}; // the white "b" plugs
//
//                                     00  01  02  03  04  05  06  07  08  09  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39
const uint8_t UHROUTER[]   PROGMEM = {  6, 31,  4, 29, 18, 39, 16, 25, 30, 23, 28,  1, 38, 11, 36, 37, 26, 27, 24, 21, 14,  3, 12, 17,  2,  7,  0, 33, 10, 35,  8,  5, 22, 19, 20, 13, 34, 15, 32,  9};
const uint8_t UHRINNER[]   PROGMEM = { 26, 11, 24, 21,  2, 31,  0, 25, 30, 39, 28, 13, 22, 35, 20, 37,  6, 23,  4, 33, 34, 19, 32,  9, 18,  7, 16, 17, 10,  3,  8,  1, 38, 27, 36, 29, 14, 15, 12,  5};
//
//                                      0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19 20  21  22  23  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39
const uint8_t UHRIN_REV[]  PROGMEM = {  0, 99,  0, 99,  1, 99,  1, 99,  2, 99,  2, 99,  3, 99,  3, 99,  4, 99,  4, 99, 5, 99,  5, 99,  6, 99,  6, 99,  7, 99,  7, 99,  8, 99,  8, 99,  9, 99,  9, 99};
const uint8_t UHROUT_REV[] PROGMEM = {  6, 99,  6, 99,  0, 99,  0, 99,  7, 99,  7, 99,  5, 99,  5, 99,  1, 99,  1, 99, 8, 99,  8, 99,  4, 99,  4, 99,  2, 99,  2, 99,  9, 99,  9, 99,  3, 99,  3, 99};
static uint8_t uhrpos = 0;

// plug r/w   size  ring  pos
// 1    red   thick outer 0
// 1    red   thin  outer 1
// 1    white thick inner 2
// 1    white thin  inner 3
// 2    red   thick outer 2
// 2    red   thin  outer 3
// 2    white thick inner 8
// 2    white thin  inner 9
// 3    red   thick outer 4
// 3    red   thin  outer 5
// 3    white thick inner 14
// 3    white thin  inner 15
// 4    red   thick outer 6
// 4    red   thin  outer 7
// 4    white thick inner 18
// 4    white thin  inner 19
// 5    red   thick outer 8
// 5    red   thin  outer 9
// 5    white thick inner 12
// 5    white thin  inner 13
// 6    red   thick outer 10
// 6    red   thin  outer 11
// 6    white thick inner 6
// 6    white thin  inner 7
// 7    red   thick outer 12
// 7    red   thin  outer 13
// 7    white thick inner 0
// 7    white thin  inner 1
// 8    red   thick outer 14
// 8    red   thin  outer 15
// 8    white thick inner 4
// 8    white thin  inner 5
// 9    red   thick outer 16
// 9    red   thin  outer 17
// 9    white thick inner 10
// 9    white thin  inner 11
// 10   red   thick outer 18
// 10   red   thin  outer 19
// 10   white thick inner 16
// 10   white thin  inner 17


//lookup table for what bits that are cleared. It is an error to have more than 2 bits cleared
// 256 locations mapped to what one or two bits that are set
// bit 9 = NA or not valid
// $ perl -e 'sub popcnt{ my $val=shift; my $cnt=0;for my $i (0..7){if (($val & 1<<$i)==0){$cnt++;};}; return $cnt;};for my $y (0..255) { my $bin=sprintf "%08b",$y;my $bits=popcnt($y);if ($bits==2){print "{ , ";}elsif ($bits==1){print "{ ,0";}else{print "{0,0";};printf "}, // # %3d=%8s,bits=%d\n",$y,$bin,$bits;}'
//
// $ perl -e 'sub popcnt{ my $val=shift; my $cnt=0;for my $i (0..7){if (($val & 1<<$i)==0){$cnt++;};}; return $cnt;};for my $y (0..255) { my $bin=sprintf "%08b",$y;my $bits=popcnt($y);if ($bits==2 || $bits==1){print "0b$bin";}else{print "0b00000000";};printf ", // # %3d=0x%02X,bits=%d\n",$y,$y,$bits;}'

const uint8_t BITMASK[] PROGMEM  = {
0b00000000, // #   0=0x00,bits=8
0b00000000, // #   1=0x01,bits=7
0b00000000, // #   2=0x02,bits=7
0b00000000, // #   3=0x03,bits=6
0b00000000, // #   4=0x04,bits=7
0b00000000, // #   5=0x05,bits=6
0b00000000, // #   6=0x06,bits=6
0b00000000, // #   7=0x07,bits=5
0b00000000, // #   8=0x08,bits=7
0b00000000, // #   9=0x09,bits=6
0b00000000, // #  10=0x0A,bits=6
0b00000000, // #  11=0x0B,bits=5
0b00000000, // #  12=0x0C,bits=6
0b00000000, // #  13=0x0D,bits=5
0b00000000, // #  14=0x0E,bits=5
0b00000000, // #  15=0x0F,bits=4
0b00000000, // #  16=0x10,bits=7
0b00000000, // #  17=0x11,bits=6
0b00000000, // #  18=0x12,bits=6
0b00000000, // #  19=0x13,bits=5
0b00000000, // #  20=0x14,bits=6
0b00000000, // #  21=0x15,bits=5
0b00000000, // #  22=0x16,bits=5
0b00000000, // #  23=0x17,bits=4
0b00000000, // #  24=0x18,bits=6
0b00000000, // #  25=0x19,bits=5
0b00000000, // #  26=0x1A,bits=5
0b00000000, // #  27=0x1B,bits=4
0b00000000, // #  28=0x1C,bits=5
0b00000000, // #  29=0x1D,bits=4
0b00000000, // #  30=0x1E,bits=4
0b00000000, // #  31=0x1F,bits=3
0b00000000, // #  32=0x20,bits=7
0b00000000, // #  33=0x21,bits=6
0b00000000, // #  34=0x22,bits=6
0b00000000, // #  35=0x23,bits=5
0b00000000, // #  36=0x24,bits=6
0b00000000, // #  37=0x25,bits=5
0b00000000, // #  38=0x26,bits=5
0b00000000, // #  39=0x27,bits=4
0b00000000, // #  40=0x28,bits=6
0b00000000, // #  41=0x29,bits=5
0b00000000, // #  42=0x2A,bits=5
0b00000000, // #  43=0x2B,bits=4
0b00000000, // #  44=0x2C,bits=5
0b00000000, // #  45=0x2D,bits=4
0b00000000, // #  46=0x2E,bits=4
0b00000000, // #  47=0x2F,bits=3
0b00000000, // #  48=0x30,bits=6
0b00000000, // #  49=0x31,bits=5
0b00000000, // #  50=0x32,bits=5
0b00000000, // #  51=0x33,bits=4
0b00000000, // #  52=0x34,bits=5
0b00000000, // #  53=0x35,bits=4
0b00000000, // #  54=0x36,bits=4
0b00000000, // #  55=0x37,bits=3
0b00000000, // #  56=0x38,bits=5
0b00000000, // #  57=0x39,bits=4
0b00000000, // #  58=0x3A,bits=4
0b00000000, // #  59=0x3B,bits=3
0b00000000, // #  60=0x3C,bits=4
0b00000000, // #  61=0x3D,bits=3
0b00000000, // #  62=0x3E,bits=3
0b00111111, // #  63=0x3F,bits=2
0b00000000, // #  64=0x40,bits=7
0b00000000, // #  65=0x41,bits=6
0b00000000, // #  66=0x42,bits=6
0b00000000, // #  67=0x43,bits=5
0b00000000, // #  68=0x44,bits=6
0b00000000, // #  69=0x45,bits=5
0b00000000, // #  70=0x46,bits=5
0b00000000, // #  71=0x47,bits=4
0b00000000, // #  72=0x48,bits=6
0b00000000, // #  73=0x49,bits=5
0b00000000, // #  74=0x4A,bits=5
0b00000000, // #  75=0x4B,bits=4
0b00000000, // #  76=0x4C,bits=5
0b00000000, // #  77=0x4D,bits=4
0b00000000, // #  78=0x4E,bits=4
0b00000000, // #  79=0x4F,bits=3
0b00000000, // #  80=0x50,bits=6
0b00000000, // #  81=0x51,bits=5
0b00000000, // #  82=0x52,bits=5
0b00000000, // #  83=0x53,bits=4
0b00000000, // #  84=0x54,bits=5
0b00000000, // #  85=0x55,bits=4
0b00000000, // #  86=0x56,bits=4
0b00000000, // #  87=0x57,bits=3
0b00000000, // #  88=0x58,bits=5
0b00000000, // #  89=0x59,bits=4
0b00000000, // #  90=0x5A,bits=4
0b00000000, // #  91=0x5B,bits=3
0b00000000, // #  92=0x5C,bits=4
0b00000000, // #  93=0x5D,bits=3
0b00000000, // #  94=0x5E,bits=3
0b01011111, // #  95=0x5F,bits=2
0b00000000, // #  96=0x60,bits=6
0b00000000, // #  97=0x61,bits=5
0b00000000, // #  98=0x62,bits=5
0b00000000, // #  99=0x63,bits=4
0b00000000, // # 100=0x64,bits=5
0b00000000, // # 101=0x65,bits=4
0b00000000, // # 102=0x66,bits=4
0b00000000, // # 103=0x67,bits=3
0b00000000, // # 104=0x68,bits=5
0b00000000, // # 105=0x69,bits=4
0b00000000, // # 106=0x6A,bits=4
0b00000000, // # 107=0x6B,bits=3
0b00000000, // # 108=0x6C,bits=4
0b00000000, // # 109=0x6D,bits=3
0b00000000, // # 110=0x6E,bits=3
0b01101111, // # 111=0x6F,bits=2
0b00000000, // # 112=0x70,bits=5
0b00000000, // # 113=0x71,bits=4
0b00000000, // # 114=0x72,bits=4
0b00000000, // # 115=0x73,bits=3
0b00000000, // # 116=0x74,bits=4
0b00000000, // # 117=0x75,bits=3
0b00000000, // # 118=0x76,bits=3
0b01110111, // # 119=0x77,bits=2
0b00000000, // # 120=0x78,bits=4
0b00000000, // # 121=0x79,bits=3
0b00000000, // # 122=0x7A,bits=3
0b01111011, // # 123=0x7B,bits=2
0b00000000, // # 124=0x7C,bits=3
0b01111101, // # 125=0x7D,bits=2
0b01111110, // # 126=0x7E,bits=2
0b01111111, // # 127=0x7F,bits=1
0b00000000, // # 128=0x80,bits=7
0b00000000, // # 129=0x81,bits=6
0b00000000, // # 130=0x82,bits=6
0b00000000, // # 131=0x83,bits=5
0b00000000, // # 132=0x84,bits=6
0b00000000, // # 133=0x85,bits=5
0b00000000, // # 134=0x86,bits=5
0b00000000, // # 135=0x87,bits=4
0b00000000, // # 136=0x88,bits=6
0b00000000, // # 137=0x89,bits=5
0b00000000, // # 138=0x8A,bits=5
0b00000000, // # 139=0x8B,bits=4
0b00000000, // # 140=0x8C,bits=5
0b00000000, // # 141=0x8D,bits=4
0b00000000, // # 142=0x8E,bits=4
0b00000000, // # 143=0x8F,bits=3
0b00000000, // # 144=0x90,bits=6
0b00000000, // # 145=0x91,bits=5
0b00000000, // # 146=0x92,bits=5
0b00000000, // # 147=0x93,bits=4
0b00000000, // # 148=0x94,bits=5
0b00000000, // # 149=0x95,bits=4
0b00000000, // # 150=0x96,bits=4
0b00000000, // # 151=0x97,bits=3
0b00000000, // # 152=0x98,bits=5
0b00000000, // # 153=0x99,bits=4
0b00000000, // # 154=0x9A,bits=4
0b00000000, // # 155=0x9B,bits=3
0b00000000, // # 156=0x9C,bits=4
0b00000000, // # 157=0x9D,bits=3
0b00000000, // # 158=0x9E,bits=3
0b10011111, // # 159=0x9F,bits=2
0b00000000, // # 160=0xA0,bits=6
0b00000000, // # 161=0xA1,bits=5
0b00000000, // # 162=0xA2,bits=5
0b00000000, // # 163=0xA3,bits=4
0b00000000, // # 164=0xA4,bits=5
0b00000000, // # 165=0xA5,bits=4
0b00000000, // # 166=0xA6,bits=4
0b00000000, // # 167=0xA7,bits=3
0b00000000, // # 168=0xA8,bits=5
0b00000000, // # 169=0xA9,bits=4
0b00000000, // # 170=0xAA,bits=4
0b00000000, // # 171=0xAB,bits=3
0b00000000, // # 172=0xAC,bits=4
0b00000000, // # 173=0xAD,bits=3
0b00000000, // # 174=0xAE,bits=3
0b10101111, // # 175=0xAF,bits=2
0b00000000, // # 176=0xB0,bits=5
0b00000000, // # 177=0xB1,bits=4
0b00000000, // # 178=0xB2,bits=4
0b00000000, // # 179=0xB3,bits=3
0b00000000, // # 180=0xB4,bits=4
0b00000000, // # 181=0xB5,bits=3
0b00000000, // # 182=0xB6,bits=3
0b10110111, // # 183=0xB7,bits=2
0b00000000, // # 184=0xB8,bits=4
0b00000000, // # 185=0xB9,bits=3
0b00000000, // # 186=0xBA,bits=3
0b10111011, // # 187=0xBB,bits=2
0b00000000, // # 188=0xBC,bits=3
0b10111101, // # 189=0xBD,bits=2
0b10111110, // # 190=0xBE,bits=2
0b10111111, // # 191=0xBF,bits=1
0b00000000, // # 192=0xC0,bits=6
0b00000000, // # 193=0xC1,bits=5
0b00000000, // # 194=0xC2,bits=5
0b00000000, // # 195=0xC3,bits=4
0b00000000, // # 196=0xC4,bits=5
0b00000000, // # 197=0xC5,bits=4
0b00000000, // # 198=0xC6,bits=4
0b00000000, // # 199=0xC7,bits=3
0b00000000, // # 200=0xC8,bits=5
0b00000000, // # 201=0xC9,bits=4
0b00000000, // # 202=0xCA,bits=4
0b00000000, // # 203=0xCB,bits=3
0b00000000, // # 204=0xCC,bits=4
0b00000000, // # 205=0xCD,bits=3
0b00000000, // # 206=0xCE,bits=3
0b11001111, // # 207=0xCF,bits=2
0b00000000, // # 208=0xD0,bits=5
0b00000000, // # 209=0xD1,bits=4
0b00000000, // # 210=0xD2,bits=4
0b00000000, // # 211=0xD3,bits=3
0b00000000, // # 212=0xD4,bits=4
0b00000000, // # 213=0xD5,bits=3
0b00000000, // # 214=0xD6,bits=3
0b11010111, // # 215=0xD7,bits=2
0b00000000, // # 216=0xD8,bits=4
0b00000000, // # 217=0xD9,bits=3
0b00000000, // # 218=0xDA,bits=3
0b11011011, // # 219=0xDB,bits=2
0b00000000, // # 220=0xDC,bits=3
0b11011101, // # 221=0xDD,bits=2
0b11011110, // # 222=0xDE,bits=2
0b11011111, // # 223=0xDF,bits=1
0b00000000, // # 224=0xE0,bits=5
0b00000000, // # 225=0xE1,bits=4
0b00000000, // # 226=0xE2,bits=4
0b00000000, // # 227=0xE3,bits=3
0b00000000, // # 228=0xE4,bits=4
0b00000000, // # 229=0xE5,bits=3
0b00000000, // # 230=0xE6,bits=3
0b11100111, // # 231=0xE7,bits=2
0b00000000, // # 232=0xE8,bits=4
0b00000000, // # 233=0xE9,bits=3
0b00000000, // # 234=0xEA,bits=3
0b11101011, // # 235=0xEB,bits=2
0b00000000, // # 236=0xEC,bits=3
0b11101101, // # 237=0xED,bits=2
0b11101110, // # 238=0xEE,bits=2
0b11101111, // # 239=0xEF,bits=1
0b00000000, // # 240=0xF0,bits=4
0b00000000, // # 241=0xF1,bits=3
0b00000000, // # 242=0xF2,bits=3
0b11110011, // # 243=0xF3,bits=2
0b00000000, // # 244=0xF4,bits=3
0b11110101, // # 245=0xF5,bits=2
0b11110110, // # 246=0xF6,bits=2
0b11110111, // # 247=0xF7,bits=1
0b00000000, // # 248=0xF8,bits=3
0b11111001, // # 249=0xF9,bits=2
0b11111010, // # 250=0xFA,bits=2
0b11111011, // # 251=0xFB,bits=1
0b11111100, // # 252=0xFC,bits=2
0b11111101, // # 253=0xFD,bits=1
0b11111110, // # 254=0xFE,bits=1
0b00000000  // # 255=0xFF,bits=0
};

/****************************************************************/
///
/// Write a single byte to i2c
///
uint8_t i2c_write(uint8_t unitaddr, uint8_t val) {
  Wire.beginTransmission(unitaddr);
  Wire.write(val);
  return Wire.endTransmission();
}

/****************************************************************/
///
///Write two bytes to i2c
///
uint8_t i2c_write2(uint8_t unitaddr, uint8_t val1, uint8_t val2) {
  Wire.beginTransmission(unitaddr);
  Wire.write(val1);
  Wire.write(val2);
  return Wire.endTransmission();
}

/****************************************************************/
///
/// read a byte from specific address (send one byte(address to read) and read a byte)
///
uint8_t i2c_read(uint8_t unitaddr, uint8_t addr) {
  i2c_write(unitaddr, addr);
  Wire.requestFrom(unitaddr, 1);
  return Wire.read();    // read one byte
}

/****************************************************************/
///
/// read 2 bytes starting at a specific address
/// read is done in two sessions - as required by mcp23017 (page 5 in datasheet)
///
uint16_t i2c_read2(uint8_t unitaddr, uint8_t addr) {
  uint16_t val;
  i2c_write(unitaddr, addr);
  Wire.requestFrom(unitaddr, 1);
  val = Wire.read();
  Wire.requestFrom(unitaddr, 1);
  return Wire.read() << 8 | val;
}

/****************************************************************/
///
/// print a 8bit hex number with leading 0
///
void printHex2(uint8_t val) {
  if (val < 0x10) {
    Serial.print(F("0"));
  };
  Serial.print(val, HEX);
} //printHex2

/****************************************************************/
///
/// print a 16bit hex number with leading 0
///
void printHex4(uint16_t val) {
  printHex2(val >> 8 & 0xFF);
  printHex2(val & 0xFF);
} //printHex4



/****************************************************************/
///
/// print a 8bit binary number with leading 0 and space at 8 bits.
///
void printBin8(uint8_t val) {
  int8_t bitno;
  for (bitno = 7; bitno >= 0; bitno--) {
    if (bitRead(val, bitno)) {
      Serial.print(F("1"));
    } else {
      Serial.print(F("0"));
    }
  }
  Serial.print(F(" "));
} //printBin8

/****************************************************************/
///
/// print a 16bit binary number with leading 0 and space at 8 bits.
///
void printBin16(uint16_t val) {
  printBin8(val >> 8 & 0xFF);
  printBin8(val & 0xFF);
} //printBin16


/****************************************************************/
///
/// Set a pin output and low
///
void setPortOut(uint8_t plug) {
  //make port "plug" output (and every other an input)
  i2c_write2(pbLookup[plug][0], IODIRA + pbLookup[plug][1], 0xff ^ (1 << pbLookup[plug][2]));
  //set  port "plug" low
  i2c_write2(pbLookup[plug][0], GPIOA + pbLookup[plug][1], 0xff ^ (1 << pbLookup[plug][2]));
}

/****************************************************************/
///
/// Set the port as input, will set other ports on same bus input also.
///
void setPortIn(uint8_t plug) {
  i2c_write2(pbLookup[plug][0], IODIRA + pbLookup[plug][1], 0xff);
  i2c_write2(pbLookup[plug][0], GPPUA  + pbLookup[plug][1], 0xff); // enable pullup
} // setPortIn

/****************************************************************/
///
/// Set the port as input, will set other ports on same bus input also.
///
void setPortInAll() {
  i2c_write2(mcp_address, IODIRA, 0xff); // Set all ports to inputs
  i2c_write2(mcp_address, IODIRB, 0xff); // Set all ports to inputs
  i2c_write2(mcp_address, GPPUA,  0xff); // enable pullup
  i2c_write2(mcp_address, GPPUB,  0xff); //
  i2c_write2(mcp_address + 1, IODIRA, 0xff); // Set all ports to inputs
  i2c_write2(mcp_address + 1, IODIRB, 0xff); // Set all ports to inputs
  i2c_write2(mcp_address + 1, GPPUA,  0xff); // enable pullup
  i2c_write2(mcp_address + 1, GPPUB,  0xff); //
} // setPortInAll

/****************************************************************/
//The logic is
//  when both inputs are high (11) it's in resting position so check what prev condition was
//   if A was low - right turn
//   if B was low - left turn
//  debounce - if less than x ms since last interrupt
//
void updateEncoderState() {
  static uint8_t i, state;

  //read state of the encoder
  state = digitalRead(encoderPins[1]) << 1 | digitalRead(encoderPins[0]);
  if ((encoderState & 0b11) != state) { // same as before ?
    // check time since last change to ignore bounces.
    // Test shown that turning fast at 1ms can still be correct while 6ms can still be bounce/wrong
    // going with high number since on the original Enigma you couldn't physically spin it to fast anyway.
    if ((millis() - encoderChange) > 10) {
      encoderState = (encoderState << 2) | state;
      encoderMoved = true;
      encoderChange = millis();
    } // if no bounce
  } // if state changed
} //updateEncoderState

/****************************************************************/
///
/// Read all plugs and load the plug array with the values
///
void readAll() {
  uint16_t val;
  val = i2c_read2(mcp_address, GPIOA);
  valA[0] = val & 0xFF;
  valA[1] = val >> 8;
  valA[0] |= 0xfc; // set the unused pins also, just so bit count is easier
  val = i2c_read2(mcp_address + 1, GPIOA);
  valA[2] = val & 0xFF;
  valA[3] = val >> 8;
  valA[2] |= 0xfc; // set the unused pins also, just so bit count is easier
}

/****************************************************************/
// Interrupt routine that will update the display
void updateDisplay() {
  static uint8_t currentLED = 0;
  static uint8_t seg; // "static" to not have to create it each interrupt

  updateEncoderState(); // check encoder

  digitalWrite(Common[currentLED], CommonInactive); // first deactivate current digit

  //move on to next digit
  if (currentLED == (MAXLED-1)) {
    currentLED = 0;
  } else {
    currentLED++;
  }

  for (seg = 0; seg < 8; seg++) { // turn on/off segments as needed
    if (LEDBuffer[currentLED] & 1 << seg) {
      digitalWrite(Segment[7 - seg], SegmentActive);
    } else {
      digitalWrite(Segment[7 - seg], SegmentInactive);
    }
  }
  digitalWrite(Common[currentLED], CommonActive); // Activate the digit

} // updateDisplay

/****************************************************************/
//
// what number (0-99) to show
//
void showNumber(uint8_t number, boolean leadingSpace = false, boolean dp = false) {

  LEDassembly[0] = (number % 100) / 10; if (dp) {
    LEDassembly[0] += 10;
  }
  LEDassembly[1] = number % 10;; if (dp) {
    LEDassembly[1] += 10;
  }
  if (leadingSpace) {
    // replace leading "0" with space
    if (LEDassembly[0] == 0 ) {
      LEDassembly[0] = 20;
    } else if (LEDassembly[0] == 10 ) {
      LEDassembly[0] = 21;
    }
  }
  //Have now figured out what we need to show, time to update display buffer
  noInterrupts();
  LEDBuffer[0] = LEDpattern[LEDassembly[0]];
  LEDBuffer[1] = LEDpattern[LEDassembly[1]];
  interrupts();
}

/****************************************************************/
/****************************************************************/
void setup() {
  boolean boardMissing;
  uint8_t i;

  Serial.begin(38400);
  Serial.println(F("UHR box v0.03"));
  Serial.println();

  Wire.begin(); // enable the wire lib
  //to fast  Wire.setClock(400000L);

  do {
    boardMissing = false;
    //Check for portexpander
    Wire.beginTransmission(mcp_address);
    if (Wire.endTransmission() != 0) {
      Serial.println(F("port expander not found - ABORT"));
      boardMissing = true;
      delay(500);
    }
  } while (boardMissing);

  Serial.println(F("Preparing plugboard"));

  // Setup the port multipler
  i2c_write2(mcp_address, IOCON, 0b00011110); // Init value for IOCON, bank(0)+INTmirror(no)+SQEOP(addr inc)+DISSLW(Slew rate disabled)+HAEN(hw addr always enabled)+ODR(INT open)+INTPOL(act-low)+0(N/A)
  i2c_write2(mcp_address, IODIRA, 0xff); // Set all ports to inputs
  i2c_write2(mcp_address, IODIRB, 0xff); // Set all ports to inputs
  i2c_write2(mcp_address, GPPUA, 0xff); // enable pullup
  i2c_write2(mcp_address, GPPUB, 0xff); //

  i2c_write2(mcp_address + 1, IOCON, 0b00011110); // Init value for IOCON, bank(0)+INTmirror(no)+SQEOP(addr inc)+DISSLW(Slew rate disabled)+HAEN(hw addr always enabled)+ODR(INT open)+INTPOL(act-low)+0(N/A)
  i2c_write2(mcp_address + 1, IODIRA, 0xff); // Set all ports to inputs
  i2c_write2(mcp_address + 1, IODIRB, 0xff); // Set all ports to inputs
  i2c_write2(mcp_address + 1, GPPUA, 0xff); // enable pullup
  i2c_write2(mcp_address + 1, GPPUB, 0xff); //

  // read current state of the pins
  readAll();

  ////////////////////////////////
  // Get ready to display a number
  for (i = 0; i < MAXLED; i++) {
    pinMode(Common[i], OUTPUT);
    digitalWrite(Common[i], CommonInactive);  // turn off all LEDs for a start
  }
  for (i = 0; i < 8; i++) {
    pinMode(Segment[i], OUTPUT);
    digitalWrite(Segment[i], SegmentInactive);
  }

  showNumber(uhrpos);

  // prep the encoder pins, reading them is done in the interrupt
  pinMode(encoderPins[0], INPUT_PULLUP);
  pinMode(encoderPins[1], INPUT_PULLUP);

  // Setup timer interrupt to update the LED
//  Timer1.initialize(1000); // Run every .001 seconds
  Timer1.initialize(7500); // Run every .0075 seconds
//  Timer1.initialize(10000); // Run every .01 seconds
  Timer1.attachInterrupt(updateDisplay);

  Serial.println(F("Setup done"));
} // setup

/****************************************************************/
void checkEncoder(){
  
  static unsigned long lastUHRChange = 0;
  static boolean uhrSaved=true;

    // first check if the selector was changed
  if (encoderMoved) {
    encoderMoved = false;
    //    printBin8(encoderState); Serial.println(); //DEBUG
    if ((encoderState & 0b11) == 0b11) { // current state is bottom of the click
      switch ((encoderState & 0b1100) >> 2) { //check prev state
        case B00: // Something wrong, a state was skipped, bad contact?
        case B11: // if current is 11 prev can't be 11 also
          break;
        case B10:
          if (uhrpos < 39)
            uhrpos++;
          else
            uhrpos = 0;
          break;
        case B01:
          if (uhrpos > 0)
            uhrpos--;
          else
            uhrpos = 39;
          break;
      } // switch
    } // if current state is bottom of the click
    showNumber(uhrpos);
    lastUHRChange=millis();
    uhrSaved=false;
  } // if encoderMoved

  if (!uhrSaved && lastUHRChange+2000 < millis()){ // Save uhrPos after 2 seconds idle
    //saveUHRpos(uhrpos);
    uhrSaved=true;
  }
      

} // checkEncoder

/****************************************************************/
/****************************************************************/
// return plugnumber based on bitmap
uint8_t getPlugInfo(uint8_t val, uint8_t base = 0) {
  uint8_t bits;

  bits = pgm_read_byte(&BITMASK[val]);
  if (bits != 0) {
    if (bits > 0xf) {
      return base + (bits >> 4); // Return just the top number, ignore the bottom part
    } else if ((bits & 0xf) > 0) { // this part is no longer used
      return base + (bits & 0xf);
    }
  }
  return 0;
} // getPlugInfo

//#define SHOWBITS
#define DEBUG
//#define DEBUG2


void loop() {
  uint8_t val1, val2;
  static uint8_t i, j, plugIn, plugOut,plug,contact;
  uint8_t bitcnt;
  static uint8_t loopcnt = 0, activePlug = 99, activeContact=99;
  static uint8_t activePlugOut = 99;
  static uint8_t activePortOut = 99;
  static unsigned long lastChange = 0;

  readAll();
  bitcnt = (__builtin_popcount(valA[0]) +
            __builtin_popcount(valA[1]) +
            __builtin_popcount(valA[2]) +
            __builtin_popcount(valA[3])
           );

#ifdef SHOWBITS
  if (bitcnt != 32 ) { // some input is low
    printBin8(valA[0]);
    printBin8(valA[1]);
    printBin8(valA[2]);
    printBin8(valA[3]);
    Serial.println();
    delay(200);
  }
  return; // skip rest of the code
#endif
  
  if (bitcnt == 31 ) { // something changed, one plug is low
    lastChange = millis();

    plug = getPlugInfo(valA[0]) + getPlugInfo(valA[1], 8); // read red/outer ring
    if (plug==0){
      plug = getPlugInfo(valA[2]) + getPlugInfo(valA[3], 8); // read white/inner ring
      if (plug != 0){
        plug+=10; // white/inner ring is plug 10-19
      }
    }
    // at any given time it is only _one_ incoming plug that is valid (red _or_ white)
    // anything else is discarded
    // also, it will _always_ go from outer/red to inner/white or inner/white to outer/red, it will never go inner to inner or outer to outer

    if (plug > 0 && plug != activePlug) { 
      if (plug < 10) { // the active signal is on the red/outer side
        //        contact = pgm_read_byte(&UHRIN[plug - 1]); // get the contact it is connected to
        contact = (plug - 1) * 4; // get the contact it is connected to
        activeContact = pgm_read_byte(&UHROUTER[(contact + uhrpos) % 40]); // find plug to trigger on the white side
        activePlug    = UHROUT_REV[activeContact-uhrpos]+11; // activePlug is now 11-20
        activePortOut = activePlug+5; // "+6" because plug 1-10 on first chip(port 0-9), 11-20 on second chip (port 16-31)
      }else{ // the active signal is on the white/inner side
        contact = pgm_read_byte(&UHROUT[plug - 11]); // get the contact it is connected to
        activeContact = pgm_read_byte(&UHRINNER[(contact + uhrpos) % 40]); // find plug to trigger on the red side
        //        activePlug=UHRIN_REV[activeContact] + 1;
        activePlug=(activeContact-uhrpos-2)/4+1;  // activePlug is now 1-10
        activePortOut=activePlug-1;
      }
    } else {
      activePortOut=99;
    }

    if (activePortOut < 20) {
      setPortOut(activePortOut);
    } else {
      setPortInAll(); // go back to listening
      activePlug = 99;
      activePortOut=99;
    }
    
#ifdef DEBUG2
    Serial.print(F("ActiveContact: "));
    Serial.println(activePlug);
    activePlug=99;
    delay(500);
    return;
#endif

#ifdef DEBUG
    uint8_t valA_prev[4], bitcnt_prev;
    for (i = 0; i < 4; i++) {
      valA_prev[i] = valA[i];
    }
    bitcnt_prev = bitcnt;
    readAll();
    bitcnt = (__builtin_popcount(valA[0]) +
              __builtin_popcount(valA[1]) +
              __builtin_popcount(valA[2]) +
              __builtin_popcount(valA[3])
             );

    // first show tested values
    Serial.print(bitcnt_prev);
    Serial.print(F(" "));
    for (i = 0; i < 4; i++) {
      Serial.print(getPlugInfo(valA_prev[i]));
      Serial.print(F(" ("));
      printBin8(valA_prev[i]);
      Serial.print(F(") "));
    }
    Serial.print(F("  "));

    Serial.print(plugIn);
    Serial.print(F(" & "));
    Serial.print(plugOut);
    Serial.print(F(" = "));
    Serial.print(activePlug);
    //    Serial.print(F(" -> "));
    //    Serial.print(activePlugOut);
    Serial.print(F(" - "));

    uint8_t plugVal;
    char dir;
    if (plugIn > 0) {
      plugVal = plugIn;
      dir = 'W';
    } else {
      plugVal = plugOut;
      dir = 'R';
    }
    Serial.print(F(" plugVal-1+uhrpos: "));
    Serial.print((uint8_t) (plugVal - 1 + uhrpos));
    Serial.print(F(", (plugVal-1+uhrpos) % 40: "));
    Serial.print((uint8_t) ((plugVal - 1 + uhrpos) % 40));
    if (plugIn > 0) {
      Serial.print(F(", &UHROUTER[(plugVal-1+uhrpos) % 40]: "));
      Serial.print((uint8_t) &UHROUTER[(plugVal - 1 + uhrpos) % 40]);
      Serial.print(F(", pgm_read_byte(..): "));
      Serial.print((uint8_t) pgm_read_byte(&UHROUTER[(plugVal - 1 + uhrpos) % 40]));
    } else {
      Serial.print(F(", &UHRINNER[(plugVal-1+uhrpos) % 40]: "));
      Serial.print((uint8_t) &UHRINNER[(plugVal - 1 + uhrpos) % 40]);
      Serial.print(F(", pgm_read_byte(..): "));
      Serial.print((uint8_t) pgm_read_byte(&UHRINNER[(plugVal - 1 + uhrpos) % 40]));
    }

    Serial.print((uint8_t) &UHROUTER[(plugVal - 1 + uhrpos) % 40]);
    Serial.print(F(", pgm_read_byte(..): "));
    Serial.print((uint8_t) pgm_read_byte(&UHROUTER[(plugVal - 1 + uhrpos) % 40]));
    Serial.print(F(" "));

    if (plugIn > 0 && plugOut > 0) {
      Serial.print(F(" <<<<<< "));
    }
    Serial.println();

    // now show current values
    Serial.print(bitcnt);
    Serial.print(F(" "));
    for (i = 0; i < 4; i++) {
      Serial.print(getPlugInfo(valA[i]));
      Serial.print(F(" ("));
      printBin8(valA[i]);
      Serial.print(F(") "));
      //        printHex4(valA[j]);Serial.print(F(" "));
    }
    Serial.println();
    Serial.println();
    delay(305 + random(330));
#endif

    /*
      for (i=0;i<4;i++){
        val1=pgm_read_word(&BITMASK[valA[i]]);
        if ((val1 >> 4)>0){
          plug[i*8+(val1 >> 4)]=true;
        }
      if ((val1 & 0xf)>0){
        plug[i*8+(val1 & 0xf)]=true;
      }

      if (val1 != 0){
        Serial.print(i);
        Serial.print(F(": "));
        Serial.print(val1);
        Serial.print(F(": "));
        printBin8(valA[i]);
        Serial.print(F(" "));
        Serial.print(val1 >> 4);
        Serial.print(F(" "));
        Serial.print(val1 & 0xF);
        Serial.println();
      }
      }
    */
  } else {
    if ((unsigned long)(millis() / 1000 - lastChange / 1000) > 300) { // after this many seconds, go to standby
      //      Serial.println(F("no activity, going to standby mode"));
      Serial.print(F("no activity for "));
      Serial.print(millis() / 1000 - lastChange / 1000);
      Serial.println(F(" seconds, going to standby mode"));
      //To be written, code to go to very deep sleep
      lastChange = millis();
    }
  } // if bitcnt==31

  /*
    if (bitcnt == 31 || bitcnt == 30) { // 31 for first round, 30 for following rounds
    if ((valA[0] & 4) == 0 && activePlug != 0){ // 4 = "E", 0="Q" on the "uhr" side
      setPortOut(0);
      activePlug=2;
    }else if ((valA[0] & 1) == 0 && activePlug != 2){ // 4 = "E", 0="Q" on the "uhr" side
      setPortOut(2);
      activePlug=0;
    } else {
      activePlug=99;
      setPortIn(0);
    }
    } else {
    setPortIn(0);
    }
    for (i=0;i<20;i++){plug[i]=false;} //clear all plugs for next round

    //(char)pgm_read_byte(&steckerbrett[0]+key-1)
    for (i = 0; i < 26; i++) {
    if (plug[i]) {
      Serial.print(" ");
      Serial.print((char)toupper(pgm_read_byte(&steckerbrett[0] + i)));
      Serial.print("  ");
    } else {
      Serial.print(">");
      Serial.print((char)tolower(pgm_read_byte(&steckerbrett[0] + i)));
      Serial.print("< ");
    }
    if (i == 8) {
      Serial.println(); Serial.print(" ");
    } else if (i == 16) {
      Serial.println();
    }
    }
    Serial.println();

    if (loopcnt % 2 == 0) {
    digitalWrite(LED_BUILTIN, LOW);
    } else {
    digitalWrite(LED_BUILTIN, HIGH);
    }
    loopcnt++;
    if (loopcnt == 10) {
    loopcnt = 0;
    }
    display.display();
    delay(10);
  */
} // loop
