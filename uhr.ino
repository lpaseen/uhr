/****************************************************************
    uhr box
    Electronic simulation of the uhr box that luftwaffe used
    v0.00 - a first version to test the basic concept
    v0.02 - added code for a dual 7seg display
    v0.03 - added code for encoder
    
*/


#include <SPI.h>
#include <Wire.h>


/****************************************************************/
// from https://github.com/PaulStoffregen/TimerOne
// also at https://www.pjrc.com/teensy/td_libs_TimerOne.html
#include <TimerOne.h>

uint8_t LEDpattern[]={
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


#define CC
#define MAXLED 2
static uint8_t LEDBuffer[MAXLED]={20,20};
static uint8_t LEDassembly[MAXLED]={20,20};

#ifdef CC
//common cathode
static uint8_t CommonActive=LOW;
static uint8_t CommonInactive=HIGH;
static uint8_t SegmentActive=HIGH;
static uint8_t SegmentInactive=LOW;
#elif CA
//common anode
static uint8_t CommonActive=HIGH;
static uint8_t CommonInactive=LOW;
static uint8_t SegmentActive=LOW;
static uint8_t SegmentInactive=HIGH;
#else
#error need CC or CA defined
#endif

//How the led display is connected
//                    digit0, digit1
const uint8_t Common[] = {A2, A3};
//                           dp, a   b   c   d   e   f   g
const uint8_t Segment[] = {  10, 6,  7,  9, 12, 11,  8, 13};

/****************************************************************/
//static const uint8_t encoderPins[] = {2,3}; // external interrupt is only available on 2 and 3
static const uint8_t encoderPins[] = {4,5}; // 
volatile uint8_t encoderState = 0xff;
volatile unsigned long encoderChange=0;
volatile boolean encoderMoved=false;

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

// Uhrkontakt (w) 00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 
// Uhrkontakt (r) 26 11 24 21 02 31 00 25 30 39 28 13 22 35 20 37 06 23 04 33 34 19 32 09 18 07 16 17 10 03 08 01 38 27 36 29 14 15 12 05

// Red plugs (a) goes to outer ring
// white plugs (b) goes to inner ring

//                               00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39
const uint8_t UHROUTER[] PROGMEM  = { 6,31, 4,29,18,39,16,25,30,23,28, 1,38,11,36,37,26,27,24,21,14, 3,12,17, 2, 7, 0,33,10,35, 8, 5,22,19,20,13,34,15,32, 9};
const uint8_t UHRINNER[] PROGMEM  = {26,11,24,21, 2,31, 0,25,30,39,28,13,22,35,20,37, 6,23, 4,33,34,19,32, 9,18, 7,16,17,10, 3, 8, 1,38,27,36,29,14,15,12, 5};
// red plugs (UHRIN) are in order 0..9
//                               0 1 2 3 4 5 6 7 8 9
const uint8_t UHROUT[] PROGMEM ={6,0,7,5,1,8,4,2,9,3}; // the white plugs
static uint8_t uhrpos=0;

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
const uint8_t BITMASK[] PROGMEM  = {
  0, // #   0=00000000,bits=8
  0, // #   1=00000001,bits=7
  0, // #   2=00000010,bits=7
  0, // #   3=00000011,bits=6
  0, // #   4=00000100,bits=7
  0, // #   5=00000101,bits=6
  0, // #   6=00000110,bits=6
  0, // #   7=00000111,bits=5
  0, // #   8=00001000,bits=7
  0, // #   9=00001001,bits=6
  0, // #  10=00001010,bits=6
  0, // #  11=00001011,bits=5
  0, // #  12=00001100,bits=6
  0, // #  13=00001101,bits=5
  0, // #  14=00001110,bits=5
  0, // #  15=00001111,bits=4
  0, // #  16=00010000,bits=7
  0, // #  17=00010001,bits=6
  0, // #  18=00010010,bits=6
  0, // #  19=00010011,bits=5
  0, // #  20=00010100,bits=6
  0, // #  21=00010101,bits=5
  0, // #  22=00010110,bits=5
  0, // #  23=00010111,bits=4
  0, // #  24=00011000,bits=6
  0, // #  25=00011001,bits=5
  0, // #  26=00011010,bits=5
  0, // #  27=00011011,bits=4
  0, // #  28=00011100,bits=5
  0, // #  29=00011101,bits=4
  0, // #  30=00011110,bits=4
  0, // #  31=00011111,bits=3
  0, // #  32=00100000,bits=7
  0, // #  33=00100001,bits=6
  0, // #  34=00100010,bits=6
  0, // #  35=00100011,bits=5
  0, // #  36=00100100,bits=6
  0, // #  37=00100101,bits=5
  0, // #  38=00100110,bits=5
  0, // #  39=00100111,bits=4
  0, // #  40=00101000,bits=6
  0, // #  41=00101001,bits=5
  0, // #  42=00101010,bits=5
  0, // #  43=00101011,bits=4
  0, // #  44=00101100,bits=5
  0, // #  45=00101101,bits=4
  0, // #  46=00101110,bits=4
  0, // #  47=00101111,bits=3
  0, // #  48=00110000,bits=6
  0, // #  49=00110001,bits=5
  0, // #  50=00110010,bits=5
  0, // #  51=00110011,bits=4
  0, // #  52=00110100,bits=5
  0, // #  53=00110101,bits=4
  0, // #  54=00110110,bits=4
  0, // #  55=00110111,bits=3
  0, // #  56=00111000,bits=5
  0, // #  57=00111001,bits=4
  0, // #  58=00111010,bits=4
  0, // #  59=00111011,bits=3
  0, // #  60=00111100,bits=4
  0, // #  61=00111101,bits=3
  0, // #  62=00111110,bits=3
  0x87, // #  63=00111111,bits=2
  0, // #  64=01000000,bits=7
  0, // #  65=01000001,bits=6
  0, // #  66=01000010,bits=6
  0, // #  67=01000011,bits=5
  0, // #  68=01000100,bits=6
  0, // #  69=01000101,bits=5
  0, // #  70=01000110,bits=5
  0, // #  71=01000111,bits=4
  0, // #  72=01001000,bits=6
  0, // #  73=01001001,bits=5
  0, // #  74=01001010,bits=5
  0, // #  75=01001011,bits=4
  0, // #  76=01001100,bits=5
  0, // #  77=01001101,bits=4
  0, // #  78=01001110,bits=4
  0, // #  79=01001111,bits=3
  0, // #  80=01010000,bits=6
  0, // #  81=01010001,bits=5
  0, // #  82=01010010,bits=5
  0, // #  83=01010011,bits=4
  0, // #  84=01010100,bits=5
  0, // #  85=01010101,bits=4
  0, // #  86=01010110,bits=4
  0, // #  87=01010111,bits=3
  0, // #  88=01011000,bits=5
  0, // #  89=01011001,bits=4
  0, // #  90=01011010,bits=4
  0, // #  91=01011011,bits=3
  0, // #  92=01011100,bits=4
  0, // #  93=01011101,bits=3
  0, // #  94=01011110,bits=3
  0x86, // #  95=01011111,bits=2
  0, // #  96=01100000,bits=6
  0, // #  97=01100001,bits=5
  0, // #  98=01100010,bits=5
  0, // #  99=01100011,bits=4
  0, // # 100=01100100,bits=5
  0, // # 101=01100101,bits=4
  0, // # 102=01100110,bits=4
  0, // # 103=01100111,bits=3
  0, // # 104=01101000,bits=5
  0, // # 105=01101001,bits=4
  0, // # 106=01101010,bits=4
  0, // # 107=01101011,bits=3
  0, // # 108=01101100,bits=4
  0, // # 109=01101101,bits=3
  0, // # 110=01101110,bits=3
  0x85, // # 111=01101111,bits=2
  0, // # 112=01110000,bits=5
  0, // # 113=01110001,bits=4
  0, // # 114=01110010,bits=4
  0, // # 115=01110011,bits=3
  0, // # 116=01110100,bits=4
  0, // # 117=01110101,bits=3
  0, // # 118=01110110,bits=3
  0x84, // # 119=01110111,bits=2
  0, // # 120=01111000,bits=4
  0, // # 121=01111001,bits=3
  0, // # 122=01111010,bits=3
  0x83, // # 123=01111011,bits=2
  0, // # 124=01111100,bits=3
  0x82, // # 125=01111101,bits=2
  0x81, // # 126=01111110,bits=2
  0x80, // # 127=01111111,bits=1
  0, // # 128=10000000,bits=7
  0, // # 129=10000001,bits=6
  0, // # 130=10000010,bits=6
  0, // # 131=10000011,bits=5
  0, // # 132=10000100,bits=6
  0, // # 133=10000101,bits=5
  0, // # 134=10000110,bits=5
  0, // # 135=10000111,bits=4
  0, // # 136=10001000,bits=6
  0, // # 137=10001001,bits=5
  0, // # 138=10001010,bits=5
  0, // # 139=10001011,bits=4
  0, // # 140=10001100,bits=5
  0, // # 141=10001101,bits=4
  0, // # 142=10001110,bits=4
  0, // # 143=10001111,bits=3
  0, // # 144=10010000,bits=6
  0, // # 145=10010001,bits=5
  0, // # 146=10010010,bits=5
  0, // # 147=10010011,bits=4
  0, // # 148=10010100,bits=5
  0, // # 149=10010101,bits=4
  0, // # 150=10010110,bits=4
  0, // # 151=10010111,bits=3
  0, // # 152=10011000,bits=5
  0, // # 153=10011001,bits=4
  0, // # 154=10011010,bits=4
  0, // # 155=10011011,bits=3
  0, // # 156=10011100,bits=4
  0, // # 157=10011101,bits=3
  0, // # 158=10011110,bits=3
  0x76, // # 159=10011111,bits=2
  0, // # 160=10100000,bits=6
  0, // # 161=10100001,bits=5
  0, // # 162=10100010,bits=5
  0, // # 163=10100011,bits=4
  0, // # 164=10100100,bits=5
  0, // # 165=10100101,bits=4
  0, // # 166=10100110,bits=4
  0, // # 167=10100111,bits=3
  0, // # 168=10101000,bits=5
  0, // # 169=10101001,bits=4
  0, // # 170=10101010,bits=4
  0, // # 171=10101011,bits=3
  0, // # 172=10101100,bits=4
  0, // # 173=10101101,bits=3
  0, // # 174=10101110,bits=3
  0x75, // # 175=10101111,bits=2
  0, // # 176=10110000,bits=5
  0, // # 177=10110001,bits=4
  0, // # 178=10110010,bits=4
  0, // # 179=10110011,bits=3
  0, // # 180=10110100,bits=4
  0, // # 181=10110101,bits=3
  0, // # 182=10110110,bits=3
  0x74, // # 183=10110111,bits=2
  0, // # 184=10111000,bits=4
  0, // # 185=10111001,bits=3
  0, // # 186=10111010,bits=3
  0x73, // # 187=10111011,bits=2
  0, // # 188=10111100,bits=3
  0x72, // # 189=10111101,bits=2
  0x71, // # 190=10111110,bits=2
  0x70, // # 191=10111111,bits=1
  0, // # 192=11000000,bits=6
  0, // # 193=11000001,bits=5
  0, // # 194=11000010,bits=5
  0, // # 195=11000011,bits=4
  0, // # 196=11000100,bits=5
  0, // # 197=11000101,bits=4
  0, // # 198=11000110,bits=4
  0, // # 199=11000111,bits=3
  0, // # 200=11001000,bits=5
  0, // # 201=11001001,bits=4
  0, // # 202=11001010,bits=4
  0, // # 203=11001011,bits=3
  0, // # 204=11001100,bits=4
  0, // # 205=11001101,bits=3
  0, // # 206=11001110,bits=3
  0x65, // # 207=11001111,bits=2
  0, // # 208=11010000,bits=5
  0, // # 209=11010001,bits=4
  0, // # 210=11010010,bits=4
  0, // # 211=11010011,bits=3
  0, // # 212=11010100,bits=4
  0, // # 213=11010101,bits=3
  0, // # 214=11010110,bits=3
  0x64, // # 215=11010111,bits=2
  0, // # 216=11011000,bits=4
  0, // # 217=11011001,bits=3
  0, // # 218=11011010,bits=3
  0x63, // # 219=11011011,bits=2
  0, // # 220=11011100,bits=3
  0x62, // # 221=11011101,bits=2
  0x61, // # 222=11011110,bits=2
  0x60, // # 223=11011111,bits=1
  0, // # 224=11100000,bits=5
  0, // # 225=11100001,bits=4
  0, // # 226=11100010,bits=4
  0, // # 227=11100011,bits=3
  0, // # 228=11100100,bits=4
  0, // # 229=11100101,bits=3
  0, // # 230=11100110,bits=3
  0x54, // # 231=11100111,bits=2
  0, // # 232=11101000,bits=4
  0, // # 233=11101001,bits=3
  0, // # 234=11101010,bits=3
  0x53, // # 235=11101011,bits=2
  0, // # 236=11101100,bits=3
  0x52, // # 237=11101101,bits=2
  0x51, // # 238=11101110,bits=2
  0x50, // # 239=11101111,bits=1
  0, // # 240=11110000,bits=4
  0, // # 241=11110001,bits=3
  0, // # 242=11110010,bits=3
  0x43, // # 243=11110011,bits=2
  0, // # 244=11110100,bits=3
  0x42, // # 245=11110101,bits=2
  0x41, // # 246=11110110,bits=2
  0x40, // # 247=11110111,bits=1
  0, // # 248=11111000,bits=3
  0x32, // # 249=11111001,bits=2
  0x31, // # 250=11111010,bits=2
  0x30, // # 251=11111011,bits=1
  0x21, // # 252=11111100,bits=2
  0x20, // # 253=11111101,bits=1
  0x10, // # 254=11111110,bits=1
  0  // # 255=11111111,bits=0
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
//Set a pin output and low
///
void setPortOut(uint8_t plug) {
  //make port "plug" output (and every other an input)
  i2c_write2(pbLookup[plug][0],IODIRA+pbLookup[plug][1],0xff ^ (1<<pbLookup[plug][2]));
  //set  port "plug" low
  i2c_write2(pbLookup[plug][0],GPIOA+pbLookup[plug][1],0xff ^ (1<<pbLookup[plug][2]));
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
    if ((millis()-encoderChange) > 10){ 
      encoderState = (encoderState << 2) | state;
      encoderMoved = true;
      encoderChange=millis();
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
  valA[0]=val & 0xFF;
  valA[1]=val >> 8;
  valA[1] |= 0xfc; // set the unused pins also, just so bit count is easier
  val = i2c_read2(mcp_address + 1, GPIOA);
  valA[2]=val & 0xFF;
  valA[3]=val >> 8;
  valA[3] |= 0xfc; // set the unused pins also, just so bit count is easier
}

/****************************************************************/
// Interrupt routine that will update the display
void updateDisplay(){
  static uint8_t currentLED=0;
  static uint8_t seg; // "static" to not have to create it each interrupt
  
  updateEncoderState(); // check encoder
  
  digitalWrite(Common[currentLED],CommonInactive); // first deactivate current digit

  //move on to next digit
  currentLED++;
  if (currentLED==MAXLED){
    currentLED=0;
  }

  for (seg=0;seg<8;seg++){ // turn on/off segments as needed
    if (LEDBuffer[currentLED] & 1<<seg){
      digitalWrite(Segment[7-seg],SegmentActive);
    }else{
      digitalWrite(Segment[7-seg],SegmentInactive);
    }
  }
  digitalWrite(Common[currentLED],CommonActive); // Activate the digit

} // updateDisplay

/****************************************************************/
//
// what number (0-99) to show
//
void showNumber(uint8_t number, boolean dp=false){
  
  LEDassembly[0]=(number%100)/10;if (dp) {LEDassembly[0]+=10;}
  LEDassembly[1]=number%10;;if (dp) {LEDassembly[1]+=10;}
  // replace leading "0" with space
  if (LEDassembly[0]==0 ){
    LEDassembly[0]=20;
  } else if (LEDassembly[0]==10 ){
    LEDassembly[0]=21;
  }
  //Have now figured out what we need to show, time to update display buffer
  noInterrupts();
  LEDBuffer[0]=LEDpattern[LEDassembly[0]];
  LEDBuffer[1]=LEDpattern[LEDassembly[1]];
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
  delay(100);

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

//  pinMode(2, INPUT);
//  pinMode(3, INPUT);
//  pinMode(4, INPUT);
//  pinMode(5, INPUT);

  pinMode(encoderPins[0], INPUT_PULLUP);
  pinMode(encoderPins[1], INPUT_PULLUP);

  // Setup timer interrupt to update the LED
  Timer1.initialize(1000); // Run every .001 seconds
  Timer1.attachInterrupt(updateDisplay);

  //  attachInterrupt(digitalPinToInterrupt(encoderPins[0]), updateEncoderState, CHANGE);
  //  attachInterrupt(digitalPinToInterrupt(encoderPins[1]), updateEncoderState, CHANGE);
} // setup


/****************************************************************/
/****************************************************************/

uint8_t getPlugInfo(uint8_t val,uint8_t base=0){
  uint8_t bits;
  
  bits=pgm_read_byte(&BITMASK[val]);
  if (bits!=0){
    if (bits > 0xf){
        return base+(bits >> 4);
    }else if ((bits & 0xf)>0){
       return base+(bits & 0xf);
    }
  }
  return 0;
} // getPlugInfo

//#define SHOWBITS
#define DEBUG


void loop() {
  uint8_t val1, val2;
  static uint8_t i, j,plugIn,plugOut;
  uint8_t bitcnt;
  static boolean plug[20];
  static uint8_t loopcnt = 0, activePlug=99;
  static uint8_t activePlugOut = 99;
  static unsigned long lastChange=0;

  if (encoderMoved){
    encoderMoved=false;
    //    Serial.println(encoderState & 0b1111,HEX);
    //    printBin8(encoderState); Serial.println(); //PSDEBUG
    if ((encoderState & 0b11)==0b11) { // current state is bottom of the click
      switch ((encoderState & 0b1100)>>2) { //check prev state
        case B00:
        case B11: // if current is 11 prev can't be 11 also
          break;
        case B10:
          if (uhrpos<39)
            uhrpos++;
          else
            uhrpos=0;
          break;
        case B01:
          if (uhrpos>0)
            uhrpos--;
          else
            uhrpos=39;
          break;
  	} // switch
      } // if current state is bottom of the click 
  } // if encoderMoved
  
  readAll();
  bitcnt = (__builtin_popcount(valA[0]) +
            __builtin_popcount(valA[1]) +
            __builtin_popcount(valA[2]) +
            __builtin_popcount(valA[3])
            );

  //  showNumber(millis()/1000);
  showNumber(uhrpos);

  if (bitcnt == 31 ){ // something changed, one plug is low
    lastChange=millis();

    plugIn =getPlugInfo(valA[0])+getPlugInfo(valA[1],8); // read red/outer ring
    plugOut=getPlugInfo(valA[2])+getPlugInfo(valA[3],8); // read white/inner ring
    // at any given time it is only _one_ incoming plug that is valid
    // anything else is discarded

    if (plugOut>0){ // the active signale is on the white/inner side, transpose it
          plugOut=pgm_read_byte(&UHROUT[plugOut-1])+1;
    }

    if (plugIn>0 && plugIn != activePlug){ // detecting a scan on the red side
      activePlug=pgm_read_byte(&UHROUTER[((plugIn-1)*2+uhrpos) % 40]); // find plug to trigger on the white side
      // BUG, probably need to do a reverse of UHROUT here
    }else if (plugOut>0 && plugOut != activePlug){ // detecting a scan on the white side
      activePlug=pgm_read_byte(&UHRINNER[((plugIn-1)*2+uhrpos) % 40]); // find plug to trigger on the red side
    }else{
      activePlug=99; // might be the plug we activated in prev scan
    }

    if (activePlug < 40){
      if(activePlug > 9){ // plug 0-9 on first chip, 10-19 on second chip
        setPortOut(activePlug+6); 
      }else{
        setPortOut(activePlug);
      }
    }else{
      setPortInAll(); // go back to listening
    }
    
#ifdef DEBUG
    uint8_t valA_prev[4],bitcnt_prev;
    for (i = 0; i < 4; i++) {
      valA_prev[i]=valA[i];
    }
    bitcnt_prev=bitcnt;
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
    if (plugIn>0){
      plugVal=plugIn;
      dir='W';
    }else{
      plugVal=plugOut;
      dir='R';
    }
    Serial.print(F(" plugVal-1+uhrpos: "));
    Serial.print((uint8_t) (plugVal-1+uhrpos));
    Serial.print(F(", (plugVal-1+uhrpos) % 40: "));
    Serial.print((uint8_t) ((plugVal-1+uhrpos) % 40));
    if (plugIn>0){
      Serial.print(F(", &UHROUTER[(plugVal-1+uhrpos) % 40]: "));
      Serial.print((uint8_t) &UHROUTER[(plugVal-1+uhrpos) % 40]);
      Serial.print(F(", pgm_read_byte(..): "));
      Serial.print((uint8_t) pgm_read_byte(&UHROUTER[(plugVal-1+uhrpos) % 40]));
    }else{         
      Serial.print(F(", &UHRINNER[(plugVal-1+uhrpos) % 40]: "));
      Serial.print((uint8_t) &UHRINNER[(plugVal-1+uhrpos) % 40]);
      Serial.print(F(", pgm_read_byte(..): "));
      Serial.print((uint8_t) pgm_read_byte(&UHRINNER[(plugVal-1+uhrpos) % 40]));
    }
                 
    Serial.print((uint8_t) &UHROUTER[(plugVal-1+uhrpos) % 40]);
    Serial.print(F(", pgm_read_byte(..): "));
    Serial.print((uint8_t) pgm_read_byte(&UHROUTER[(plugVal-1+uhrpos) % 40]));
    Serial.print(F(" "));

    if (plugIn>0 && plugOut>0){Serial.print(F(" <<<<<< "));}
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
    delay(305+random(330));
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
  }else{
    if ((unsigned long)(millis()/1000-lastChange/1000)>300){ // after this many seconds, go to standby
      //      Serial.println(F("no activity, going to standby mode"));
      Serial.print(F("no activity for "));
      Serial.print(millis()/1000-lastChange/1000);
      Serial.println(F(" seconds, going to standby mode"));
      //To be written, code to go to very deep sleep
      lastChange=millis();
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
