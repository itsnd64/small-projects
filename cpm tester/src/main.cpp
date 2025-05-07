#include <Arduino.h>

#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <stdlib.h>
#include <string.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdint.h>
#include <avr/power.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

#define LCD_CHAR_DIODE1  1      // Diode-Icon; will be generated as custom character
#define LCD_CHAR_DIODE2  2      // Diode-Icon; will be generated as custom character

// REF_C_KORR corrects the reference Voltage for capacity measurement (<40uF) and has mV units.
// Greater values gives lower capacity results.
#define REF_C_KORR 12

// The CAP_EMPTY_LEVEL  defines the empty voltage level for capacitors in mV.
// Choose a higher value, if your Tester reports "Cell!" by unloading capacitors.
#define CAP_EMPTY_LEVEL 4

// The AUTOSCALE_ADC option enables the autoscale ADC (ADC use VCC and Bandgap Ref).
#define AUTOSCALE_ADC
#define REF_R_KORR 3

// The ESR_ZERO value define the zero value of ESR measurement (units = 0.01 Ohm).
//#define ESR_ZERO 29
#define ESR_ZERO 20

// Setting EBC_STYPE will select the old style to present the order of Transistor connection (EBC=...).
// Omitting the option will select the 123=... style.  Every point is replaced by a character identifying 
// type of connected transistor pin (B=Base, E=Emitter, C=Collector, G=Gate, S=Source, D=Drain).
// If you select EBC_STYLE=321 , the style will be 321=... , the inverted order to the 123=... style.
//#define EBC_STYLE
//#define EBC_STYLE 321

// The ANZ_MESS option specifies, how often an ADC value is read and accumulated.
// Possible values of ANZ_MESS are 5 to 200.
#define ANZ_MESS 25

// The sleep mode of the ATmega168 or ATmega328 is normally used by the software to save current.
// You can inhibit this with the option INHIBIT_SLEEP_MODE .
//#define INHIBIT_SLEEP_MODE

// ******** end of selectable options

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

// ########  Configuration

#ifndef ADC_PORT
#define ADC_PORT PORTC
#define ADC_DDR DDRC
#define ADC_PIN PINC
#define TP1 0
#define TP2 1
#define TP3 2
#define TPext 3
#define TPREF 4
#define TPBAT 5


#define TxD 3	  // TxD-Pin of Software-UART; must be at Port C !
#ifdef WITH_UART
  #define TXD_MSK (1<<TxD)
#else
  #define TXD_MSK 0xF8
#endif
  #define TXD_VAL TXD_MSK

/*
  exact values of used resistors (Ohm).
  The standard value for R_L is 680 Ohm, for R_H 470kOhm.

  To calibrate your tester the resistor-values can be adjusted:
*/
#define R_L_VAL 4700          // standard value 680 Ohm, multiplied by 10 for 0.1 Ohm resolution
#define R_H_VAL 10000         // standard value 470000 Ohm, multiplied by 10, divided by 100 

#define R_DDR DDRB
#define R_PORT PORTB

/*
  Port for the Test resistors
  The Resistors must be connected to the lower 6 Pins of the Port in following sequence:
  RLx = 680R-resistor for Test-Pin x
  RHx = 470k-resistor for Test-Pin x

  RL1 an Pin 0
  RH1 an Pin 1
  RL2 an Pin 2
  RH2 an Pin 3
  RL3 an Pin 4
  RH3 an Pin 5
*/

#define ON_DDR DDRD
#define ON_PORT PORTD
#define ON_PIN_REG PIND
#define ON_PIN 18               // Pin, must be switched to high to switch power on
#define RST_PIN 17            // Pin, is switched to low, if push button is pressed

#define U_VCC 5000// U_VCC defines the VCC Voltage of the ATmega in mV units
#define U_SCALE 4// U_SCALE can be set to 4 for better resolution of ReadADC function for resistor measurement
#define R_ANZ_MESS 190 // R_ANZ_MESS can be set to a higher number of measurements (up to 200) for resistor measurement
// ########  End of configuration 


// the following definitions specify where to load external data from: EEprom or flash

#define MEM_TEXT PROGMEM
#define MEM2_TEXT PROGMEM
#define MEM_read_word(a)  pgm_read_word(a)
#define MEM_read_byte(a)  pgm_read_byte(a)
#define MEM2_read_byte(a)  pgm_read_byte(a)
#define MEM2_read_word(a)  pgm_read_word(a)
#define lcd_fix2_string(a)  lcd_pgm_string(a)
#define use_lcd_pgm


// RH_OFFSET : systematic offset of resistor measurement with RH (470k) 
// resolution is 0.1 Ohm, 3500 defines a offset of 350 Ohm
#define RH_OFFSET 350 

// TP2_CAP_OFFSET is a additionally offset for TP2 capacity measurements in pF units
#define TP2_CAP_OFFSET 2

// CABLE_CAP defines the capacity (pF) of 12cm cable with clip at the terminal pins
#define CABLE_CAP 3

// automatic selection of right call type
#define ACALL rcall

#define MCU_STATUS_REG MCUCR
#define ADC_COMP_CONTROL ADCSRB
#define TI1_INT_FLAGS TIFR1
#define DEFAULT_BAND_GAP 1070
#define DEFAULT_RH_FAKT  884      // mega328 1070 mV
#define LONG_HFE // LONG_HFE  activates computation of current amplification factor with long variables
#define COMMON_COLLECTOR // COMMON_COLLECTOR activates measurement of current amplification factor in common collector circuit  (Emitter follower)
#define PIN_RM 200
#define PIN_RP 220
#define CC0 36 // CC0 defines the capacity of empty terminal pins 1 & 3 without cable
#define COMP_SLEW1 4000 // Slew rate correction  val += COMP_SLEW1 / (val + COMP_SLEW2)
#define COMP_SLEW2 180
#define C_NULL CC0+CABLE_CAP+(COMP_SLEW1 / (CC0 + CABLE_CAP + COMP_SLEW2))
#define MUX_INT_REF 0x0e	// channel number of internal 1.1 V
#define ADC_internal_reference (ref_mv + REF_R_KORR)
#define LONG_WAIT_TIME 28000
#define SHORT_WAIT_TIME 5000
#define OFF_WAIT_TIME  LONG_WAIT_TIME// if POWER OFF function is not selected, wait 14s before repeat measurement*
#define AUTO_CLOCK_DIV (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0)
#define FAST_CLOCK_DIV (1<<ADPS2) | (1<<ADPS0)

#ifdef INHIBIT_SLEEP_MODE
  // save memory, do not use the sleep mode
  #define wait_about5ms() wait5ms()
  #define wait_about10ms() wait10ms()
  #define wait_about20ms() wait20ms()
  #define wait_about30ms() wait30ms()
  #define wait_about50ms() wait50ms()
  #define wait_about100ms() wait100ms()
  #define wait_about200ms() wait200ms()
  #define wait_about300ms() wait300ms()
  #define wait_about400ms() wait400ms()
  #define wait_about500ms() wait500ms()
  #define wait_about1s() wait1s()
  #define wait_about2s() wait2s()
  #define wait_about3s() wait3s()
  #define wait_about4s() wait4s()
#else
  // use sleep mode to save current for user interface
  #define wait_about5ms() sleep_5ms(1)
  #define wait_about10ms() sleep_5ms(2)
  #define wait_about20ms() sleep_5ms(4)
  #define wait_about30ms() sleep_5ms(6)
  #define wait_about50ms() sleep_5ms(10)
  #define wait_about100ms() sleep_5ms(20)
  #define wait_about200ms() sleep_5ms(40)
  #define wait_about300ms() sleep_5ms(60)
  #define wait_about400ms() sleep_5ms(80)
  #define wait_about500ms() sleep_5ms(100)
  #define wait_about1s() sleep_5ms(200)
  #define wait_about2s() sleep_5ms(400)
  #define wait_about3s() sleep_5ms(600)
  #define wait_about4s() sleep_5ms(800)
#endif
#define RR680PL (R_L_VAL + PIN_RP)
#define RR680MI (R_L_VAL + PIN_RM)
#define RRpinPL (PIN_RP)
#define RRpinMI (PIN_RM)
#define RESTART_DELAY_TICS 16384
#endif  // #ifndef ADC_PORT


// the hFE (B) can be determined with common collector and common emitter circuit
// with more than 16K both methodes are possible
#ifdef COMMON_COLLECTOR
  #if FLASHEND > 0x3fff
    #define COMMON_EMITTER
  #endif
#else
  #define COMMON_EMITTER
#endif


void UfOutput(uint8_t bcdnum);
void mVOutput(uint8_t nn);
void RvalOut(uint8_t ii);
void ChargePin10ms(uint8_t PinToCharge, uint8_t ChargeDirection);
void EntladePins();
void DisplayValue(unsigned long Value, int8_t Exponent, unsigned char Unit, unsigned char digits);
void sleep_5ms(uint16_t pause);
void PinLayout(char pin1, char pin2, char pin3);
void CheckPins(uint8_t HighPin, uint8_t LowPin, uint8_t TristatePin);
void GetIr(uint8_t hipin, uint8_t lopin);
unsigned int ReadADC(uint8_t Probe);
unsigned int W5msReadADC(uint8_t Probe);
unsigned int W10msReadADC(uint8_t Probe);
unsigned int W20msReadADC(uint8_t Probe);
void ReadCapacity(uint8_t HighPin, uint8_t LowPin);
unsigned int getRLmultip(unsigned int cvolt);
void Scale_C_with_vcc(void);
void ReadInductance(void);
uint16_t get_log(uint16_t permil);
uint16_t GetESR(uint8_t hipin, uint8_t lopin);
void us500delay(unsigned int us);
void GetVloss();
void Calibrate_UR(void);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_pgm_string(const unsigned char *data);
void lcd_testpin(unsigned char temp);
void lcd_fix_string(const unsigned char *data);

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */
#define COMMON
  /*
  The voltage at a capacitor grows with  Uc = VCC * (1 - e**(-t/T))
  The voltage 1.3V is reached at  t = -ln(3.7/5)*T  = 0.3011*T . 
  Time constant is  T = R * C ; also
  C = T / R
  for the resistor 470 kOhm  is C = t / (0.3011 * 470000)
  H_Fakt = 707/100 for a result in pF units.
  */

// Big Capacities (>50uF) are measured with up to 500 load-pulses with the 680 Ohm resistor.
// Each  of this load-puls has an length of 10ms. After every load-pulse the voltage of the
// capacitor is measured. If the voltage is more than 300mV, the capacity is computed by
// interpolating the corresponding values of the table RLtab and multiply that with the number
// of load pulses (*10).

// Widerstand 680 Ohm                300   325   350   375   400   425   450   475   500   525   550   575   600   625   650   675   700   725   750   775   800   825   850   875   900   925   950   975  1000  1025  1050  1075  1100  1125  1150  1175  1200  1225  1250  1275  1300  1325  1350  1375  1400  mV
const uint16_t RLtab[] MEM_TEXT = {22447,20665,19138,17815,16657,15635,14727,13914,13182,12520,11918,11369,10865,10401, 9973, 9577, 9209, 8866, 8546, 8247, 7966, 7702, 7454, 7220, 6999, 6789, 6591, 6403, 6224, 6054, 5892, 5738, 5590, 5449, 5314, 5185, 5061, 4942, 4828, 4718, 4613, 4511, 4413, 4319, 4228};

#if FLASHEND > 0x1fff
  //                                {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63,  64,  65,  66,  67,  68,  69,  70,  71,  72,  73,  74,  75,  76,  77,  78,  79,  80,  81,  82,  83,  84,  85,  86,  87,  88,  89,  90,  91 };
  const uint16_t LogTab[] PROGMEM = {0, 20, 41, 62, 83, 105, 128, 151, 174, 198, 223, 248, 274, 301, 329, 357, 386, 416, 446, 478, 511, 545, 580, 616, 654, 693, 734, 777, 821, 868, 916, 968, 1022, 1079, 1139, 1204, 1273, 1347, 1427, 1514, 1609, 1715, 1833, 1966, 2120, 2303, 2526 };
#endif

// with integer factors the ADC-value will be changed to mV resolution in ReadADC !
// all if statements are corrected to the mV resolution.


// Strings in PROGMEM or in EEprom

const unsigned char TestRunning[] MEM_TEXT = "testing...";
const unsigned char BatWeak[] MEM_TEXT = "weak";
const unsigned char BatEmpty[] MEM_TEXT = "empty!";
const unsigned char TestFailed2[] MEM_TEXT = "damaged ";
const unsigned char Component[] MEM_TEXT = "part";
const unsigned char Triac[] MEM_TEXT = "Triac";
const unsigned char Thyristor[] MEM_TEXT = "Thyristor";
const unsigned char Unknown[] MEM_TEXT = " unknown";
const unsigned char TestFailed1[] MEM_TEXT = "No, unknown, or";
const unsigned char OrBroken[] MEM_TEXT = "or damaged ";
const unsigned char TestTimedOut[] MEM_TEXT = "Timeout!";
#define Cathode_char 'C'

// Strings, which are not dependent of any language
const unsigned char Bat_str[] MEM_TEXT = "Bat. ";
const unsigned char OK_str[] MEM_TEXT = "OK";
const unsigned char mosfet_str[] MEM_TEXT = "-MOS";
const unsigned char jfet_str[] MEM_TEXT = "JFET";
const unsigned char GateCap_str[] MEM_TEXT = "C=";
const unsigned char hfe_str[] MEM_TEXT ="B=";
const unsigned char NPN_str[] MEM_TEXT = "NPN ";
const unsigned char PNP_str[] MEM_TEXT = "PNP ";
const unsigned char N123_str[] MEM_TEXT = " 123=";
const unsigned char Uf_str[] MEM_TEXT = "Uf=";
const unsigned char vt_str[] MEM_TEXT = " Vt=";
const unsigned char Vgs_str[] MEM_TEXT = "@Vgs=";
const unsigned char CapZeich[] MEM_TEXT = {'-', 186,'-',0};
const unsigned char Cell_str[] MEM_TEXT = "Cell!";
const unsigned char VCC_str[] MEM_TEXT = "VCC=";
const unsigned char ESR_str[] MEM_TEXT = " ESR=";
const unsigned char VLOSS_str[] MEM_TEXT = " Vloss=";
const unsigned char Lis_str[] MEM_TEXT = "L=";
const unsigned char Ir_str[] MEM_TEXT = "  Ir=";
const unsigned char VERSION_str[] MEM2_TEXT = "Ttester 69.69.69";
const unsigned char AnKat[] MEM_TEXT = {'+', 16, '|', '-',0};
const unsigned char KatAn[] MEM_TEXT = {'-', '|', 17, '+',0};
const unsigned char Diodes[] MEM_TEXT = {'*','?', '*', ' ',0};
const unsigned char Resistor_str[] MEM_TEXT = {'-', '[', '=', ']','-',0};

const unsigned char PinRLtab[] PROGMEM = { (1<<(TP1*2)), (1<<(TP2*2)), (1<<(TP3*2))};  // Table of commands to switch the  R-L resistors Pin 0,1,2
const unsigned char PinADCtab[] PROGMEM = { (1<<TP1), (1<<TP2), (1<<TP3)};  // Table of commands to switch the ADC-Pins 0,1,2


/*
// generate Omega- and u-character as Custom-character, if these characters has a number of loadable type
#if LCD_CHAR_OMEGA < 8
  const unsigned char CyrillicOmegaIcon[] MEM_TEXT = {0,0,14,17,17,10,27,0};	// Omega
#endif
#if LCD_CHAR_U < 8
  const unsigned char CyrillicMuIcon[] MEM_TEXT = {0,17,17,17,19,29,16,16};	// micro
#endif
*/

#ifdef AUTO_CAL
  //const uint16_t R680pl EEMEM = R_L_VAL+PIN_RP;	// total resistor to VCC
  //const uint16_t R680mi EEMEM = R_L_VAL+PIN_RM;	// total resistor to GND
  const int8_t RefDiff EEMEM = REF_R_KORR;		// correction of internal Reference Voltage
#endif

const uint8_t PrefixTab[] MEM_TEXT = { 'p','n','u','m',0,'k','M'};  // p,n,u,m,-,k,M

#ifdef AUTO_CAL
  //const uint16_t cap_null EEMEM = C_NULL;	// Zero offset of capacity measurement 
  const int16_t ref_offset EEMEM = REF_C_KORR;	// default correction of internal reference voltage for capacity measurement
  // LoPin:HiPin                        2:1    3:1    1:2                    :     3:2                   1:3    2:3
  const uint8_t c_zero_tab[] EEMEM = { C_NULL,C_NULL,C_NULL+TP2_CAP_OFFSET,C_NULL,C_NULL+TP2_CAP_OFFSET,C_NULL,C_NULL }; // table of zero offsets
#endif

const uint8_t EE_ESR_ZEROtab[] PROGMEM = {ESR_ZERO, ESR_ZERO, ESR_ZERO, ESR_ZERO};	// zero offset of ESR measurement

// End of EEPROM-Strings

// Multiplier for capacity measurement with R_H (470KOhm)
unsigned int RHmultip = DEFAULT_RH_FAKT;


struct Diode_t {
  uint8_t Anode;
  uint8_t Cathode;
  unsigned int Voltage;
};

COMMON struct Diode_t diodes[6];
COMMON uint8_t NumOfDiodes;

COMMON struct {
  unsigned long hfe[2];		// current amplification factor 
  unsigned int uBE[2];		// B-E-voltage of the Transistor
  uint8_t b,c,e;		// pins of the Transistor
}trans;

COMMON unsigned int gthvoltage;	// Gate-threshold voltage 

COMMON uint8_t PartReady;	// part detection is finished 
COMMON uint8_t PartMode;
COMMON uint8_t tmpval, tmpval2;
COMMON unsigned int ref_mv;     // Reference-voltage  in mV units

COMMON struct resis_t{
  unsigned long rx;		// value of resistor RX  
  #if FLASHEND > 0x1fff
    unsigned long lx;		// inductance 10uH or 100uH
    int8_t lpre;		// prefix for inductance
  #endif
  uint8_t ra,rb;		// Pins of RX
  uint8_t rt;			// Tristate-Pin (inactive)
} resis[3];

COMMON uint8_t ResistorsFound;	// Number of found resistors
COMMON uint8_t ii;		// multipurpose counter

COMMON struct cap_t {
  unsigned long cval;		// capacitor value 
  unsigned long cval_max;	// capacitor with maximum value
  union t_combi{
  unsigned long dw;		// capacity value without corrections
  uint16_t w[2];
  } cval_uncorrected;
  #if FLASHEND > 0x1fff
    unsigned int esr;		// serial resistance of C in 0.01 Ohm
    unsigned int v_loss;	// voltage loss 0.1%
  #endif
  uint8_t ca, cb;		// pins of capacitor
  int8_t cpre;			// Prefix for capacitor value  -12=p, -9=n, -6=u, -3=m
  int8_t cpre_max;		// Prefix of the biggest capacitor
} cap;

#ifndef INHIBIT_SLEEP_MODE
  // with sleep mode we need a global ovcnt16
  COMMON volatile uint16_t ovcnt16;
  COMMON volatile uint8_t unfinished;
#endif

COMMON int16_t load_diff;	// difference voltage of loaded capacitor and internal reference

COMMON uint8_t WithReference;	// Marker for found precision voltage reference = 1
COMMON uint8_t PartFound;	// the found part 
COMMON char outval[12];		// String for ASCII-outpu
COMMON uint8_t empty_count;	// counter for max count of empty measurements
COMMON uint8_t mess_count;	// counter for max count of nonempty measurements

COMMON struct ADCconfig_t {
  uint8_t Samples;		// number of ADC samples to take
  uint8_t RefFlag;		// save Reference type VCC of IntRef
  uint16_t U_Bandgap;		// Reference Voltage in mV
  uint16_t U_AVCC;		// Voltage of AVCC
} ADCconfig;

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

// definitions of parts
#define PART_NONE 0
#define PART_DIODE 1
#define PART_TRANSISTOR 2
#define PART_FET 3
#define PART_TRIAC 4
#define PART_THYRISTOR 5
#define PART_RESISTOR 6
#define PART_CAPACITOR 7
#define PART_CELL 8

// special definition for different parts 
// FETs
#define PART_MODE_N_E_MOS 2
#define PART_MODE_P_E_MOS 3
#define PART_MODE_N_D_MOS 4
#define PART_MODE_P_D_MOS 5
#define PART_MODE_N_JFET 6
#define PART_MODE_P_JFET 7

// Bipolar
#define PART_MODE_NPN 1
#define PART_MODE_PNP 2

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

// wait functions
#define  wait5s()    delay(5000)
#define  wait4s()    delay(4000)
#define  wait3s()    delay(3000)
#define  wait2s()    delay(2000)
#define  wait1s()    delay(1000)
#define  wait500ms() delay(500)
#define  wait400ms() delay(400)
#define  wait300ms() delay(300)
#define  wait200ms() delay(200)
#define  wait100ms() delay(100)
#define  wait50ms()  delay(50)
#define  wait40ms()  delay(40)
#define  wait30ms()  delay(30)
#define  wait20ms()  delay(20)
#define  wait10ms()  delay(10)
#define  wait5ms()   delay(5)
#define  wait4ms()   delay(4)
#define  wait3ms()   delay(3)
#define  wait2ms()   delay(2)
#define  wait1ms()   delay(1)
#define  wait500us() delayMicroseconds(500)
#define  wait400us() delayMicroseconds(400)
#define  wait300us() delayMicroseconds(300)
#define  wait200us() delayMicroseconds(200)
#define  wait100us() delayMicroseconds(100)
#define  wait50us()  delayMicroseconds(50)
#define  wait40us()  delayMicroseconds(40)
#define  wait30us()  delayMicroseconds(30)
#define  wait20us()  delayMicroseconds(20)
#define  wait10us()  delayMicroseconds(10)
#define  wait5us()   delayMicroseconds(5)
#define  wait4us()   delayMicroseconds(4)
#define  wait3us()   delayMicroseconds(3)
#define  wait2us()   delayMicroseconds(2)
#define  wait1us()   delayMicroseconds(1)

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

// Interfacing of a HD44780 compatible LCD with 4-Bit-Interface mode

// LCD-commands
#define CMD_ClearDisplay         0x01
#define CMD_ReturnHome           0x02
#define CMD_SetEntryMode         0x04
#define CMD_SetDisplayAndCursor  0x08
#define CMD_SetIFOptions         0x20
#define CMD_SetCGRAMAddress      0x40    // for Custom character
#define CMD_SetDDRAMAddress      0x80    // set Cursor 

#define CMD1_SetBias             0x10	 // set Bias (instruction table 1, DOGM)
#define CMD1_PowerControl        0x50	 // Power Control, set Contrast C5:C4 (instruction table 1, DOGM)
#define CMD1_FollowerControl     0x60	 // Follower Control, amplified ratio (instruction table 1, DOGM)
#define CMD1_SetContrast         0x70	 // set Contrast C3:C0 (instruction table 1, DOGM)

// Makros for LCD
#define lcd_line1() lcd_set_cursor(0,0)  // move to beginning of 1 row
#define lcd_line2() lcd_set_cursor(1,0)  // move to beginning of 2 row
#define lcd_line3() lcd_set_cursor(2,0)  // move to beginning of 3 row
#define lcd_line4() lcd_set_cursor(3,0)  // move to beginning of 4 row

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

#ifndef INHIBIT_SLEEP_MODE
  // prepare sleep mode
  EMPTY_INTERRUPT(TIMER2_COMPA_vect);
  EMPTY_INTERRUPT(ADC_vect);
#endif

uint8_t tmp = 0;
//unsigned int PRR;

byte TestKey;
byte TestKeyPin = 7;  // A3

Adafruit_SSD1306 display(128, 64, &Wire, -1);

// begin of transistortester program
void setup(){
  pinMode(TestKeyPin, INPUT_PULLUP);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.cp437(true);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("Transistor");
  lcd_set_cursor(1, 0);
  display.println("Tester");
  lcd_set_cursor(2, 0);
  display.println("for Arduino");
  lcd_set_cursor(3, 0);
  display.println("69.69.69");

  // ADC-Init
  ADCSRA = (1<<ADEN) | AUTO_CLOCK_DIV;	// prescaler=8 or 64 (if 8Mhz clock)
  #define WDRF_HOME MCUSR
  #define T2_PERIOD (1024 / (F_CPU / 1000000UL));	// set to 128 or 64 us 
	
  if(tmp) { 
    // check if Watchdog-Event 
    // this happens, if the Watchdog is not reset for 2s
    // can happen, if any loop in the Program doen't finish.
    lcd_line1();
    lcd_fix_string(TestTimedOut);	// Output Timeout
    wait_about3s();			// wait for 3 s
    //ON_PORT = 0;			// shut off!
    //ON_DDR = (1<<ON_PIN);		// switch to GND
    //return;
  }

  #define display_time OFF_WAIT_TIME

  empty_count = 0;
  mess_count = 0;
}

void loop()
{
  // Entry: if start key is pressed before shut down
start:
  display.display();
  TestKey = 1;
  while(TestKey) {
    TestKey = digitalRead(TestKeyPin);
    delay(100);
  }
  while(!TestKey) {
    TestKey = digitalRead(TestKeyPin);
    delay(100);
  }
  display.clearDisplay();
  delay(100);

  PartFound = PART_NONE;	// no part found
  NumOfDiodes = 0;		// Number of diodes = 0
  PartReady = 0;
  PartMode = 0;
  WithReference = 0;		// no precision reference voltage
  ADC_DDR = TXD_MSK;		// activate Software-UART 
  ResistorsFound = 0;		// no resistors found
  cap.ca = 0;
  cap.cb = 0;

  ADCconfig.RefFlag = 0;
  Calibrate_UR();		// get Ref Voltages and Pin resistance
  lcd_line1();    // 1 row
  
  ADCconfig.U_Bandgap = ADC_internal_reference;  // set internal reference voltage for ADC
  lcd_fix2_string(VERSION_str);	// if no Battery check, Version .. in row 1

  // begin tests
  #if FLASHEND > 0x1fff
    if (WithReference) {
      // 2.5V precision reference is checked OK
      if ((mess_count == 0) && (empty_count == 0)) {
        // display VCC= only first time
        lcd_line2();
        lcd_fix_string(VCC_str);			// VCC=
        DisplayValue(ADCconfig.U_AVCC,-3,'V',3);	// Display 3 Digits of this mV units
        wait_about1s();
      }
    }
  #endif
  lcd_line2();			// LCD position row 2, column 1
  lcd_fix_string(TestRunning);	// String: testing...
  lcd_line2();		// LCD position row 2, column 1
  display.display();
  display.setCursor(0,0);

  delay(5);

  EntladePins();		// discharge all capacitors!

  if(PartFound == PART_CELL) {
    display.clearDisplay();
    lcd_fix_string(Cell_str);	// display "Cell!"
    goto end2;
  }
  // check all 6 combinations for the 3 pins 
  //       High  Low  Tri
  CheckPins(TP1, TP2, TP3);
  CheckPins(TP2, TP1, TP3);
  CheckPins(TP1, TP3, TP2);
  CheckPins(TP3, TP1, TP2);
  CheckPins(TP2, TP3, TP1);
  CheckPins(TP3, TP2, TP1);
  
  // separate check if is is a capacitor
  if(((PartFound == PART_NONE) || (PartFound == PART_RESISTOR) || (PartFound == PART_DIODE)) ) {
    EntladePins();		// discharge capacities
    // measurement of capacities in all 3 combinations
    cap.cval_max = 0;		// set max to zero
    cap.cpre_max = -12;		// set max to pF unit

    ReadCapacity(TP3, TP1);
    ReadCapacity(TP3, TP2);
    ReadCapacity(TP2, TP1);

    #if FLASHEND > 0x1fff
      ReadInductance();		// measure inductance
    #endif
  }

  // All checks are done, output result to display
  display.clearDisplay();

  if(PartFound == PART_DIODE) {
    if(NumOfDiodes == 1) {		// single Diode
      //lcd_fix_string(Diode);		// "Diode: "

      #if FLASHEND > 0x1fff
        // enough memory to sort the pins
        // the higher test pin number is right side
        if (diodes[0].Anode < diodes[0].Cathode) {
          lcd_testpin(diodes[0].Anode);
          lcd_fix_string(AnKat);          // "->|-"
          lcd_testpin(diodes[0].Cathode);
        } else {
          lcd_testpin(diodes[0].Cathode);
          lcd_fix_string(KatAn);          // "-|<-"
          lcd_testpin(diodes[0].Anode);
        }
      #endif
      #if FLASHEND > 0x1fff
        GetIr(diodes[0].Cathode,diodes[0].Anode);
      #endif

      UfOutput(0x70);

      lcd_line3();

      // load current of capacity is (5V-1.1V)/(470000 Ohm) = 8298nA
      lcd_fix_string(GateCap_str);			// "C="
      ReadCapacity(diodes[0].Cathode,diodes[0].Anode);	// Capacity opposite flow direction
      DisplayValue(cap.cval,cap.cpre,'F',3);
      goto end;

    } else if(NumOfDiodes == 2) { 	// double diode
      display.write('2');
      lcd_fix_string(Diodes);		// "diodes "

      if(diodes[0].Anode == diodes[1].Anode) { //Common Anode
        lcd_testpin(diodes[0].Cathode);
        lcd_fix_string(KatAn);          // "-|<-"
        lcd_testpin(diodes[0].Anode);
        lcd_fix_string(AnKat);          // "->|-"
        lcd_testpin(diodes[1].Cathode);
        UfOutput(0x01);
        goto end;

      } else if(diodes[0].Cathode == diodes[1].Cathode) { //Common Cathode
        lcd_testpin(diodes[0].Anode);
        lcd_fix_string(AnKat);          // "->|-"
        lcd_testpin(diodes[0].Cathode);
        lcd_fix_string(KatAn);          // "-|<-"
        lcd_testpin(diodes[1].Anode);
        UfOutput(0x01);
        goto end;

      } else if ((diodes[0].Cathode == diodes[1].Anode) && (diodes[1].Cathode == diodes[0].Anode)) {
        // Antiparallel
        lcd_testpin(diodes[0].Anode);
        lcd_fix_string(AnKat);          // "->|-"
        lcd_testpin(diodes[0].Cathode);
        lcd_fix_string(AnKat);          // "->|-"
        lcd_testpin(diodes[1].Cathode);
        UfOutput(0x01);
        goto end;
      }

    } else if(NumOfDiodes == 3) {
      // Serial of 2 Diodes; was detected as 3 Diodes 
      trans.b = 3;
      trans.c = 3;

      // Check for any constallation of 2 serial diodes:
      // Only once the pin No of anyone Cathode is identical of another anode.
      // two diodes in series is additionally detected as third big diode.

      if(diodes[0].Cathode == diodes[1].Anode) trans.b = 0,trans.c = 1;
      if(diodes[0].Anode == diodes[1].Cathode) trans.b = 1,trans.c = 0;
      if(diodes[0].Cathode == diodes[2].Anode) trans.b = 0,trans.c = 2;
      if(diodes[0].Anode == diodes[2].Cathode) trans.b = 2,trans.c = 0;
      if(diodes[1].Cathode == diodes[2].Anode) trans.b = 1,trans.c = 2;
      if(diodes[1].Anode == diodes[2].Cathode) trans.b = 2,trans.c = 1;

      if((trans.b < 3) && (trans.c < 3)) {
        display.write('3');
        lcd_fix_string(Diodes);			// "Diodes "
        lcd_testpin(diodes[trans.b].Anode);
        lcd_fix_string(AnKat);			// "->|-"
        lcd_testpin(diodes[trans.b].Cathode);
        lcd_fix_string(AnKat);			// "->|-"
        lcd_testpin(diodes[trans.c].Cathode);
        UfOutput( (trans.b<<4)|trans.c);
        goto end;
      }
    }
    // end (PartFound == PART_DIODE)

  } else if (PartFound == PART_TRANSISTOR) {
    if(PartReady != 0) {
      if((trans.hfe[0]>trans.hfe[1])) {
        // if the amplification factor was higher at first testr: swap C and E !
        tmp = trans.c;
        trans.c = trans.e;
        trans.e = tmp;
      } else {
        trans.hfe[0] = trans.hfe[1];
        trans.uBE[0] = trans.uBE[1];
      }
    }

    if(PartMode == PART_MODE_NPN) lcd_fix_string(NPN_str);		// "NPN "
    else lcd_fix_string(PNP_str);		// "PNP "

    if( NumOfDiodes > 2) {	// Transistor with protection diode
        // Layout with 123= style
        if (((PartMode == PART_MODE_NPN) && (trans.c > trans.e)) || ((PartMode != PART_MODE_NPN) && (trans.c < trans.e))) lcd_fix_string(AnKat);  // "->|-"
        else lcd_fix_string(KatAn);  // "-|<-"
    }

    lcd_line2();

    PinLayout('E','B','C'); 		// EBC= or 123=...

    lcd_line3();

    lcd_fix_string(hfe_str);		// "B="  (hFE)
    DisplayValue(trans.hfe[0],0,0,3);
    display.write(' ');

    lcd_line4();
    lcd_fix_string(Uf_str);		// "Uf="
    DisplayValue(trans.uBE[0],-3,'V',3);
    goto end;

    // end (PartFound == PART_TRANSISTOR)

  } else if (PartFound == PART_FET) {	// JFET or MOSFET

    if(PartMode&1) display.write('P');			// P-channel
    else display.write('N');			// N-channel
    display.write('-');

    tmp = PartMode/2;
    if (tmp == (PART_MODE_N_D_MOS/2)) display.write('D');			// N-D
    if (tmp == (PART_MODE_N_E_MOS/2)) display.write('E');			// N-E
    if (tmp == (PART_MODE_N_JFET/2)) lcd_fix_string(jfet_str);         // "JFET"
    else lcd_fix_string(mosfet_str);       // "-MOS "

    lcd_line2();

    PinLayout('S','G','D'); 		// SGD= or 123=...

    if((NumOfDiodes > 0) && (PartMode < PART_MODE_N_D_MOS)) {
      // MOSFET with protection diode; only with enhancement-FETs
      // layout with 123= style
      if (((PartMode&1) && (trans.c < trans.e)) || ((!(PartMode&1)) && (trans.c > trans.e)))
        {
          display.write(LCD_CHAR_DIODE1);	// show Diode symbol >|
        } else {
          display.write(LCD_CHAR_DIODE2);	// show Diode symbol |<
        }
    }

    lcd_line3();
    if(PartMode < PART_MODE_N_D_MOS) {		// enhancement-MOSFET
      // Gate capacity
      lcd_fix_string(GateCap_str);		// "C="
      ReadCapacity(trans.b,trans.e);		// measure capacity
      DisplayValue(cap.cval,cap.cpre,'F',3);

      lcd_line4();

      lcd_fix_string(vt_str);			// "Vt="
    } else {
      display.write('I');
      display.write('=');
      DisplayValue(trans.uBE[1],-5,'A',2);
      lcd_line4();
      lcd_fix_string(Vgs_str);			// " Vgs="
    }

    // Gate-threshold voltage
    DisplayValue(gthvoltage,-3,'V',2);
    goto end;

    // end (PartFound == PART_FET)

  } else if (PartFound == PART_THYRISTOR) {
    lcd_fix_string(Thyristor);			// "Thyristor"
    goto gakOutput;

  } else if (PartFound == PART_TRIAC) {
    lcd_fix_string(Triac);			// "Triac"
    goto gakOutput;

  } else if(PartFound == PART_RESISTOR) {
    if (ResistorsFound == 1) {		 	// single resistor
      lcd_testpin(resis[0].rb);			// Pin-number 1
      lcd_fix_string(Resistor_str);
      lcd_testpin(resis[0].ra);			// Pin-number 2
    } else {					// R-Max suchen
      ii = 0;
      if (resis[1].rx > resis[0].rx) ii = 1;
      if (ResistorsFound == 2) ii = 2;
      else if (resis[2].rx > resis[ii].rx) ii = 2;

      char x = '1';
      char y = '3';
      char z = '2';
   
      if (ii == 1) {
        //x = '1';
        y = '2';
        z = '3';
      }

      if (ii == 2) {
        x = '2';
        y = '1';
        z = '3';
      }

      display.write(x);
      lcd_fix_string(Resistor_str);    // "-[=]-"
      display.write(y);
      lcd_fix_string(Resistor_str);    // "-[=]-"
      display.write(z);
    }

    lcd_line2();  // 2 row 

    if (ResistorsFound == 1) {
      RvalOut(0);

      #if FLASHEND > 0x1fff
        if (resis[0].lx != 0) {
          // resistor have also Inductance

          #if defined(NOK5110) || defined(OLED096)
            lcd_line3();
          #endif

          lcd_fix_string(Lis_str);				// "L="
          DisplayValue(resis[0].lx,resis[0].lpre,'H',3);	// output inductance
        }
      #endif

    } else {
      // output resistor values in right order
      if (ii == 0) {
        RvalOut(1);
        RvalOut(2);
      }
      if (ii == 1) {
        RvalOut(0);
        RvalOut(2);
      }
      if (ii == 2) {
        RvalOut(0);
        RvalOut(1);
      }
    }
    goto end;

    // end (PartFound == PART_RESISTOR)

  // capacity measurement is wanted
  } else if(PartFound == PART_CAPACITOR) {
    //lcd_fix_string(Capacitor);
    lcd_testpin(cap.ca);		// Pin number 1
    lcd_fix_string(CapZeich);		// capacitor sign
    lcd_testpin(cap.cb);		// Pin number 2

    #if FLASHEND > 0x1fff
      GetVloss();			// get Voltage loss of capacitor
      if (cap.v_loss != 0) {

        #if defined(NOK5110) || defined(OLED096)
          lcd_line4();
        #endif

        lcd_fix_string(VLOSS_str);	// "  Vloss="
        DisplayValue(cap.v_loss,-1,'%',2);
      }
    #endif

    lcd_line2();  // 2 row 
    DisplayValue(cap.cval_max,cap.cpre_max,'F',4);

    #if FLASHEND > 0x1fff
      cap.esr = GetESR(cap.cb, cap.ca);		// get ESR of capacitor
      if (cap.esr < 65530) {

        #if defined(NOK5110) || defined(OLED096)
          lcd_line3();
        #endif

        lcd_fix_string(ESR_str);
        DisplayValue(cap.esr,-2,234,2);
      }
    #endif

    goto end;
  }

  if(NumOfDiodes == 0) { 		// no diodes are found
    lcd_fix_string(TestFailed1); 	// "No, unknown, or"
    lcd_line2(); 			// 2 row 
    lcd_fix_string(TestFailed2); 	// "damaged "
    lcd_fix_string(Component);		// "part"
  } else {
    lcd_fix_string(Component);		// "part"
    lcd_fix_string(Unknown); 		// " unknown"
    lcd_line2(); 			// 2 row 
    lcd_fix_string(OrBroken); 		// "or damaged "
    display.write(NumOfDiodes + '0');
    lcd_fix_string(AnKat);		// "->|-"
  }

  empty_count++;
  mess_count = 0;
  goto end2;

gakOutput:
  lcd_line2();  // 2 row 
  PinLayout(Cathode_char,'G','A'); 	// CGA= or 123=...

//- - - - - - - - - - - - - - - - - - - - - - - - - - - -
end:
  empty_count = 0;		// reset counter, if part is found
  mess_count++;			// count measurements

end2:
  //ADC_DDR = (1<<TPREF) | TXD_MSK;  // switch pin with reference to GND, release relay
  ADC_DDR = TXD_MSK;                 // switch pin with reference to GND, release relay
  goto start;

  while(!(ON_PIN_REG & (1<<RST_PIN)));	// wait ,until button is released
  wait_about200ms();
  // wait 14 seconds or 5 seconds (if repeat function)

  for(gthvoltage = 0;gthvoltage<display_time;gthvoltage+=10) {
    if(!(ON_PIN_REG & (1<<RST_PIN))) {
      // If the key is pressed again... 
      // goto start of measurement 
      goto start;
    }
    wdt_reset();
    wait_about10ms();
  }
  goto start;  // POWER_OFF not selected, repeat measurement

  return;
} // end main


//******************************************************************
// output of flux voltage for 1-2 diodes in row 2
// bcdnum = Numbers of both Diodes:
// higher 4 Bit  number of first Diode
// lower 4 Bit  number of second Diode (Structure diodes[nn])
// if number >= 3  no output is done
void UfOutput(uint8_t bcdnum) {
  lcd_line2(); 				// 2 row
  lcd_fix_string(Uf_str);		// "Uf="
  mVOutput(bcdnum >> 4);
  mVOutput(bcdnum & 0x0f);
}

void mVOutput(uint8_t nn) {
  if (nn < 3) {
    // Output in mV units
    DisplayValue(diodes[nn].Voltage,-3,'V',3);
    display.write(' ');
  }
}

void RvalOut(uint8_t ii) {	
  // output of resistor value
    uint16_t rr;
    if ((resis[ii].rx < 100) && (resis[0].lx == 0)) rr = GetESR(resis[ii].ra,resis[ii].rb),DisplayValue(rr,-2,234,3);
    else DisplayValue(resis[ii].rx,-1,234,4);

  display.write(' ');
}

//******************************************************************

void ChargePin10ms(uint8_t PinToCharge, uint8_t ChargeDirection) {
  // Load the specified pin to the specified direction with 680 Ohm for 10ms.
  // Will be used by discharge of MOSFET Gates or to load big capacities.
  // Parameters:
  // PinToCharge: specifies the pin as mask for R-Port
  // ChargeDirection: 0 = switch to GND (N-Kanal-FET), 1= switch to VCC(P-Kanal-FET)

  if(ChargeDirection&1) {
    R_PORT |= PinToCharge;	// R_PORT to 1 (VCC) 
  } else {
    R_PORT &= ~PinToCharge; 	// or 0 (GND)
  }

  R_DDR |= PinToCharge;			// switch Pin to output, across R to GND or VCC
  wait_about10ms();			// wait about 10ms
  // switch back Input, no current
  R_DDR &= ~PinToCharge;		// switch back to input
  R_PORT &= ~PinToCharge;		// no Pull up
}


// first discharge any charge of capacitors
void EntladePins() {
  uint8_t adc_gnd;		// Mask of ADC-outputs, which can be directly connected to GND
  unsigned int adcmv[3];	// voltages of 3 Pins in mV
  unsigned int clr_cnt;		// Clear Counter
  uint8_t lop_cnt;		// loop counter

  // max. time of discharge in ms  (10000/20) == 10s
  #define MAX_ENTLADE_ZEIT  (10000/20)

  for(lop_cnt=0;lop_cnt<10;lop_cnt++) {
    adc_gnd = TXD_MSK;		// put all ADC to Input
    ADC_DDR = adc_gnd;
    ADC_PORT = TXD_VAL;		// ADC-outputs auf 0
    R_PORT = 0;			// R-outputs auf 0
    R_DDR = (2<<(TP3*2)) | (2<<(TP2*2)) | (2<<(TP1*2));  // R_H for all Pins to GND

    adcmv[0] = W5msReadADC(TP1);	// which voltage has Pin 1?
    adcmv[1] = ReadADC(TP2);		// which voltage has Pin 2?
    adcmv[2] = ReadADC(TP3);		// which voltage has Pin 3?

    if ((PartFound == PART_CELL) || (adcmv[0] < CAP_EMPTY_LEVEL) 
                                  & (adcmv[1] < CAP_EMPTY_LEVEL) 
                                  & (adcmv[2] < CAP_EMPTY_LEVEL)) {
      ADC_DDR = TXD_MSK;		// switch all ADC-Pins to input
      R_DDR = 0;			// switch all R_L Ports (and R_H) to input
      return;				// all is discharged
    }

    // all Pins with voltage lower than 1V can be connected directly to GND (ADC-Port)
    if (adcmv[0] < 1000) adc_gnd |= (1<<TP1);	// Pin 1 directly to GND
    if (adcmv[1] < 1000) adc_gnd |= (1<<TP2);	// Pin 2 directly to GND
    if (adcmv[2] < 1000) adc_gnd |= (1<<TP3);	// Pin 3 directly to  GND
    ADC_DDR = adc_gnd;		// switch all selected ADC-Ports at the same time

    // additionally switch the leaving Ports with R_L to GND.
    // since there is no disadvantage for the already directly switched pins, we can
    // simply switch all  R_L resistors to GND
    R_DDR = (1<<(TP3*2)) | (1<<(TP2*2)) | (1<<(TP1*2));	// Pins across R_L resistors to GND

    for(clr_cnt=0;clr_cnt<MAX_ENTLADE_ZEIT;clr_cnt++) {
      wdt_reset();
      adcmv[0] = W20msReadADC(TP1);	// which voltage has Pin 1?
      adcmv[1] = ReadADC(TP2);		// which voltage has Pin 2?
      adcmv[2] = ReadADC(TP3);		// which voltage has Pin 3?

      if (adcmv[0] < 1300) ADC_DDR |= (1<<TP1);	// below 1.3V , switch directly with ADC-Port to GND
      if (adcmv[1] < 1300) ADC_DDR |= (1<<TP2);	// below 1.3V, switch directly with ADC-Port to GND
      if (adcmv[2] < 1300) ADC_DDR |= (1<<TP3);	// below 1.3V, switch directly with ADC-Port to GND
      if ((adcmv[0] < (CAP_EMPTY_LEVEL+2)) && (adcmv[1] < (CAP_EMPTY_LEVEL+2)) && (adcmv[2] < (CAP_EMPTY_LEVEL+2))) break;
    }

    if (clr_cnt == MAX_ENTLADE_ZEIT) {
      PartFound = PART_CELL;		// mark as Battery
      // there is charge on capacitor, warn later!
    }

    for(adcmv[0]=0;adcmv[0]<clr_cnt;adcmv[0]++) {
      // for safety, discharge 5% of discharge  time
      wait1ms();
    }
  }  // end for lop_cnt
}


/* ************************************************************************
 *   display of values and units
 * ************************************************************************ */
/*
 *  display value and unit
 *  - max. 4 digits excluding "." and unit
 *
 *  requires:
 *  - value
 *  - exponent of factor related to base unit (value * 10^x)
 *    e.g: p = 10^-12 -> -12
 *  - unit character (0 = none)
 *  digits = 2, 3 or 4
 */
void DisplayValue(unsigned long Value, int8_t Exponent, unsigned char Unit, unsigned char digits)
{
  char OutBuffer[15];
  unsigned int      Limit;
  unsigned char     Prefix;		// prefix character
  uint8_t           Offset;		// exponent of offset to next 10^3 step
  uint8_t           Index;		// index ID
  uint8_t           Length;		// string length

  Limit = 100;				// scale value down to 2 digits
  if (digits == 3) Limit = 1000;	// scale value down to 3 digits
  if (digits == 4) Limit = 10000;	// scale value down to 4 digits

  while (Value >= Limit)
  {
    Value += 5;				// for automatic rounding
    Value = Value / 10;			// scale down by 10^1
    Exponent++;				// increase exponent by 1
  }

  // determine prefix

  Length = Exponent + 12;
  if ((int8_t)Length <  0) Length = 0;	// Limit to minimum prefix
  if (Length > 18) Length = 18;		// Limit to maximum prefix
  Index = Length / 3;
  Offset = Length % 3;

  if (Offset > 0)
    {
      Index++;				// adjust index for exponent offset, take next prefix
      Offset = 3 - Offset;		// reverse value (1 or 2)
    }

  Prefix = MEM_read_byte((uint8_t *)(&PrefixTab[Index]));   // look up prefix in table

  // display value

  // convert value into string
  utoa((unsigned int)Value, OutBuffer, 10);
  Length = strlen(OutBuffer);

  // position of dot
  Exponent = Length - Offset;		// calculate position

  if (Exponent <= 0)			// we have to prepend "0."
  {
    // 0: factor 10 / -1: factor 100
    display.write('.');
    if (Exponent < 0) display.write('0');	// extra 0 for factor 100
  }

  if (Offset == 0) Exponent = -1;	// disable dot if not needed

  // adjust position to array or disable dot if set to 0
  //Exponent--;

  // display value and add dot if requested
  Index = 0;
  while (Index < Length)		// loop through string
  {
    display.write(OutBuffer[Index]);		// display char
    Index++;				// next one
    if (Index == Exponent) {
      display.write('.');			// display dot
    }
  }

  // display prefix and unit
  if (Prefix != 0) display.write(Prefix);
  if (Unit) display.write(Unit);
}


#ifndef INHIBIT_SLEEP_MODE
// set the processor to sleep state
// wake up will be done with compare match interrupt of counter 2
void sleep_5ms(uint16_t pause){
  // pause is the delay in 5ms units
  uint8_t t2_offset;

  #define RESTART_DELAY_US (RESTART_DELAY_TICS/(F_CPU/1000000UL))
  // for 8 MHz crystal the Restart delay is 16384/8 = 2048us

  while (pause > 0) {

    if (pause > 1) {
      // Startup time is too long with 1MHz Clock!!!!
      t2_offset = (10000 - RESTART_DELAY_US) / T2_PERIOD;	// set to 10ms above the actual counter
      pause -= 2;
    } else {
      t2_offset = (5000 - RESTART_DELAY_US) / T2_PERIOD;	// set to 5ms above the actual counter
      pause = 0;
    }

    OCR2A = TCNT2 + t2_offset;			// set the compare value
    TIMSK2 = (0<<OCIE2B) | (1<<OCIE2A) | (0<<TOIE2);  // enable output compare match A interrupt

    set_sleep_mode(SLEEP_MODE_PWR_SAVE);
    //set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_mode();
    // wake up after output compare match interrupt

    wdt_reset();
  }

  TIMSK2 = (0<<OCIE2B) | (0<<OCIE2A) | (0<<TOIE2);  // disable output compare match A interrupt
}
#endif


// show the Pin Layout of the device 
void PinLayout(char pin1, char pin2, char pin3) {
  // pin1-3 is EBC or SGD or CGA
  // Layout with 123= style
  lcd_fix_string(N123_str);			// " 123="
  for (ii=0;ii<3;ii++) {
    if (ii == trans.e)  display.write(pin1);	// Output Character in right order
    if (ii == trans.b)  display.write(pin2);
    if (ii == trans.c)  display.write(pin3);
  }
}

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

//******************************************************************
void CheckPins(uint8_t HighPin, uint8_t LowPin, uint8_t TristatePin) {
  /*
  Function for checking the characteristic of a component with the following pin assignment 
  parameters:
  HighPin: Pin, which will be switched to VCC at the beginning
  LowPin: Pin, which will be switch to GND at the beginning
  TristatePin: Pin, which will be undefined at the beginning
  TristatePin will be switched to GND and VCC also .
  */
  struct {
    unsigned int lp_otr;
    unsigned int hp1;
    unsigned int hp2;
    unsigned int hp3;
    unsigned int lp1;
    unsigned int lp2;
    unsigned int tp1;
    unsigned int tp2;
  } adc;
  uint8_t LoPinRL;		// mask to switch the LowPin with R_L
  uint8_t LoPinRH;		// mask to switch the LowPin with R_H
  uint8_t TriPinRL;		// mask to switch the TristatePin with R_L
  uint8_t TriPinRH;		// mask to switch the TristatePin with R_H
  uint8_t HiPinRL;		// mask to switch the HighPin with RL
  uint8_t HiPinRH;		// mask to switch the HighPin with R_H
  uint8_t HiADCp;		// mask to switch the ADC port High-Pin
  uint8_t LoADCp;		// mask to switch the ADC port Low-Pin
  uint8_t HiADCm;		// mask to switch the ADC DDR port High-Pin
  uint8_t LoADCm;		// mask to switch the ADC DDR port Low-Pin
  uint8_t PinMSK;
  uint8_t ii;			// temporary variable
  unsigned int tmp16;		// temporary variable

  #if FLASHEND > 0x1fff
    int udiff;
  #endif

  unsigned long c_hfe;	// amplification factor for common Collector (Emitter follower)
  struct resis_t *thisR;
  unsigned long lrx1;
  unsigned long lirx1;
  unsigned long lirx2;

  /*
    switch HighPin directls to VCC 
    switch R_L port for LowPin to GND 
    TristatePin remains switched to input , no action required 
  */

  wdt_reset();
  
  LoPinRL = pgm_read_byte(&PinRLtab[LowPin]);		// instruction for LowPin R_L
  LoPinRH = LoPinRL + LoPinRL;				// instruction for LowPin R_H
  TriPinRL = pgm_read_byte(&PinRLtab[TristatePin]);	// instruction for TristatePin R_L
  TriPinRH = TriPinRL + TriPinRL;			// instruction for TristatePin R_H
  HiPinRL = pgm_read_byte(&PinRLtab[HighPin]);		// instruction for HighPin R_L
  HiPinRH = HiPinRL + HiPinRL;				// instruction for HighPin R_H

  HiADCp = pgm_read_byte(&PinADCtab[HighPin]);		// instruction for ADC High-Pin 
  LoADCp = pgm_read_byte(&PinADCtab[LowPin]);		// instruction for ADC Low-Pin
  HiADCm = HiADCp | TXD_MSK;
  HiADCp |= TXD_VAL;
  LoADCm = LoADCp | TXD_MSK;
  LoADCp |= TXD_VAL;

  // setting of Pins 
  R_PORT = 0;				// resistor-Port outputs to 0
  R_DDR = LoPinRL;			// Low-Pin to output and across R_L to GND
  ADC_DDR = HiADCm;			// High-Pin to output
  ADC_PORT = HiADCp;			// High-Pin fix to Vcc

  // for some MOSFET the gate (TristatePin) must be discharged
  ChargePin10ms(TriPinRL,0);		// discharge for N-Kanal
  adc.lp_otr = W5msReadADC(LowPin);	// read voltage of Low-Pin 
  if (adc.lp_otr >= 977) {		// no current now? 
    ChargePin10ms(TriPinRL,1);	 	// else: discharge for P-channel (Gate to VCC)
    adc.lp_otr = ReadADC(LowPin);	// read voltage of Low-Pin again
  }

  //if(adc.lp_otr > 92) {  // there is some current without TristatePin current 
  if(adc.lp_otr > 455) {   // there is more than 650uA current without TristatePin current 
    // Test if N-JFET or if self-conducting N-MOSFET
    R_DDR = LoPinRL | TriPinRH;		// switch R_H for Tristate-Pin (probably Gate) to GND
    adc.lp1 = W20msReadADC(LowPin);	// measure voltage at the assumed Source 
    adc.tp1 = ReadADC(TristatePin);	// measure Gate voltage
    R_PORT = TriPinRH;			// switch R_H for Tristate-Pin (probably Gate) to VCC
    adc.lp2 = W20msReadADC(LowPin);	// measure voltage at the assumed Source again

    // If it is a self-conducting MOSFET or JFET, then must be: adc.lp2 > adc.lp1 
    if (adc.lp2>(adc.lp1+488)) {
      if (PartFound != PART_FET) {
        // measure voltage at the  Gate, differ between MOSFET and JFET
        ADC_PORT = TXD_VAL;
        ADC_DDR = LoADCm;		// Low-Pin fix to GND
        R_DDR = TriPinRH | HiPinRL;	// High-Pin to output
        R_PORT = TriPinRH | HiPinRL;	// switch R_L for High-Pin to VCC
        adc.lp2 = W20msReadADC(TristatePin); // read voltage of assumed Gate 

        if (adc.lp2>3911) {  		// MOSFET
          PartFound = PART_FET;		// N-Kanal-MOSFET
          PartMode = PART_MODE_N_D_MOS; // Depletion-MOSFET
        } else {  // JFET (pn-passage between Gate and Source is conducting )
          PartFound = PART_FET;		// N-Kanal-JFET
          PartMode = PART_MODE_N_JFET;
        }

        //if ((PartReady == 0) || (adc.lp1 > trans.uBE[0])) 
        // there is no way to find out the right Source / Drain
        trans.uBE[0] = adc.lp1;
        gthvoltage = adc.lp1 - adc.tp1;	// voltage GS (Source - Gate)
        trans.uBE[1] = (unsigned int)(((unsigned long)adc.lp1 * 1000) / RR680MI);  // Id 0.01mA
        trans.b = TristatePin;		// save Pin numbers found for this FET
        trans.c = HighPin;
        trans.e = LowPin;
      }
    }

    ADC_PORT = TXD_VAL;		// direct outputs to GND

    // Test, if P-JFET or if self-conducting P-MOSFET
    ADC_DDR = LoADCm;			// switch Low-Pin (assumed Drain) direct to GND,
					// R_H for Tristate-Pin (assumed Gate) is already switched to VCC
    R_DDR = TriPinRH | HiPinRL;		// High-Pin to output
    R_PORT = TriPinRH | HiPinRL;	// High-Pin across R_L to Vcc
    adc.hp1 = W20msReadADC(HighPin);	// measure voltage at assumed Source 
    adc.tp1 = ReadADC(TristatePin);	// measure Gate voltage
    R_PORT = HiPinRL;			// switch R_H for Tristate-Pin (assumed Gate) to GND
    adc.hp2 = W20msReadADC(HighPin);	// read voltage at assumed Source again

    // if it is a self-conducting P_MOSFET or P-JFET , then must be:  adc.hp1 > adc.hp2 
    if (adc.hp1>(adc.hp2+488)) {
      if (PartFound != PART_FET) {
        // read voltage at the Gate , to differ between MOSFET and JFET
        ADC_PORT = HiADCp;		// switch High-Pin directly to VCC
        ADC_DDR = HiADCm;		// switch High-Pin to output
        adc.tp2 = W20msReadADC(TristatePin); //read voltage at the assumed Gate 

        if (adc.tp2<977) { 		// MOSFET
          PartFound = PART_FET;		// P-Kanal-MOSFET
          PartMode = PART_MODE_P_D_MOS; // Depletion-MOSFET
        } else { 			// JFET (pn-passage between Gate and Source is conducting)
          PartFound = PART_FET;		// P-Kanal-JFET
          PartMode = PART_MODE_P_JFET;
        }

        gthvoltage = adc.tp1 - adc.hp1;		// voltage GS (Gate - Source)
        trans.uBE[1] = (unsigned int)(((unsigned long)(ADCconfig.U_AVCC - adc.hp1) * 1000) / RR680PL); // Id 0.01mA
        trans.b = TristatePin;			// save Pin numbers found for this FET
        trans.c = LowPin;
        trans.e = HighPin;
      }
    }
  }  // end component has current without TristatePin signal

  // Test circuit with common collector (Emitter follower) PNP
  ADC_PORT = TXD_VAL;
  ADC_DDR = LoADCm;			// Collector direct to GND
  R_PORT = HiPinRL;			// switch R_L port for HighPin (Emitter) to VCC
  R_DDR = TriPinRL | HiPinRL;		// Base resistor  R_L to GND
  adc.hp1 = ADCconfig.U_AVCC - W5msReadADC(HighPin);	// voltage at the Emitter resistor
  adc.tp1 = ReadADC(TristatePin);	// voltage at the base resistor

  if (adc.tp1 < 10) {
    R_DDR =  TriPinRH | HiPinRL;	// Tripin=RH-
    adc.hp1 = ADCconfig.U_AVCC - W5msReadADC(HighPin);
    adc.tp1 = ReadADC(TristatePin);	// voltage at base resistor 

    c_hfe = ((unsigned long)adc.hp1 * (unsigned long)(((unsigned long)R_H_VAL * 100) / (unsigned int)RR680PL)) / (unsigned int)adc.tp1;	
  } else c_hfe = (unsigned long)((adc.hp1 - adc.tp1) / adc.tp1);

  // set Pins again for circuit with common Emitter PNP
  R_DDR = LoPinRL;		// switch R_L port for Low-Pin to output (GND)
  R_PORT = 0;			// switch all resistor ports to GND
  ADC_DDR = HiADCm;		// switch High-Pin to output
  ADC_PORT = HiADCp;		// switch High-Pin to VCC
  wait_about5ms();
  
  if (adc.lp_otr < 977) {
    // if the component has no connection between  HighPin and LowPin;
    // Test to PNP
    R_DDR = LoPinRL | TriPinRL;		// switch R_L port for Tristate-Pin to output (GND), for Test of PNP
    adc.lp1 = W5msReadADC(LowPin);	// measure voltage at LowPin

    if (adc.lp1 > 3422) {
      // component has current => PNP-Transistor or equivalent
      // compute current amplification factor in both directions
      R_DDR = LoPinRL | TriPinRH;	// switch R_H port for Tristate-Pin (Base) to output (GND)

      adc.lp1 = W5msReadADC(LowPin);	// measure voltage at LowPin (assumed Collector)
      adc.tp2 = ReadADC(TristatePin);	// measure voltage at TristatePin (Base) 

      // check, if Test is done before 
      if ((PartFound == PART_TRANSISTOR) || (PartFound == PART_FET)) {
         PartReady = 1;
      }
      trans.uBE[PartReady] = ReadADC(HighPin) - adc.tp2;	// Base Emitter Voltage

      // compute current amplification factor for circuit with common Emitter
      // hFE = B = Collector current / Base current

      if(adc.tp2 < 53) adc.tp2 = 53;
      tmp16 = adc.lp1;

      if (tmp16 > adc.lp_otr) {
        tmp16 -= adc.lp_otr;
      }
        trans.hfe[PartReady] = ((unsigned int)tmp16 * (unsigned long)(((unsigned long)R_H_VAL * 100) / 
                                (unsigned int)RR680MI)) / (unsigned int)adc.tp2;	
      // current amplification factor for common  Collector (Emitter follower)
      // c_hFE = (Emitter current - Base current) / Base current
        if (c_hfe > trans.hfe[PartReady]) {
          trans.hfe[PartReady] = c_hfe;
          trans.uBE[PartReady] = ADCconfig.U_AVCC - adc.hp1 - adc.tp1;	// Base Emitter Voltage common collector
        }
 
      if (PartFound != PART_THYRISTOR) {
        if (adc.tp2 > 977) {
          // PNP-Transistor is found (Base voltage moves to VCC)
          PartFound = PART_TRANSISTOR;
          PartMode = PART_MODE_PNP;
        } else {
          if ((adc.lp_otr < 97) && (adc.lp1 > 2000)) {
            // is flow voltage low enough in the closed  state?
            // (since D-Mode-FET would be by mistake detected as E-Mode )
            PartFound = PART_FET;		// P-Kanal-MOSFET is found (Basis/Gate moves not to VCC)
            PartMode = PART_MODE_P_E_MOS;
    
            // measure the Gate threshold voltage
            // Switching of Drain is monitored with digital input
            // Low level is specified up to 0.3 * VCC
            // High level is specified above 0.6 * VCC
            PinMSK = LoADCm & 7;
            ADMUX = TristatePin | (1<<REFS0);	// switch to TristatePin, Ref. VCC
            gthvoltage = 1;			// round up ((1*4)/9)
    
            for(ii=0;ii<11;ii++) {
              wdt_reset();
              ChargePin10ms(TriPinRL,1);
              R_DDR = LoPinRL | TriPinRH;		// switch R_H for Tristate-Pin (Basis) to GND
    
              while (!(ADC_PIN&PinMSK));		// Wait, until the MOSFET switches and Drain moves to VCC
                      			// 1 is detected with more than 2.5V (up to 2.57V) with tests of mega168 and mega328
              R_DDR = LoPinRL;
              ADCSRA |= (1<<ADSC);		// Start Conversion
    
              while (ADCSRA&(1<<ADSC));		// wait

              gthvoltage += (1023 - ADCW);	// Add Tristatepin-Voltage
            }
    
            gthvoltage *= 4;		// is equal to 44*ADCW
            gthvoltage /= 9;		// gives resolution in mV
          }
        }

        trans.b = TristatePin;
        trans.c = LowPin;
        trans.e = HighPin;
      }  // end if PartFound != PART_THYRISTOR
    }  // end component has current => PNP

    #ifdef COMMON_COLLECTOR
      // Low-Pin=RL- HighPin=VCC
      R_DDR = LoPinRL | TriPinRL;
      R_PORT = TriPinRL;			// TriPin=RL+  NPN with common Collector
      adc.lp1 = W5msReadADC(LowPin);		// voltage at Emitter resistor
      adc.tp1 = ADCconfig.U_AVCC - ReadADC(TristatePin);	// voltage at Base resistor
    
      if (adc.tp1 < 10) {
        R_DDR = LoPinRL | TriPinRH;
        R_PORT = TriPinRH;			// Tripin=RH+
        adc.lp1 = W5msReadADC(LowPin);
        adc.tp1 = ADCconfig.U_AVCC - ReadADC(TristatePin);	// voltage at Base resistor

        c_hfe = ((unsigned long)adc.lp1 * (unsigned long)(((unsigned long)R_H_VAL * 100) / 
                (unsigned int)RR680MI)) / (unsigned int)adc.tp1;	
    
      } else c_hfe = (adc.lp1 - adc.tp1) / adc.tp1;
    
    #endif

    // Tristate (can be Base) to VCC, Test if NPN
    ADC_DDR = LoADCm;			// Low-Pin to output 0V
    ADC_PORT = TXD_VAL;			// switch Low-Pin to GND
    R_DDR = TriPinRL | HiPinRL;		// RL port for High-Pin and Tristate-Pin to output
    R_PORT = TriPinRL | HiPinRL;	// RL port for High-Pin and Tristate-Pin to Vcc
    adc.hp1 = W5msReadADC(HighPin);	// measure voltage at High-Pin  (Collector)
    
    if (adc.hp1 < 1600) {
      // component has current => NPN-Transistor or somthing else    
      if (PartReady==1) goto widmes;
    
      // Test auf Thyristor:
      // Gate discharge
      ChargePin10ms(TriPinRL,0);	// Tristate-Pin (Gate) across R_L 10ms to GND
      adc.hp3 = W5msReadADC(HighPin);	// read voltage at High-Pin (probably Anode) again
                                        // current should still flow, if not,
                                        // no Thyristor or holding current to low 
    
      R_PORT = 0;			// switch R_L for High-Pin (probably Anode) to GND (turn off)
      wait_about5ms();
      R_PORT = HiPinRL;			// switch R_L for High-Pin (probably Anode) again to VCC
      adc.hp2 = W5msReadADC(HighPin);	// measure voltage at the High-Pin (probably Anode) again
    
      if ((adc.hp3 < 1600) && (adc.hp2 > 4400)) {
        // if the holding current was switched off the thyristor must be switched off too. 
        // if Thyristor was still swiched on, if gate was switched off => Thyristor
        PartFound = PART_THYRISTOR;
    
        // Test if Triac
        R_DDR = 0;
        R_PORT = 0;
        ADC_PORT = LoADCp;		// Low-Pin fix to VCC
        wait_about5ms();
    
        R_DDR = HiPinRL;		// switch R_L port HighPin to output (GND)
        if(W5msReadADC(HighPin) > 244) goto savenresult;		// measure voltage at the  High-Pin (probably A2); if too high:
        R_DDR = HiPinRL | TriPinRL;	// switch R_L port for TristatePin (Gate) to output (GND) => Triac should be triggered 
        if(W5msReadADC(TristatePin) < 977) goto savenresult; 		// measure voltage at the Tristate-Pin (probably Gate) ;
        if(ReadADC(HighPin) < 733) goto savenresult; 		// component has no current => no Triac => abort
        R_DDR = HiPinRL;		// TristatePin (Gate) to input 
        if(W5msReadADC(HighPin) < 733) goto savenresult; 		// component has no current without base current => no Triac => abort
        R_PORT = HiPinRL;		// switch R_L port for HighPin to VCC => switch off holding current 
        wait_about5ms();
        R_PORT = 0;			// switch R_L port for HighPin again to GND; Triac should now switched off
        if(W5msReadADC(HighPin) > 244) goto savenresult;		// measure voltage at the High-Pin (probably A2) ;
        PartFound = PART_TRIAC;
        PartReady = 1;
        goto savenresult;
      }
    
      // Test if NPN Transistor or MOSFET
      //ADC_DDR = LoADCm;		// Low-Pin to output 0V
      R_DDR = HiPinRL | TriPinRH;	// R_H port of Tristate-Pin (Basis) to output
      R_PORT = HiPinRL | TriPinRH;	// R_H port of Tristate-Pin (Basis) to VCC
      wait_about50ms();
      adc.hp2 = ADCconfig.U_AVCC - ReadADC(HighPin);		// measure the voltage at the collector resistor 
      adc.tp2 = ADCconfig.U_AVCC - ReadADC(TristatePin);	// measure the voltage at the base resistor 

      if((PartFound == PART_TRANSISTOR) || (PartFound == PART_FET)) {
        PartReady = 1;		// check, if test is already done once
      }
    
      #ifdef COMMON_EMITTER
        trans.uBE[PartReady] = ADCconfig.U_AVCC - adc.tp2 - ReadADC(LowPin);
    
        // compute current amplification factor for common Emitter
        // hFE = B = Collector current / Base current
        if (adc.tp2 < 53) adc.tp2 = 53;
    
        tmp16 = adc.hp2;
        if (tmp16 > adc.lp_otr) tmp16 -= adc.lp_otr;
      trans.hfe[PartReady] = ((tmp16 / ((RR680PL+500)/1000)) * (R_H_VAL/500)) / (adc.tp2/500);
      #endif
    
      #ifdef COMMON_COLLECTOR
        // compare current amplification factor for common Collector (Emitter follower)
        // hFE = (Emitterstrom - Basisstrom) / Basisstrom
    
        #ifdef COMMON_EMITTER
          if (c_hfe >  trans.hfe[PartReady]) {
        #endif
            trans.hfe[PartReady] = c_hfe;
            trans.uBE[PartReady] = ADCconfig.U_AVCC - adc.lp1 - adc.tp1;
        #ifdef COMMON_EMITTER
          }
        #endif
      #endif
    
      if(adc.tp2 > 2557) {		// Basis-voltage R_H is low enough
        PartFound = PART_TRANSISTOR;	// NPN-Transistor is found (Base is near GND)
        PartMode = PART_MODE_NPN;
      } else {				// Basis has low current
        if((adc.lp_otr < 97) && (adc.hp2 > 3400)) {
          // if flow voltage in switched off mode low enough?
          // (since D-Mode-FET will be detected in error as E-Mode )
          PartFound = PART_FET;		// N-Kanal-MOSFET is found (Basis/Gate will Not be pulled down)
          PartMode = PART_MODE_N_E_MOS;
    
          // Switching of Drain is monitored with digital input
          // Low level is specified up to 0.3 * VCC
          // High level is specified above 0.6 * VCC
          PinMSK = HiADCm & 7;
    
          // measure Threshold voltage of Gate
          ADMUX = TristatePin | (1<<REFS0);	// measure TristatePin, Ref. VCC
          gthvoltage = 1;			// round up ((1*4)/9)
    
          for(ii=0;ii<11;ii++) {
            wdt_reset();
            ChargePin10ms(TriPinRL,0);	// discharge Gate 10ms with RL 
            R_DDR = HiPinRL | TriPinRH;	// slowly charge Gate 
            R_PORT = HiPinRL | TriPinRH;
            while ((ADC_PIN&PinMSK));	// Wait, until the MOSFET switch and Drain moved to low 
                                        // 0 is detected with input voltage of 2.12V to 2.24V (tested with mega168 & mega328)
            R_DDR = HiPinRL;		// switch off current
            ADCSRA |= (1<<ADSC);		// start ADC conversion
            while (ADCSRA&(1<<ADSC));	// wait until ADC finished
            gthvoltage += ADCW;		// add result of ADC
          }
    
          gthvoltage *= 4;	// is equal to 44 * ADCW
          gthvoltage /= 9;	// scale to mV
        }
      }

savenresult:
      trans.b = TristatePin;	// save Pin-constellation
      trans.c = HighPin;
      trans.e = LowPin;
    }  // end component conduct => npn

    ADC_DDR = TXD_MSK;		// switch all ADC-Ports to input
    ADC_PORT = TXD_VAL;		// switch all ADC-Ports to 0 (no Pull up)

    // Finish
    // end component has no connection between HighPin and LowPin
    goto widmes;
  }

  // component has current
  // Test if Diode
  ADC_PORT = TXD_VAL;

  for (ii=0;ii<200;ii++) {
    ADC_DDR = LoADCm | HiADCm;   	// discharge by short of Low and High side
    wait_about5ms();             	// Low and Highpin to GND for discharge
    ADC_DDR = LoADCm;            	// switch only Low-Pin fix to GND
    adc.hp1 = ReadADC(HighPin);  	// read voltage at High-Pin
    if (adc.hp1 < (150/8)) break;
  }

  /*
    It is possible, that wrong Parts are detected without discharging, because
    the gate of a MOSFET can be charged.
    The additional measurement with the big resistor R_H is made, to differ antiparallel diodes
    from resistors.
    A diode has a voltage, that is nearly independent from the current.
    The voltage of a resistor is proportional to the current.
  */
  // check first with low current (R_H=470k)
  // With this method the diode can be better differed from a capacitor,
  // but a parallel to a capacitor mounted diode can not be found.
  R_DDR = HiPinRH;			// switch R_H port for High-Pin output (VCC)
  R_PORT = HiPinRH;
  ChargePin10ms(TriPinRL,1);		// discharge of P-Kanal-MOSFET gate
  adc.hp2 = W5msReadADC(HighPin); 	// M--|<--HP--R_H--VCC
  ChargePin10ms(TriPinRL,0);		// discharge for N-Kanal-MOSFET gate
  adc.hp3 = W5msReadADC(HighPin);	// M--|<--HP--R_H--VCC
  
  // check with higher current (R_L=680)
  R_DDR = HiPinRL;			// switch R_L port for High-Pin to output (VCC)
  R_PORT = HiPinRL;
  adc.hp1 = W5msReadADC(HighPin) - ReadADC(LowPin);
  ChargePin10ms(TriPinRL,1);		// discharge for N-Kanal-MOSFET gate
  adc.lp_otr = W5msReadADC(HighPin) - ReadADC(LowPin);
  
  R_DDR = HiPinRH;			// switch R_H port for High-Pin output (VCC)
  R_PORT = HiPinRH;

  if(adc.lp_otr > adc.hp1) {
    adc.hp1 = adc.lp_otr;		// the higher value wins
    adc.hp3 = adc.hp2;
  } else {
    ChargePin10ms(TriPinRL,0);	// discharge for N-Kanal-MOSFET gate
  }

  adc.hp2 = W5msReadADC(HighPin); 	// M--|<--HP--R_H--VCC

  //if((adc.hp1 > 150) && (adc.hp1 < 4640) && (adc.hp1 > (adc.hp3+(adc.hp3/8))) && (adc.hp3*8 > adc.hp1)) {
  if((adc.hp1 > 150) && (adc.hp1 < 4640) && (adc.hp2 < adc.hp1) && (adc.hp1 > (adc.hp3+(adc.hp3/8))) && (adc.hp3*16 > adc.hp1)) {
    // voltage is above 0,15V and below 4,64V => Ok
    if((PartFound == PART_NONE) || (PartFound == PART_RESISTOR)) {
      PartFound = PART_DIODE;	// mark for diode only, if no other component is found
                                // since there is a problem with Transistors with a protection diode
    }

    diodes[NumOfDiodes].Anode = HighPin;
    diodes[NumOfDiodes].Cathode = LowPin;
    diodes[NumOfDiodes].Voltage = adc.hp1;	// voltage in Millivolt 
    NumOfDiodes++;
  }  // end voltage is above 0,15V and below 4,64V 

widmes:
  if (NumOfDiodes > 0) goto clean_ports;
  // resistor measurement
  wdt_reset();

  // U_SCALE can be set to 4 for better resolution of ReadADC result
  #if U_SCALE != 1
    ADCconfig.U_AVCC *= U_SCALE;	// scale to higher resolution, mV scale is not required
    ADCconfig.U_Bandgap *= U_SCALE;
  #endif

  #if R_ANZ_MESS != ANZ_MESS
    ADCconfig.Samples = R_ANZ_MESS;	// switch to special number of repetitions
  #endif

  #define MAX_REPEAT (700 / (5 + R_ANZ_MESS/8))

  ADC_PORT = TXD_VAL;
  ADC_DDR = LoADCm;		// switch Low-Pin to output (GND)
  R_DDR = HiPinRL;		// switch R_L port for High-Pin to output (VCC)
  R_PORT = HiPinRL;	

  #if FLASHEND > 0x1fff
    adc.hp2 = 0;

    for (ii=1;ii<MAX_REPEAT;ii++) {
      // wait until voltage is stable
      adc.tp1 = W5msReadADC(LowPin);		// low-voltage at Rx with load
      adc.hp1 = ReadADC(HighPin);		// voltage at resistor Rx with R_L
      udiff = adc.hp1 - adc.hp2;
      if (udiff < 0) udiff = -udiff;
      if (udiff < 3) break;
      adc.hp2 = adc.hp1;
      wdt_reset();
    }

    if (ii == MAX_REPEAT) goto testend;
  #else
    adc.tp1 = W5msReadADC(LowPin);	// low-voltage at Rx with load
    adc.hp1 = ReadADC(HighPin);		// voltage at resistor Rx with R_L
  #endif

  if (adc.tp1 > adc.hp1) adc.tp1 = adc.hp1;

  R_PORT = 0;
  R_DDR = HiPinRH;			// switch R_H port for High-Pin to output (GND)
  adc.hp2 = W5msReadADC(HighPin);	// read voltage, should be down

  if (adc.hp2 > (20*U_SCALE)) goto testend; // if resistor, voltage should be down

  R_PORT = HiPinRH;			// switch R_H for High-Pin to VCC
  adc.hp2 = W5msReadADC(HighPin);	// voltage at resistor Rx with R_H

  ADC_DDR = HiADCm;			// switch High-Pin to output
  ADC_PORT = HiADCp;			// switch High-Pin to VCC
  R_PORT = 0;
  R_DDR = LoPinRL;			// switch R_L for Low-Pin to GND

  adc.lp2 = 0;

  for (ii=1;ii<MAX_REPEAT;ii++) {
    // wait until voltage is stable
    adc.tp2 = W5msReadADC(HighPin);	// high voltage with load
    adc.lp1 = ReadADC(LowPin);	// voltage at the other end of Rx
    udiff = adc.lp1 - adc.lp2;
    if (udiff < 0) udiff = -udiff;
    if (udiff < 3) break;
    adc.lp2 = adc.lp1;
    wdt_reset();
  }

  if (ii == MAX_REPEAT) goto testend;

  if (adc.tp2 < adc.lp1) adc.tp2 = adc.lp1;

  R_DDR = LoPinRH;			// switch R_H for Low-Pin to GND
  adc.lp2 = W5msReadADC(LowPin);

  if((adc.hp1 < (4400*U_SCALE)) && (adc.hp2 > (97*U_SCALE))) goto testend; //voltage break down isn't insufficient  

  //if ((adc.hp2 + (adc.hp2 / 61)) < adc.hp1)
  if (adc.hp2 < (4972*U_SCALE)) { 
    // voltage breaks down with low test current and it is not nearly shorted  => resistor
    //if (adc.lp1 < 120) { 		// take measurement with R_H 
    if (adc.lp1 < (169*U_SCALE)) {  	// take measurement with R_H 
      ii = 'H';
      if (adc.lp2 < (38*U_SCALE)) goto testend; // measurement > 60MOhm to big resistance
      // two measurements with R_H resistors (470k) are made:
      // lirx1 (measurement at HighPin)
      lirx1 = (unsigned long)((unsigned int)R_H_VAL) * (unsigned long)adc.hp2 / (ADCconfig.U_AVCC - adc.hp2);
      // lirx2 (measurement at LowPin)
      lirx2 = (unsigned long)((unsigned int)R_H_VAL) * (unsigned long)(ADCconfig.U_AVCC - adc.lp2) / adc.lp2;

      #define U_INT_LIMIT (990*U_SCALE)		// 1V switch limit in ReadADC for atmega family
      #define FAKT_LOW 4		// resolution is about four times better

      if (adc.hp2 < U_INT_LIMIT) lrx1 = (lirx1*FAKT_LOW + lirx2) / (FAKT_LOW+1);	// weighted average of both R_H measurements
      else if (adc.lp2 < U_INT_LIMIT)lrx1 = (lirx2*FAKT_LOW + lirx1) / (FAKT_LOW+1);	// weighted average of both R_H measurements
      else lrx1 = (lirx1 + lirx2) / 2;	// average of both R_H measurements

      lrx1 *= 100;
      lrx1 += RH_OFFSET;		// add constant for correction of systematic error

    } else {
      ii = 'L';
      // two measurements with R_L resistors (680) are made:
      // lirx1 (measurement at HighPin)

      if (adc.tp1 > adc.hp1) adc.hp1 = adc.tp1;		// diff negativ is illegal
      lirx1 =(unsigned long)RR680PL * (unsigned long)(adc.hp1 - adc.tp1) / (ADCconfig.U_AVCC - adc.hp1);
      if (adc.tp2 < adc.lp1) adc.lp1 = adc.tp2;		// diff negativ is illegal

      // lirx2 (Measurement at LowPin)
      lirx2 =(unsigned long)RR680MI * (unsigned long)(adc.tp2 -adc.lp1) / adc.lp1;
      //lrx1 =(unsigned long)R_L_VAL * (unsigned long)adc.hp1 / (adc.hp3 - adc.hp1);

      #ifdef AUTOSCALE_ADC
        if (adc.hp1 < U_INT_LIMIT) {
          lrx1 = (lirx1*FAKT_LOW + lirx2) / (FAKT_LOW+1);	// weighted average of both R_L measurements
        } else if (adc.lp1 < U_INT_LIMIT) {
          lrx1 = (lirx2*FAKT_LOW + lirx1) / (FAKT_LOW+1);	// weighted average of both R_L measurements
        } else
      #endif
        {
          lrx1 = (lirx1 + lirx2) / 2;	// average of both R_L measurements
        }
    }
    // lrx1  is tempory result

    if((PartFound == PART_DIODE) || (PartFound == PART_NONE) || (PartFound == PART_RESISTOR)) {
      for (ii=0; ii<ResistorsFound; ii++) {
        // search measurements with inverse polarity 
        thisR = &resis[ii];
        if (thisR->rt != TristatePin) continue;

        // must be measurement with inverse polarity 
        // resolution is 0.1 Ohm, 1 Ohm = 10 !
        lirx1 = (labs((long)lrx1 - (long)thisR->rx) * 10) / (lrx1 + thisR->rx + 100);

        if (lirx1  > 0) goto testend;  // <10% mismatch

        PartFound = PART_RESISTOR;
        goto testend;
      }  // end for

      // no same resistor with the same Tristate-Pin found, new one
      thisR = &resis[ResistorsFound];	// pointer to a free resistor structure
      thisR->rx = lrx1;			// save resistor value

      #if FLASHEND > 0x1fff
        thisR->lx = 0;			// no inductance
      #endif

        thisR->ra = LowPin;		// save Pin numbers
        thisR->rb = HighPin;
        thisR->rt = TristatePin;	// Tristate is saved for easier search of inverse measurement
        ResistorsFound++;		// 1 more resistor found
    }
  }

testend:

  #if U_SCALE != 1
    ADCconfig.U_AVCC /= U_SCALE;	// scale back to mV resolution
    ADCconfig.U_Bandgap /= U_SCALE;
  #endif

  #if R_ANZ_MESS != ANZ_MESS
    ADCconfig.Samples = ANZ_MESS;	// switch back to standard number of repetition
  #endif

  #ifdef DebugOut
    #if DebugOut < 10
      wait_about2s();
    #endif
  #endif

clean_ports:

  ADC_DDR = TXD_MSK;		// all ADC-Pins Input
  ADC_PORT = TXD_VAL;		// all ADC outputs to Ground, keine Pull up
  R_DDR = 0;			// all resistor-outputs to Input
  R_PORT = 0;			// all resistor-outputs to Ground, no Pull up
}  // end CheckPins()

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

// Get residual current in reverse direction of a diode
//=================================================================
void GetIr(uint8_t hipin, uint8_t lopin) {
  unsigned int u_res;			// reverse voltage at 470k
  unsigned int ir_nano;
  //unsigned int ir_micro;
  uint8_t LoPinR_L;
  uint8_t HiADC;

  HiADC = pgm_read_byte(&PinADCtab[hipin]);
  ADC_PORT = HiADC | TXD_VAL;			// switch ADC port to high level
  ADC_DDR = HiADC | TXD_MSK;			// switch High Pin direct to VCC
  LoPinR_L = pgm_read_byte(&PinRLtab[lopin]);  	// R_L mask for LowPin R_L load
  R_PORT = 0;					// switch R-Port to GND
  R_DDR = LoPinR_L + LoPinR_L;			// switch R_H port for LowPin to output (GND)

  u_res = W5msReadADC(lopin);		// read voltage
  if (u_res == 0) return;		// no Output, if no current in reverse direction

  lcd_line4();

  lcd_fix_string(Ir_str);		// output text "  Ir="

  // R_H_VAL has units of 10 Ohm, u_res has units of mV, ir_nano has units of nA
  ir_nano = (unsigned long)(u_res * 100000UL) / R_H_VAL;
  DisplayValue(ir_nano,-9,'A',2);	// output two digits of current with nA units

  ADC_DDR = TXD_MSK;			// switch off
  ADC_PORT = TXD_VAL;			// switch off
  R_DDR = 0;				// switch off current

  return;
}

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

#ifdef INHIBIT_SLEEP_MODE
  //#define StartADCwait() ADCSRA = (1<<ADSC) | (1<<ADEN) | (1<<ADIF) | AUTO_CLOCK_DIV; /* enable ADC and start */
  #define StartADCwait() ADCSRA = StartADCmsk; /* Start conversion */\
  while (ADCSRA & (1 << ADSC))  /* wait until conversion is done */
#else
  #define StartADCwait() ADCSRA = (1<<ADEN) | (1<<ADIF) | (1<<ADIE) | AUTO_CLOCK_DIV; /* enable ADC and Interrupt */\
  set_sleep_mode(SLEEP_MODE_ADC);\
  sleep_mode();	/* Start ADC, return, if ADC has finished */
#endif

unsigned int ReadADC (uint8_t Probe) {
  unsigned int U; 		// return value (mV)
  uint8_t Samples; 		// loop counter
  unsigned long Value; 		// ADC value
  Probe |= (1 << REFS0); 	// use internal reference anyway

sample:
  ADMUX = Probe; 	// set input channel and U reference

    // if voltage reference changes, wait for voltage stabilization
    if ((Probe & (1 << REFS1)) != 0) wait100us(); 		// time for voltage stabilization
  // always do one dummy read of ADC, 112us

  StartADCwait();		// start ADC and wait

  // sample ADC readings
  Value = 0UL; 			// reset sampling variable
  Samples = 0; 			// number of samples to take

  while (Samples < ADCconfig.Samples) {		// take samples
    StartADCwait();				// start ADC and wait
    Value += ADCW; 				// add ADC reading

    // auto-switch voltage reference for low readings
    if ((Samples == 4) && (ADCconfig.U_Bandgap > 255) && ((uint16_t)Value < 1024) && !(Probe & (1 << REFS1))) {
      Probe |= (1 << REFS1); 		// select internal bandgap reference
      goto sample; 	// re-run sampling
    }

    Samples++; 		// one more done
  }

  // convert ADC reading to voltage - single sample: U = ADC reading * U_ref / 1024
  // get voltage of reference used
  if (Probe & (1 << REFS1)) U = ADCconfig.U_Bandgap; 	// bandgap reference
  else U = ADCconfig.U_AVCC; 	// Vcc reference

  // convert to voltage
  Value *= U; 		// ADC readings * U_ref
  Value /= 1023; 	// / 1024 for 10bit ADC

  // de-sample to get average voltage
  Value /= ADCconfig.Samples;
  U = (unsigned int)Value;
  return U;
  //return ((unsigned int)(Value / (1023 * (unsigned long)ADCconfig.Samples)));
}

unsigned int W5msReadADC (uint8_t Probe) {
  wait_about5ms();
  return (ReadADC(Probe));
}

unsigned int W10msReadADC (uint8_t Probe) {
  wait_about10ms();
  return (ReadADC(Probe));
}

unsigned int W20msReadADC (uint8_t Probe) {
  wait_about20ms();
  return (ReadADC(Probe));
}

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

// new code by K.-H. Kubbeler
// ReadCapacity tries to find the value of a capacitor by measuring the load time.
// first of all the capacitor is discharged.
// Then a series of up to 500 load pulses with 10ms duration each is done across the R_L (680Ohm)
// resistor.
// After each load pulse the voltage of the capacitor is measured without any load current.
// If voltage reaches a value of more than 300mV and is below 1.3V, the capacity can be
// computed from load time and voltage by a interpolating a build in table.
// If the voltage reaches a value of more than 1.3V with only one load pulse,
// another measurement methode is used:
// The build in 16bit counter can save the counter value at external events.
// One of these events can be the output change of a build in comparator.
// The comparator can compare the voltage of any of the ADC input pins with the voltage
// of the internal reference (1.3V or 1.1V).
// After setting up the comparator and counter properly, the load of capacitor is started 
// with connecting the positive pin with the R_H resistor (470kOhm) to VCC and immediately
// the counter is started. By counting the overflow Events of the 16bit counter and watching
// the counter event flag  the total load time of the capacitor until reaching the internal
// reference voltage can be measured.
// If any of the tries to measure the load time is successful,
// the following variables are set:
// cap.cval = value of the capacitor 
// cap.cval_uncorrected = value of the capacitor uncorrected
// cap.esr = serial resistance of capacitor,  0.01 Ohm units
// cap.cpre = units of cap.cval (-12==pF, -9=nF, -6=uF)
// ca   = Pin number (0-2) of the LowPin
// cb   = Pin number (0-2) of the HighPin

//=================================================================
void ReadCapacity(uint8_t HighPin, uint8_t LowPin) {
  // check if capacitor and measure the capacity value
  unsigned int tmpint;
  unsigned int adcv[4];

  #ifdef INHIBIT_SLEEP_MODE
    unsigned int ovcnt16;
  #endif

  uint8_t HiPinR_L, HiPinR_H;
  uint8_t LoADC;
  uint8_t ii;

  #if FLASHEND > 0x1fff
    unsigned int vloss;		// lost voltage after load pulse in 0.1% 
  #endif

  LoADC = pgm_read_byte(&PinADCtab[LowPin]) | TXD_MSK;
  HiPinR_L = pgm_read_byte(&PinRLtab[HighPin]);		// R_L mask for HighPin R_L load
  HiPinR_H = HiPinR_L + HiPinR_L;			// double for HighPin R_H load

  if(PartFound == PART_RESISTOR) return;	// We have found a resistor already 
  for (ii=0;ii<NumOfDiodes;ii++) if ((diodes[ii].Cathode == LowPin) && (diodes[ii].Anode == HighPin) && (diodes[ii].Voltage < 1500)) return;
  
  #if FLASHEND > 0x1fff
    cap.esr = 0;			// set ESR of capacitor to zero
    vloss = 0;				// set lost voltage to zero
  #endif

  cap.cval = 0;				// set capacity value to zero
  cap.cpre = -12;			// default unit is pF
  EntladePins();			// discharge capacitor

  ADC_PORT = TXD_VAL;			// switch ADC-Port to GND
  R_PORT = 0;				// switch R-Port to GND
  ADC_DDR = LoADC;			// switch Low-Pin to output (GND)
  R_DDR = HiPinR_L;			// switch R_L port for HighPin to output (GND)
  adcv[0] = ReadADC(HighPin);		// voltage before any load 

  // ******** should adcv[0] be measured without current???
  adcv[2] = adcv[0];			// preset to prevent compiler warning

  for (ovcnt16=0; ovcnt16<500; ovcnt16++) {
    R_PORT = HiPinR_L;			// R_L to 1 (VCC) 
    R_DDR = HiPinR_L;			// switch Pin to output, across R to GND or VCC
    wait10ms();				// wait exactly 10ms, do not sleep

    R_DDR = 0;				// switch back to input
    R_PORT = 0;				// no Pull up
    wait500us();			// wait a little time

    wdt_reset();

    // read voltage without current, is already charged enough?
    adcv[2] = ReadADC(HighPin);

    if (adcv[2] > adcv[0]) adcv[2] -= adcv[0];		// difference to beginning voltage
    else adcv[2] = 0;			// voltage is lower or same as beginning voltage

    if ((ovcnt16 == 126) && (adcv[2] < 75)) break;		// don't try to load any more
    if (adcv[2] > 300) break;		// probably 100mF can be charged well-timed 
  }

  if (adcv[2] < 301) goto keinC;		// was never charged enough, >100mF or shorted

  // voltage is rised properly and keeps the voltage enough
  if ((ovcnt16 == 0 ) && (adcv[2] > 1300)) {
    goto messe_mit_rh;		// Voltage of more than 1300mV is reached in one pulse, too fast loaded
  }

  // Capacity is more than about 50uF
  // wait the half the time which was required for loading
  adcv[3] = adcv[2];			// preset to prevent compiler warning

  for (tmpint=0; tmpint<=ovcnt16; tmpint++) {
    wait5ms();
    adcv[3] = ReadADC(HighPin);	// read voltage again, is discharged only a little bit ?
    wdt_reset();
  }

  if (adcv[3] > adcv[0]) adcv[3] -= adcv[0];		// difference to beginning voltage
  else adcv[3] = 0;			// voltage is lower or same as beginning voltage

  if (adcv[2] > adcv[3]) adcv[3] = adcv[2] - adcv[3];	// lost voltage during load time wait
  else adcv[3] = 0;			// no lost voltage
  #if FLASHEND > 0x1fff
    // compute equivalent parallel resistance from voltage drop
    if (adcv[3] > 0) vloss = (unsigned long)(adcv[3] * 1000UL) / adcv[2]; // there is any voltage drop (adcv[3])! adcv[2] is the loaded voltage.
  #endif

  if (adcv[3] > 100) {
    // more than 100mV is lost during load time
    if (ovcnt16 == 0 ) goto messe_mit_rh;		// Voltage of more than 1300mV is reached in one pulse, but not hold
    goto keinC;			// capacitor does not keep the voltage about 5ms
  }

  cap.cval_uncorrected.dw = ovcnt16 + 1;
  // compute factor with load voltage + lost voltage during the voltage load time
  cap.cval_uncorrected.dw *= getRLmultip(adcv[2]+adcv[3]);	// get factor to convert time to capacity from table

  cap.cval = cap.cval_uncorrected.dw;	// set result to uncorrected
  cap.cpre = -9;			// switch units to nF 
  Scale_C_with_vcc();

  // cap.cval for this type is at least 40000nF, so the last digit will be never shown
  cap.cval *= (1000);	// correct with C_H_KORR with 0.1% resolution, but prevent overflow
  cap.cval /= 100;

  goto checkDiodes;

//==================================================================================
// Measurement of little capacity values
messe_mit_rh:
  // little capacity value, about  < 50 uF
  EntladePins();			// discharge capacitor

  // measure with the R_H (470kOhm) resistor 
  R_PORT = 0;		// R_DDR ist HiPinR_L
  ADC_DDR = (1<<TP1) | (1<<TP2) | (1<<TP3) | (1<<TxD);	// switch all Pins to output
  ADC_PORT = TXD_VAL;		// switch all ADC Pins to GND
  R_DDR = HiPinR_H;   		// switch R_H resistor port for HighPin to output (GND)

  // setup Analog Comparator
  ADC_COMP_CONTROL = (1<<ACME);			// enable Analog Comparator Multiplexer
  ACSR =  (1<<ACBG) | (1<<ACI)  | (1<<ACIC);	// enable, 1.3V, no Interrupt, Connect to Timer1 
  ADMUX = (1<<REFS0) | HighPin;			// switch Mux to High-Pin
  ADCSRA = (1<<ADIF) | AUTO_CLOCK_DIV; 		// disable ADC
  wait200us();					// wait for bandgap to start up

  // setup Counter1
  ovcnt16 = 0;
  TCCR1A = 0;			// set Counter1 to normal Mode
  TCNT1 = 0;			// set Counter to 0
  TI1_INT_FLAGS = (1<<ICF1) | (1<<OCF1B) | (1<<OCF1A) | (1<<TOV1);	// clear interrupt flags

  #ifndef INHIBIT_SLEEP_MODE
    TIMSK1 = (1<<TOIE1) | (1<<ICIE1);	// enable Timer overflow interrupt and input capture interrupt
    unfinished = 1;
  #endif

  R_PORT = HiPinR_H;           	// switch R_H resistor port for HighPin to VCC

  if(PartFound == PART_FET) {
    // charge capacitor with R_H resistor
    TCCR1B = (1<<CS10);	//Start counter 1MHz or 8MHz
    ADC_DDR = (((1<<TP1) | (1<<TP2) | (1<<TP3) | TXD_MSK) & ~(1<<HighPin));	// release only HighPin ADC port
  } else {
    TCCR1B =  (1<<CS10);	// start counter 1MHz or 8MHz
    ADC_DDR = LoADC;		// stay LoADC Pin switched to GND, charge capacitor with R_H slowly
  }

  //******************************
  #ifdef INHIBIT_SLEEP_MODE
    while(1) {
      // Wait, until  Input Capture is set
      ii = TI1_INT_FLAGS;	// read Timer flags

      if (ii & (1<<ICF1))  {
        break;
      }

      if((ii & (1<<TOV1))) {		// counter overflow, 65.536 ms @ 1MHz, 8.192ms @ 8MHz
        TI1_INT_FLAGS = (1<<TOV1);	// Reset OV Flag
        wdt_reset();
        ovcnt16++;

        if(ovcnt16 == (F_CPU/5000)) {
          break; 			// Timeout for Charging, above 12 s
        }
      }
    }

    TCCR1B = (0<<ICNC1) | (0<<ICES1) | (0<<CS10);  // stop counter
    TI1_INT_FLAGS = (1<<ICF1);		// Reset Input Capture
    tmpint = ICR1;			// get previous Input Capture Counter flag

    // check actual counter, if an additional overflow must be added
    if((TCNT1 > tmpint) && (ii & (1<<TOV1))) {
      // this OV was not counted, but was before the Input Capture
      TI1_INT_FLAGS = (1<<TOV1);	// Reset OV Flag
      ovcnt16++;
    }

  #else
    while(unfinished) {
      set_sleep_mode(SLEEP_MODE_IDLE);
      sleep_mode();    			// wait for interrupt
      wdt_reset();

      if(ovcnt16 == (F_CPU/5000)) {
        break; 				// Timeout for Charging, above 12 s
      }
    }

    TCCR1B = (0<<ICNC1) | (0<<ICES1) | (0<<CS10);  	// stop counter
    tmpint = ICR1;					// get previous Input Capture Counter flag
    TIMSK1 = (0<<TOIE1) | (0<<ICIE1);	// disable Timer overflow interrupt and input capture interrupt

    if (TCNT1 < tmpint) {
      ovcnt16--;			// one ov to much
    }
  #endif

  //------------------------------------------------------------
  ADCSRA = (1<<ADEN) | (1<<ADIF) | AUTO_CLOCK_DIV;	 // enable ADC
  R_DDR = 0;					// switch R_H resistor port for input
  R_PORT = 0;					// switch R_H resistor port pull up for HighPin off
  adcv[2] = ReadADC(HighPin);   		// get loaded voltage
  load_diff = adcv[2] + REF_C_KORR - ref_mv;	// build difference of capacitor voltage to Reference Voltage
  //------------------------------------------------------------

  if (ovcnt16 >= (F_CPU/10000)) goto keinC;	// no normal end

  //cap.cval_uncorrected = CombineII2Long(ovcnt16, tmpint);
  cap.cval_uncorrected.w[1] = ovcnt16;
  cap.cval_uncorrected.w[0] = tmpint;

  cap.cpre = -12;			// cap.cval unit is pF 
  if (ovcnt16 > 65) {
    cap.cval_uncorrected.dw /= 100;	// switch to next unit
    cap.cpre += 2;			// set unit, prevent overflow
  }

  cap.cval_uncorrected.dw *= RHmultip;		// 708
  cap.cval_uncorrected.dw /= (F_CPU / 10000);	// divide by 100 (@ 1MHz clock), 800 (@ 8MHz clock)
  cap.cval = cap.cval_uncorrected.dw;		// set the corrected cap.cval
  Scale_C_with_vcc();

  if (cap.cpre == -12) {
    #if COMP_SLEW1 > COMP_SLEW2
      if (cap.cval < COMP_SLEW1) {
        // add slew rate dependent offset
        cap.cval += (COMP_SLEW1 / (cap.cval+COMP_SLEW2 ));
      }
    #endif

    if (HighPin == TP2) cap.cval += TP2_CAP_OFFSET;	// measurements with TP2 have 2pF less capacity

    if (cap.cval > C_NULL) cap.cval -= C_NULL;		// subtract constant offset (pF)
    else cap.cval = 0;			// unsigned long may not reach negativ value
  }

  R_DDR = HiPinR_L; 		// switch R_L for High-Pin to GND

  if(cap.cval < 25) goto keinC;  // capacity to low, < 50pF @1MHz (25pF @8MHz)

  // end low capacity 

checkDiodes:

  if((NumOfDiodes > 0)  && (PartFound != PART_FET)) {
  // nearly shure, that there is one or more diodes in reverse direction,
  // which would be wrongly detected as capacitor 
  } else {
    PartFound = PART_CAPACITOR;		// capacitor is found

    if ((cap.cpre > cap.cpre_max) || ((cap.cpre == cap.cpre_max) && (cap.cval > cap.cval_max))) {
      // we have found a greater one
      cap.cval_max = cap.cval;
      cap.cpre_max = cap.cpre;

      #if FLASHEND > 0x1fff
        cap.v_loss = vloss;		// lost voltage in 0.01%
      #endif

      cap.ca = LowPin;		// save LowPin
      cap.cb = HighPin;		// save HighPin
    }
  }

keinC:
  ADC_DDR =  TXD_MSK;		// switch all ADC ports to input
  ADC_PORT = TXD_VAL;		// switch all ADC outputs to GND, no pull up
  R_DDR = 0;			// switch all resistor ports to input
  R_PORT = 0; 			// switch all resistor outputs to GND, no pull up

  return;
}  // end ReadCapacity()


unsigned int getRLmultip(unsigned int cvolt) {
  #define RL_Tab_Abstand 25     // displacement of table 25mV
  #define RL_Tab_Beginn 300     // begin of table ist 300mV
  #define RL_Tab_Length 1100    // length of table is 1400-300

  unsigned int uvolt;
  unsigned int y1, y2;
  uint8_t tabind;
  uint8_t tabres;

  if (cvolt >= RL_Tab_Beginn) uvolt = cvolt - RL_Tab_Beginn;
  else uvolt = 0;			// limit to begin of table

  tabind = uvolt / RL_Tab_Abstand;
  tabres = uvolt % RL_Tab_Abstand;
  tabres = RL_Tab_Abstand - tabres;

  if (tabind > (RL_Tab_Length/RL_Tab_Abstand)) {
    tabind = (RL_Tab_Length/RL_Tab_Abstand);	// limit to end of table
  }

  y1 = MEM_read_word(&RLtab[tabind]);
  y2 = MEM_read_word(&RLtab[tabind+1]);
  return ( ((y1 - y2) * tabres + (RL_Tab_Abstand/2)) / RL_Tab_Abstand + y2); // interpolate table
}

void Scale_C_with_vcc(void) {

  while (cap.cval > 100000) {
    cap.cval /= 10;
    cap.cpre ++;			// prevent overflow
  }

  cap.cval *= ADCconfig.U_AVCC;		// scale with measured voltage
  cap.cval /= U_VCC;			// Factors are computed for U_VCC
}

#ifndef INHIBIT_SLEEP_MODE
// Interrupt Service Routine for timer1 Overflow
ISR(TIMER1_OVF_vect, ISR_BLOCK)
{
  ovcnt16++;				// count overflow
}

// Interrupt Service Routine for timer1 capture event (Comparator)
ISR(TIMER1_CAPT_vect, ISR_BLOCK)
{
  unfinished = 0;			// clear unfinished flag
}
#endif

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

// new code by K.-H. Kubbeler
// The 680 Ohm resistor (R_L_VAL) at the Lowpin will be used as current sensor
// The current with a coil will with (1 - e**(-t*R/L)), where R is
// the sum of Pin_RM , R_L_VAL , Resistance of coil and Pin_RP.
// L in the inductance of the coil.

//=================================================================
void ReadInductance(void) {
#if FLASHEND > 0x1fff

  // check if inductor and measure the inductance value
  unsigned int tmpint;
  unsigned int umax;
  unsigned int total_r;		// total resistance of current loop
  unsigned int mess_r;		// value of resistor used for current measurement
  unsigned long inductance[4];	// four inductance values for different measurements

  union t_combi{
  unsigned long dw;     // time_constant
  uint16_t w[2];
  } timeconstant;

  uint16_t per_ref1,per_ref2;	// percentage
  uint8_t LoPinR_L;	// Mask for switching R_L resistor of low pin
  uint8_t HiADC;	// Mask for switching the high pin direct to VCC
  uint8_t ii;
  uint8_t count;	// counter for the different measurements

  //uint8_t found;	// variable used for searching resistors 
  #define found 0

  uint8_t cnt_diff;     // resistance dependent offset
  uint8_t LowPin;	// number of pin with low voltage
  uint8_t HighPin;	// number of pin with high voltage 
  int8_t ukorr;		// correction of comparator voltage
  uint8_t nr_pol1;	// number of successfull inductance measurement with polarity 1
  uint8_t nr_pol2;	// number of successfull inductance measurement with polarity 2

  if(PartFound != PART_RESISTOR) return;	// We have found no resistor

  if (ResistorsFound != 1) return;	// do not search for inductance, more than 1 resistor

  //for (found=0;found<ResistorsFound;found++) {
  //  if (resis[found].rx > 21000) continue;

  if (resis[found].rx > 21000) return;
  // we can check for Inductance, if resistance is below 2100 Ohm

  for (count=0; count<4; count++) {
    // Try four times (different direction and with delayed counter start)

    if (count < 2) {
      // first and second pass, direction 1
      LowPin = resis[found].ra;
      HighPin = resis[found].rb;
    } else {
      // third and fourth pass, direction 2
      LowPin = resis[found].rb;
      HighPin = resis[found].ra;
    }

    HiADC = pgm_read_byte(&PinADCtab[HighPin]);
    LoPinR_L = pgm_read_byte(&PinRLtab[LowPin]);	// R_L mask for HighPin R_L load

    //==================================================================================
    // Measurement of Inductance values
    R_PORT = 0;			// switch R port to GND
    ADC_PORT = TXD_VAL;		// switch ADC-Port to GND

    if ((resis[found].rx < 240) && ((count & 0x01) == 0)) {
      // we can use PinR_L for measurement
      mess_r = RR680MI - R_L_VAL;			// use only pin output resistance
      ADC_DDR = HiADC | (1<<LowPin) | TXD_MSK;		// switch HiADC and Low Pin to GND, 
    } else {
      R_DDR = LoPinR_L;   		// switch R_L resistor port for LowPin to output (GND)
      ADC_DDR = HiADC | TXD_MSK;	// switch HiADC Pin to GND 
      mess_r = RR680MI;			// use 680 Ohm and PinR_L for current measurement
    }

    // Look, if we can detect any current
    for (ii=0;ii<20;ii++) {
      // wait for current is near zero
      umax = W10msReadADC(LowPin);
      total_r =  ReadADC(HighPin);
      if ((umax < 2) && (total_r < 2)) break;	// low current detected
    }

    // setup Analog Comparator
    ADC_COMP_CONTROL = (1<<ACME);			// enable Analog Comparator Multiplexer
    ACSR =  (1<<ACBG) | (1<<ACI)  | (1<<ACIC);		// enable, 1.3V, no Interrupt, Connect to Timer1 
    ADMUX = (1<<REFS0) | LowPin;			// switch Mux to Low-Pin
    ADCSRA = (1<<ADIF) | AUTO_CLOCK_DIV; 		// disable ADC
   
    // setup Counter1
    timeconstant.w[1] = 0;	// set ov counter to 0
    TCCR1A = 0;			// set Counter1 to normal Mode
    TCNT1 = 0;			// set Counter to 0
    TI1_INT_FLAGS = (1<<ICF1) | (1<<OCF1B) | (1<<OCF1A) | (1<<TOV1);	// reset TIFR or TIFR1
    HiADC |= TXD_VAL;
    wait200us();		// wait for bandgap to start up

    if ((count & 0x01) == 0 ) {
      // first start counter, then start current
      TCCR1B =  (1<<ICNC1) | (0<<ICES1) | (1<<CS10);	// start counter 1MHz or 8MHz
      ADC_PORT = HiADC;					// switch ADC-Port to VCC
    } else {
      // first start current, then start counter with delay
      // parasitic capacity of coil can cause high current at the beginning
      ADC_PORT = HiADC;		// switch ADC-Port to VCC

      wait3us();		// ignore current peak from capacity
      TI1_INT_FLAGS = (1<<ICF1);			// Reset Input Capture
      TCCR1B =  (1<<ICNC1) | (0<<ICES1) | (1<<CS10);	// start counter 1MHz or 8MHz
    }
      
    //******************************
    while(1) {
      // Wait, until  Input Capture is set
      ii = TI1_INT_FLAGS;		// read Timer flags

      if (ii & (1<<ICF1)) break;

      if((ii & (1<<TOV1))) {		// counter overflow, 65.536 ms @ 1MHz, 8.192ms @ 8MHz
        TI1_INT_FLAGS = (1<<TOV1);	// Reset OV Flag
        wdt_reset();
        timeconstant.w[1]++;		// count one OV

        if(timeconstant.w[1] == (F_CPU/100000UL)) break; 			// Timeout for Charging, above 0.13 s
      }
    }

    TCCR1B = (0<<ICNC1) | (0<<ICES1) | (0<<CS10);  	// stop counter
    TI1_INT_FLAGS = (1<<ICF1);				// Reset Input Capture
    timeconstant.w[0] = ICR1;		// get previous Input Capture Counter flag

    // check actual counter, if an additional overflow must be added
    if((TCNT1 > timeconstant.w[0]) && (ii & (1<<TOV1))) {
      // this OV was not counted, but was before the Input Capture
      TI1_INT_FLAGS = (1<<TOV1);		// Reset OV Flag
      timeconstant.w[1]++;			// count one additional OV
    }

    ADC_PORT = TXD_VAL;					// switch ADC-Port to GND
    ADCSRA = (1<<ADEN) | (1<<ADIF) | AUTO_CLOCK_DIV; 	// enable ADC

    for (ii=0;ii<20;ii++) {
      // wait for current is near zero
      umax = W10msReadADC(LowPin);
      total_r =  ReadADC(HighPin);

      if ((umax < 2) && (total_r < 2)) break;	// low current detected
    }

    #define CNT_ZERO_42 6
    #define CNT_ZERO_720 7

    total_r = (mess_r + resis[found].rx + RRpinMI);
    cnt_diff = total_r / ((14000UL * 8) / (F_CPU/1000000UL));
    // Voltage of comparator in % of umax
    tmpint = (ref_mv + REF_C_KORR);
    if (mess_r < R_L_VAL) {
      cnt_diff = CNT_ZERO_42; // measurement without 680 Ohm
      if (timeconstant.dw < 225) ukorr = (timeconstant.w[0] / 5) - 20;
      else ukorr = 25;
      tmpint -= (40 + ukorr);
    } else {
      cnt_diff += CNT_ZERO_720; // measurement with 680 Ohm resistor
      tmpint += 40; // if 680 Ohm resistor is used, use REF_L_KORR for correction
    }

    if (timeconstant.dw > cnt_diff) timeconstant.dw -= cnt_diff;
    else timeconstant.dw = 0;
    if ((count&0x01) == 1) timeconstant.dw += (3 * (F_CPU/1000000UL))+10; // second pass with delayed counter start
    if (timeconstant.w[1] >= (F_CPU/100000UL)) timeconstant.dw = 0;  // no transition found
    if (timeconstant.dw > 10) timeconstant.dw -= 1;

    // compute the maximum Voltage umax with the Resistor of the coil
    umax = ((unsigned long)mess_r * (unsigned long)ADCconfig.U_AVCC) / total_r;
    per_ref1 = ((unsigned long)tmpint * 1000) / umax;
    //per_ref2 = (uint8_t)MEM2_read_byte(&LogTab[per_ref1]);	// -log(1 - per_ref1/100)
    per_ref2 = get_log(per_ref1);				// -log(1 - per_ref1/1000)

    //*********************************************************
    // lx in 0.01mH units, L = Tau * R
    per_ref1 = ((per_ref2 * (F_CPU/1000000UL)) + 5) / 10;
    inductance[count] = (timeconstant.dw * total_r ) / per_ref1;

    if (((count&0x01) == 0) && (timeconstant.dw > ((F_CPU/1000000UL)+3))) {
      // transition is found, measurement with delayed counter start is not necessary
      inductance[count+1] = inductance[count];	// set delayed measurement to same value
      count++;					// skip the delayed measurement
    }

    wdt_reset();
  }  // end for count

  ADC_PORT = TXD_VAL;		// switch ADC Port to GND
  wait_about20ms();

  nr_pol1 = 0;
  if (inductance[1] > inductance[0]) { nr_pol1 = 1; } 

  nr_pol2 = 2;
  if (inductance[3] > inductance[2]) { nr_pol2 = 3; } 

  if (inductance[nr_pol2] < inductance[nr_pol1]) nr_pol1 = nr_pol2;

  resis[found].lx = inductance[nr_pol1];
  resis[found].lpre = -5;					// 10 uH units

  if (((nr_pol1 & 1) == 1) || (resis[found].rx >= 240)) {
    // with 680 Ohm resistor total_r is more than 7460
    resis[found].lpre = -4;					// 100 uH units
    resis[found].lx = (resis[found].lx + 5) / 10;
  } 

  //} // end loop for all resistors

  // switch all ports to input
  ADC_DDR =  TXD_MSK;		// switch all ADC ports to input
  R_DDR = 0;			// switch all resistor ports to input

#endif
  return;
}  // end ReadInductance()


#if FLASHEND > 0x1fff
// get_log interpolate a table with the function -log(1 - (permil/1000))
uint16_t get_log(uint16_t permil) {
  #define Log_Tab_Distance 20           // displacement of table is 20 mil

  uint16_t y1, y2;			// table values
  uint16_t result;			// result of interpolation
  uint8_t tabind;			// index to table value
  uint8_t tabres;			// distance to lower table value, fraction of Log_Tab_Distance

  tabind = permil / Log_Tab_Distance;	// index to table
  tabres = permil % Log_Tab_Distance;	// fraction of table distance

  // interpolate the table of factors
  y1 = pgm_read_word(&LogTab[tabind]);		// get the lower table value
  y2 = pgm_read_word(&LogTab[tabind+1]); 	// get the higher table value

  result = ((y2 - y1) * tabres ) / Log_Tab_Distance + y1;  // interpolate
  return(result);
}
#endif

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

#define MAX_CNT 255

/* The sleep mode for ADC can be used. It is implemented for 8MHz and 16MHz operation */
/* But the ESR result is allways higher than the results with wait mode. */
/* The time of ESR measurement is higher with the sleep mode (checked with oszilloscope) */
/* The reason for the different time is unknown, the start of the next ADC measurement */
/* should be initiated before the next ADC-clock (8 us). One ADC takes 13 ADC clock + 1 clock setup. */
/* The setting to sleep mode takes 10 clock tics, the wakeup takes about 24 clock tics, but 8us are 64 clock tics. */
/* I have found no reason, why a reset of the ADC clock divider should occur during ESR measurement. */ 
//#define ADC_Sleep_Mode

#define StartADCwait() ADCSRA = StartADCmsk; /* Start conversion */\
while (ADCSRA & (1 << ADSC))  /* wait until conversion is done */

/************************************************************************/
/* Predefine the wait time for switch off the load current for big caps */
/************************************************************************/
//         wdt_reset();		// with wdt_reset the timing can be adjusted,
      // when time is too short, voltage is down before SH of ADC
      // when time is too long, capacitor will be overloaded.
      // That will cause too high voltage without current.
#define DelayBigCap() wait10us();	/* 2.5 ADC clocks = 20us */ \
        wait5us();		/*  */ \
        wait4us();	/* pulse length 19.375 us */ 
#define DelayBigCap() delayMicroseconds(20)
//=================================================================
uint16_t GetESR(uint8_t hipin, uint8_t lopin) {
#if FLASHEND > 0x1fff
  // measure the ESR value of capacitor
  unsigned int adcv[4];		// array for 4 ADC readings
  unsigned long sumvolt[4];	// array for 3 sums of ADC readings
  unsigned long cap_val_nF;
  uint16_t esrvalue;
  uint8_t HiPinR_L;		// used to switch 680 Ohm to HighPin
  uint8_t HiADC;		// used to switch Highpin directly to GND or VCC
  uint8_t LoPinR_L;		// used to switch 680 Ohm to LowPin
  uint8_t LoADC;		// used to switch Lowpin directly to GND or VCC
  uint8_t ii,jj;		// tempory values
  uint8_t StartADCmsk;		// Bit mask to start the ADC
  uint8_t SelectLowPin,SelectHighPin;
  uint8_t big_cap;
  int8_t esr0;			// used for ESR zero correction
  big_cap = 1;

  if (PartFound == PART_CAPACITOR) {
    ii = cap.cpre_max;
    cap_val_nF = cap.cval_max;

    while (ii < -9) { 		// set cval to nF unit
      cap_val_nF /= 10;		// reduce value by factor ten
      ii++;			// take next decimal prefix
    }

    if (cap_val_nF < (1800/18)) return(0xffff);		// capacity lower than 1.8 uF
    StartADCmsk =  (1<<ADSC) | (1<<ADEN) | (1<<ADIF) | AUTO_CLOCK_DIV;  // enable and start ADC
  }

  LoADC = pgm_read_byte(&PinADCtab[lopin]) | TXD_MSK;
  HiADC = pgm_read_byte(&PinADCtab[hipin]) | TXD_MSK;
  LoPinR_L = pgm_read_byte(&PinRLtab[lopin]);		// R_L mask for LowPin R_L load
  HiPinR_L = pgm_read_byte(&PinRLtab[hipin]);		// R_L mask for HighPin R_L load
  SelectLowPin = (lopin | (1<<REFS1) | (1<<REFS0));	// switch ADC to LowPin, Internal Ref. 
  SelectHighPin = (hipin | (1<<REFS1) | (1<<REFS0));	// switch ADC to HighPin, Internal Ref. 

  // Measurement of ESR of capacitors AC Mode
  sumvolt[0] = 1;		// set sum of LowPin voltage to 1 to prevent divide by zero
  sumvolt[2] = 1;		// clear sum of HighPin voltage with current
                                // offset is about (x*10*200)/34000 in 0.01 Ohm units
  sumvolt[1] = 0;		// clear sum of HighPin voltage without current
  sumvolt[3] = 0;		// clear sum of HighPin voltage without current
  EntladePins();		// discharge capacitor
  ADC_PORT = TXD_VAL;		// switch ADC-Port to GND
  ADMUX = SelectLowPin;		// set Mux input and Voltage Reference to internal 1.1V

  wait100us();		// time for voltage stabilization
  // start voltage must be negativ
  ADC_DDR = HiADC;			// switch High Pin to GND
  R_PORT = LoPinR_L;			// switch R-Port to VCC
  R_DDR = LoPinR_L;			// switch R_L port for HighPin to output (VCC)
  wait10us();
  wait2us();
  R_DDR = 0;				// switch off current
  R_PORT = 0;
  StartADCwait();			// set ADCSRA Interrupt Mode, sleep

  // Measurement frequency is given by sum of ADC-Reads < 680 Hz for normal ADC speed.
  // For fast ADC mode the frequency is below 2720 Hz (used for capacity value below 3.6 uF).
  // ADC Sample and Hold (SH) is done 1.5 ADC clock number after real start of conversion.
  // Real ADC-conversion is started with the next ADC-Clock (125kHz) after setting the ADSC bit.

  for(ii=0;ii<MAX_CNT;ii++) {
    ADC_DDR = LoADC;			// switch Low-Pin to output (GND)
    R_PORT = LoPinR_L;			// switch R-Port to VCC
    R_DDR = LoPinR_L;			// switch R_L port for LowPin to output (VCC)
    ADMUX = SelectLowPin;
    StartADCwait();			// set ADCSRA Interrupt Mode, sleep
    StartADCwait();			// set ADCSRA Interrupt Mode, sleep
    adcv[0] = ADCW;			// Voltage LowPin with current
    ADMUX = SelectHighPin;

    StartADCwait();			// ADCSRA = (1<<ADEN) | (1<<ADIF) | (1<<ADIE) | AUTO_CLOCK_DIV;	
    ADCSRA = (1<<ADSC) | (1<<ADEN) | (1<<ADIF) | AUTO_CLOCK_DIV; // enable ADC and start with ADSC
    wait4us();
    R_PORT = HiPinR_L;			// switch R-Port to VCC
    R_DDR = HiPinR_L;			// switch R_L port for HighPin to output (VCC)
    DelayBigCap();			// wait predefined time

    R_DDR = 0;				// switch current off,  SH is 1.5 ADC clock behind real start
    R_PORT = 0;
    while (ADCSRA&(1<<ADSC));		// wait for conversion finished
    adcv[1] = ADCW;			// Voltage HighPin with current

    wdt_reset();

    // ******** Reverse direction, connect High side with GND ********
    ADC_DDR = HiADC;			// switch High Pin to GND
    R_PORT = HiPinR_L;			// switch R-Port to VCC
    R_DDR = HiPinR_L;			// switch R_L port for HighPin to output (VCC)
    wdt_reset();
    ADMUX = SelectHighPin;
    StartADCwait();			// set ADCSRA Interrupt Mode, sleep
    StartADCwait();			// set ADCSRA Interrupt Mode, sleep
    adcv[2] = ADCW;			// Voltage HighPin with current
    ADMUX = SelectLowPin;

    StartADCwait();			// set ADCSRA Interrupt Mode, sleep
    ADCSRA = (1<<ADSC) | (1<<ADEN) | (1<<ADIF) | AUTO_CLOCK_DIV; // enable ADC and start with ADSC
    wait4us();
    R_PORT = LoPinR_L;
    R_DDR = LoPinR_L;			// switch LowPin with 680 Ohm to VCC
    DelayBigCap();			// wait predefined time

    R_DDR = 0;				// switch current off
    R_PORT = 0;

    while (ADCSRA&(1<<ADSC));		// wait for conversion finished
    adcv[3] = ADCW;			// Voltage LowPin with current

    sumvolt[0] += adcv[0];		// add sum of both LowPin voltages with current
    sumvolt[1] += adcv[1];		// add  HighPin voltages with current
    sumvolt[2] += adcv[2];		// add  LowPin voltages with current
    sumvolt[3] += adcv[3];		// add  HighPin voltages with current
  } // end for

  sumvolt[0] += sumvolt[2];

  if ((sumvolt[1] + sumvolt[3]) > sumvolt[0]) sumvolt[2] = (sumvolt[1] + sumvolt[3]) - sumvolt[0];	// difference HighPin - LowPin Voltage with current
  else sumvolt[2] = 0;

  if (PartFound == PART_CAPACITOR) sumvolt[2] -= (1745098UL*MAX_CNT) / (cap_val_nF * (cap_val_nF + 19));

  esrvalue = (sumvolt[2] * 10 * (unsigned long)RRpinMI) / (sumvolt[0]+sumvolt[2]);
  esrvalue += esrvalue / 14;					// esrvalue + 7%
  esr0 = (int8_t)pgm_read_byte(&EE_ESR_ZEROtab[hipin+lopin]);

  if (esrvalue > esr0) esrvalue -= esr0;
  else esrvalue = 0;

  return (esrvalue);
#else
  return (0);
#endif
}

void us500delay(unsigned int us)  // = delayMicroseconds(us) + 500ns
{
  if (--us == 0) return;
  us <<= 2;
  __asm__ __volatile__ (
    "1: sbiw %0,1" "\n\t"            // 2 cycles
    "brne 1b" : "=w" (us) : "0" (us) // 2 cycles
  );
}

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

// new code by K.-H. Kubbeler
// ca   = Pin number (0-2) of the LowPin
// cb   = Pin number (0-2) of the HighPin

//=================================================================
void GetVloss() {
#if FLASHEND > 0x1fff
  // measure voltage drop after load pulse
  unsigned int tmpint;
  unsigned int adcv[4];

  union t_combi{
  unsigned long dw;     // capacity value  in 100nF units
  uint16_t w[2];
  } lval;

  uint8_t ii;
  uint8_t HiPinR_L;
  uint8_t LoADC;

  if (cap.v_loss > 0) return;		// Voltage loss is already known

  LoADC = pgm_read_byte(&PinADCtab[cap.ca]) | TXD_MSK;
  HiPinR_L = pgm_read_byte(&PinRLtab[cap.cb]);		// R_L mask for HighPin R_L load

  EntladePins();			// discharge capacitor
  ADC_PORT = TXD_VAL;			// switch ADC-Port to GND
  R_PORT = 0;				// switch R-Port to GND
  ADC_DDR = LoADC;			// switch Low-Pin to output (GND)
  R_DDR = HiPinR_L;			// switch R_L port for HighPin to output (GND)
  adcv[0] = ReadADC(cap.cb);		// voltage before any load 

  // ******** should adcv[0] be measured without current???
  if (cap.cpre_max > -9) return;	// too much capacity

  lval.dw = cap.cval_max;
  for (ii=cap.cpre_max+12;ii<4;ii++) lval.dw = (lval.dw + 5) / 10;

  if ((lval.dw == 0) || (lval.dw > 5000)) return;// capacity more than 50uF, Voltage loss is already measured

  R_PORT = HiPinR_L;  // R_L to 1 (VCC) 
  R_DDR = HiPinR_L;   // switch Pin to output, across R to GND or VCC

  for (tmpint=0; tmpint<lval.w[0]; tmpint+=2) wait5us();			// wait exactly 5us

  R_DDR = 0;			// switch back to input
  R_PORT = 0;			// no Pull up
  //wait10us();			// wait a little time
  wdt_reset();

  // read voltage without current
  ADCconfig.Samples = 5;	// set ADC to only 5 samples
  adcv[2] = ReadADC(cap.cb);
  if (adcv[2] > adcv[0]) adcv[2] -= adcv[0];		// difference to beginning voltage
  else adcv[2] = 0;		// voltage is lower or same as beginning voltage

  // wait 2x the time which was required for loading
  for (tmpint=0; tmpint<lval.w[0]; tmpint++) wait5us();

  adcv[3] = ReadADC(cap.cb);		// read voltage again, is discharged only a little bit ?
  ADCconfig.Samples = ANZ_MESS;		// set ADC back to configured No. of samples
  wdt_reset();

  if (adcv[3] > adcv[0]) adcv[3] -= adcv[0];			// difference to beginning voltage
  else adcv[3] = 0;			// voltage is lower or same as beginning voltage

  // build difference to load voltage
  if (adcv[2] > adcv[3]) adcv[1] = adcv[2] - adcv[3];	// lost voltage during load time wait
  else adcv[1] = 0;			// no lost voltage

  // compute voltage drop as part from loaded voltage
  if (adcv[1] > 0) {
    // there is any voltage drop (adcv[1]) !
    // adcv[2] is the loaded voltage.
    cap.v_loss = (unsigned long)(adcv[1] * 500UL) / adcv[2];
  }

  // discharge capacitor again
  EntladePins();		// discharge capacitors
  // ready
  // switch all ports to input

  ADC_DDR =  TXD_MSK;		// switch all ADC ports to input
  ADC_PORT = TXD_VAL;		// switch all ADC outputs to GND, no pull up
  R_DDR = 0;			// switch all resistor ports to input
  R_PORT = 0; 			// switch all resistor outputs to GND, no pull up

#endif
  return;
}  // end GetVloss()

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

void Calibrate_UR(void) {
  // get reference voltage, calibrate VCC with external 2.5V and
  // get the port output resistance
  //--------------------------------------------
  ADCconfig.U_AVCC = U_VCC;     // set initial VCC Voltage
  ADCconfig.Samples = 190;	// set number of ADC reads near to maximum

  #if FLASHEND > 0x1fff
    ADC_PORT = TXD_VAL;                 // switch to 0V
    ADC_DDR = (1<<TPREF) | TXD_MSK;     // switch pin with 2.5V reference to GND
    wait1ms();
    ADC_DDR =  TXD_MSK;   		// switch pin with reference back to input
    trans.uBE[1] = W5msReadADC(TPREF); 	// read voltage of 2.5V precision reference

    if ((trans.uBE[1] > 2250) && (trans.uBE[1] < 2750)) {
      // precision voltage reference connected, update U_AVCC
      WithReference = 1;
      ADCconfig.U_AVCC = (unsigned long)((unsigned long)ADCconfig.U_AVCC * 2495) / trans.uBE[1];
    }
  #endif
  ref_mv = DEFAULT_BAND_GAP;    	// set to default Reference Voltage
  ADCconfig.U_Bandgap = ADC_internal_reference; // set internal reference voltage for ADC
  ADCconfig.Samples = ANZ_MESS; 	// set to configured number of ADC samples
} 
 
/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */
void lcd_set_cursor(uint8_t row, uint8_t col) {
  display.setCursor(6*col, 10*row);
}

void lcd_pgm_string(const unsigned char *data) {
  unsigned char cc;
  while(1) {
    cc = pgm_read_byte(data);
    if((cc == 0) || (cc == 128)) return;
    display.write(cc);
    data++;
  }
}

// sends numeric character (Pin Number) to the LCD 
// from binary 0 we send ASCII 1
void lcd_testpin(unsigned char temp) {
  display.write(temp + '1');
}

void lcd_fix_string(const unsigned char *data) {
  unsigned char cc;
  while(1) {
    cc = MEM_read_byte(data);
    if((cc == 0) || (cc == 128)) return;
    display.write(cc);
    data++;
  }
}