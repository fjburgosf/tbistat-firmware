
#include <Wire.h>
#include <si5351.h>
#include "ADC.h"
#include <IntervalTimer.h>

//---------------------------------------------------------------
// --- DEVICE CONFIGURATIONS
//---------------------------------------------------------------


//Analog signal input pin
#define ADCIn 14  //ADC 1 EIS

#define HWSERIAL Serial2

IntervalTimer timer0; // timers
ADC *adc = new ADC(); // adc object using ADC0

#define DDSON 8   // CV/CA on
#define EIS 7  // EIS on
#define A_00 6  //  AMPlifier resist one ON
#define A_01 5  //  AMPlifier resist two ON
#define EN 4  //  AMPlifier resist three ON

// AD5933 CONSTANTS AND DEFINITIONS
#define CTRL_OUTPUT_RANGE_1     (0b00000000)
#define CTRL_OUTPUT_RANGE_2     (0b00000110)
#define CTRL_OUTPUT_RANGE_3     (0b00000100)
#define CTRL_OUTPUT_RANGE_4     (0b00000010)
// Device address and address pointer
#define AD5933_ADDR     (0x0D)
#define ADDR_PTR        (0xB0)
// Control Register
#define CTRL_REG1       (0x80)
#define CTRL_REG2       (0x81)
// Start Frequency Register
#define START_FREQ_1    (0x82)
#define START_FREQ_2    (0x83)
#define START_FREQ_3    (0x84)
// Frequency increment register
#define INC_FREQ_1      (0x85)
#define INC_FREQ_2      (0x86)
#define INC_FREQ_3      (0x87)
// Number of increments register
#define NUM_INC_1       (0x88)
#define NUM_INC_2       (0x89)
// Number of settling time cycles register
#define NUM_SCYCLES_1   (0x8A)
#define NUM_SCYCLES_2   (0x8B)
// Status register
#define STATUS_REG      (0x8F)
// Real data register
#define REAL_DATA_1     (0x94)
#define REAL_DATA_2     (0x95)
// Imaginary data register
#define IMAG_DATA_1     (0x96)
#define IMAG_DATA_2     (0x97)
// Clock sources
#define CLOCK_INTERNAL  (CTRL_CLOCK_INTERNAL)
#define CLOCK_EXTERNAL  (CTRL_CLOCK_EXTERNAL)
// PGA gain options
#define PGA_GAIN_X1     (CTRL_PGA_GAIN_X1)
#define PGA_GAIN_X5     (CTRL_PGA_GAIN_X5)
// VP gain options
#define outvp2    (CTRL_OUTPUT_RANGE_1)
#define outvp1    (CTRL_OUTPUT_RANGE_2)
#define outvp_4 (CTRL_OUTPUT_RANGE_3)
#define outvp_2 (CTRL_OUTPUT_RANGE_4)
// Power modes
#define POWER_STANDBY   (CTRL_STANDBY_MODE)
#define POWER_DOWN      (CTRL_POWER_DOWN_MODE)
#define POWER_ON        (CTRL_NO_OPERATION)
// I2C result success/fail
#define I2C_RESULT_SUCCESS       (0)
#define I2C_RESULT_DATA_TOO_LONG (1)
#define I2C_RESULT_ADDR_NAK      (2)
#define I2C_RESULT_DATA_NAK      (3)
#define I2C_RESULT_OTHER_FAIL    (4)
// Control register options
#define CTRL_NO_OPERATION       (0b00000000)
#define CTRL_INIT_START_FREQ    (0b00010000)
#define CTRL_START_FREQ_SWEEP   (0b00100000)
#define CTRL_INCREMENT_FREQ     (0b00110000)
#define CTRL_REPEAT_FREQ        (0b01000000)
#define CTRL_TEMP_MEASURE       (0b10010000)
#define CTRL_POWER_DOWN_MODE    (0b10100000)
#define CTRL_STANDBY_MODE       (0b10110000)
#define CTRL_RESET              (0b00010000)
#define CTRL_CLOCK_EXTERNAL     (0b00001000)
#define CTRL_CLOCK_INTERNAL     (0b00000000)
#define CTRL_PGA_GAIN_X1        (0b00000001)
#define CTRL_PGA_GAIN_X5        (0b00000000)
// Status register options
#define STATUS_TEMP_VALID       (0x01)
#define STATUS_DATA_VALID       (0x02)
#define STATUS_SWEEP_DONE       (0x04)
#define STATUS_ERROR            (0xFF)
// Frequency sweep parameters
#define SWEEP_DELAY             (1)


//---------------------------------------------------------------
// --- ALL VARIABLES AND SETTINGS
//---------------------------------------------------------------

//--- System setting ---
volatile unsigned long initfreq = 0;             //EIS init frequency
volatile unsigned long increment = 0;             //EIS init frequency
volatile unsigned int nofincrements = 0;             //EIS init frequency
volatile byte eisclockinternal = 1; // eis clock source
volatile boolean internalclock = true; // eis clock source
volatile unsigned long externalclockvalue = 0;           //EIS ext clock value
volatile byte  eisintgain;
volatile byte  EisVPPlevel;
volatile bool eison = false;
int led = 13;


Si5351 si5351;
#define BUFFER_SIZE 1000
volatile uint16_t eis_data[BUFFER_SIZE]; 

byte Rbamp = 0;                              //amplifier resistance EIS
//--- System variables ---

volatile unsigned int N = 0;            //Current step number
volatile long outlast_Inp = 0;
const byte numChars = 50;
char receivedChars[numChars];   // an array to store the received data
static byte ndx = 0;
boolean first = false;

//---------------------------------------------------------------
// --- MAIN DEVICE INITIALIZATION
//---------------------------------------------------------------
void setup()
{
  SCB_SHPR3 = 0x20200000;
  pinMode(ADCIn, INPUT);
  HWSERIAL.begin(115200);
  delay(1000);

  pinMode(DDSON, OUTPUT);
  pinMode(EIS, OUTPUT);
  pinMode(A_00, OUTPUT);
  pinMode(A_01, OUTPUT);
  pinMode(EN, OUTPUT);
  pinMode(led, OUTPUT);
  digitalWriteFast(DDSON, HIGH); 
  digitalWriteFast(EIS, LOW); 
  digitalWriteFast(A_00, LOW); 
  digitalWriteFast(A_01, LOW);  
  digitalWriteFast(EN, LOW);  
  
  adc->adc0->setAveraging(1); // set number of averages
  adc->adc0->setResolution(12); // set bits of resolution  // 12 OR 16 
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // change the conversion speed
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); // change the sampling speed
  adc->adc0->startContinuous(ADCIn);
  
  timer0.priority(0);

  bool i2c_found = false;
        i2c_found = si5351.init(SI5351_CRYSTAL_LOAD_10PF, 0, 0);
        if (!i2c_found)
         {
     //sendResponse("Device not found on I2C bus!");
        }
        else {
      //sendResponse("found!!!");
          }
    digitalWriteFast(led, HIGH); 
  
}

//---------------------------------------------------------------
// --- MAIN LOOP
//---------------------------------------------------------------
//This is running continuously, unless device is in timed interupt
void loop()
{

  if (!eison) {
    cmdLine();
  }
   
}


void stoptimer()
{
  timer0.end();
}

//---------------------------------------------------------------
// --- INTERPRETING COMMANDS
//---------------------------------------------------------------
//This is interpreting incoming commands by ID character
//param is numerical input parameter of the command
bool action(char ID, long param)
{
  char charaux[50];
  switch (ID)
  {
    
     case 'A': //Set EIS TIA amplifier resistance Rb   
         Rbamp = (byte)param;
      
      if (Rbamp == 1) {
        digitalWriteFast(EN, LOW);  
        digitalWriteFast(A_00, LOW);  
        digitalWriteFast(A_01, LOW); 
        digitalWriteFast(EN, HIGH);
      }
      else if (Rbamp == 2) {
        digitalWriteFast(EN, LOW);  
        digitalWriteFast(A_00, HIGH);  
        digitalWriteFast(A_01, LOW); 
        digitalWriteFast(EN, HIGH);
      }
      else if (Rbamp == 3) {
        digitalWriteFast(EN, LOW);  
        digitalWriteFast(A_00, LOW);  
        digitalWriteFast(A_01, HIGH); 
        digitalWriteFast(EN, HIGH);
      }
      else if (Rbamp == 4) {
        digitalWriteFast(EN, LOW);  
        digitalWriteFast(A_00, HIGH);  
        digitalWriteFast(A_01, HIGH); 
        digitalWriteFast(EN, HIGH);
      }
      
      charaux[0] = 'D';
      ltoa(param, &charaux[1], 10);
      return true;

    case 'B': //Set initial frequency EIS
      initfreq = (unsigned long) param;
      charaux[0] = 'E';
      ltoa(param, &charaux[1], 10);
      return true;

    case 'C': //Set timer interrupt period
      increment = (unsigned long) param;
      charaux[0] = 'F';
      ltoa(param, &charaux[1], 10);
      return true;

    case 'D': //Set number of FFT points
      nofincrements = (unsigned int)param;
      charaux[0] = 'G';
      ltoa(param, &charaux[1], 10);
      return true;

    case 'E': //Set clock value
      externalclockvalue = (unsigned long)param;
      si5351.set_freq(externalclockvalue, SI5351_CLK0);
      delay(10);
      return true;  

    case 'F': //setup the EIS sequence
      eison = true;
      N = 0;
      HWSERIAL.end();
      digitalWriteFast(EIS, HIGH);  //
      digitalWriteFast(led, LOW); 
      delay(250);
      if (!first)   {setupad5933(); first=true;
      delay(4000);
      frequencySweepRaw();
      }
      else { frequencySweepRaw();
      }
      timerInit((int) increment);
      return true;

    case 'I': //Set EIS clock source
      eisclockinternal = (byte)param;

      if (eisclockinternal == 0) {
        internalclock = false;
      }
      else if (internalclock == 1) {
        internalclock = true;
      }
      charaux[0] = 'H';
      ltoa(param, &charaux[1], 10);
      return true;
     
     case 'J': // internal gain, 1 equal to gain 5, 0 to gain 1
      eisintgain = (byte)param;
      return true;

     case 'K': // VPP level, FROM 1 TO 4. 
      EisVPPlevel = (byte)param;
      
      return true;  
    
     case 'N': //Halt the sequence
      N = 0;
      digitalWriteFast(EIS, LOW);  //
      digitalWriteFast(EN, LOW); //
      sendResponse("m");
      return true;
    
    default:
      return false;
  }
}

//---------------------------------------------------------------
// --- PRECISION TIMING - Timer interupt on Timer 2
//---------------------------------------------------------------

//This function initializes timer and set the period defined by timerBaseStep
void timerInit(int srate)
{
  //adc->enableInterrupts(ADC_0);
  timer0.priority(0);
  timer0.begin(datacqu, srate); //25000 us is an int
  timer0.priority(0);
}

FASTRUN void datacqu(void)
{             
              if (eison){
                  N = N + 1;     
             if(N > nofincrements) {
                   stoptimer();
                   HWSERIAL.begin(115200);
                   delay(50);   // was in 10
                 for (unsigned int tt=0;tt<(N-1);tt++){
                      outlast_Inp=eis_data[tt]; 
                      eis_data[tt]=0; 
                      sendeisdata();  
                      delayMicroseconds(40);
                      HWSERIAL.flush();
                      }
                eison = false;
                digitalWriteFast(led, HIGH); 
                sendResponse("m");
                           }
                  else {
                   eis_data[N-1]=(uint16_t)adc->adc0->analogReadContinuous();
                  }
                
               }
    
}


//Command line processor. Return false if command isn't right or true if successfully processed
//All commands have structure "X(param)", where "X" is command identifing capital ASCII character
//and paremeter is numerical parameter in text format. 
void cmdLine()
{
  char ID;    //Command ID
  long param = 0; //Parameter of the function. Default is zero
  char datac[25];
  int begi = -1, endi = -1; //begin and end index: BEGIN "(" and END ")"
  char m;
  char endMarker = '\n';
  char rc;

  if (HWSERIAL.available() > 0) {
    rc = HWSERIAL.read();
    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    }
    else {

      receivedChars[ndx] = '\0'; // terminate the string
      m = ndx;
      ndx = 0;
      for (byte xx = 0; xx < m; xx++)
      {
        if (receivedChars[xx] == '(') begi = xx;
        if (receivedChars[xx] == ')') endi = xx;
      }
      if ((begi > 0) && (endi > begi)) //Looks like valid command
      {
        ID = receivedChars[begi - 1]; //ID character is just one before parenthesis
        if ((endi - begi) > 1) //There is also a parameter.
        {
          int k = 2;
          while (receivedChars[k] != ')') {
            datac[k - 2] = receivedChars[k];
            k++;
          }
          datac[k - 2] = '\0';
          param = atol(datac); //Convert parameter to number
     
          for (int k = 0; k < numChars; k++) {
            receivedChars[k] = '\0';
          }
        }
        action(ID, param);

      }

    }
  }

}

//Send respons back (text format)
void sendResponse(char val [])
{
  HWSERIAL.println(val);
}


//---------------------------------------------------------------
// --- HARDWARE COMMUNICATION
//---------------------------------------------------------------


void setupad5933()
{
  boolean setgain = false;
  boolean setvppgain = false;
  
   
   if (EisVPPlevel == 1) {
    setvppgain = AD5933setVGain(CTRL_OUTPUT_RANGE_1);
  }
  else if (EisVPPlevel == 2) {
    setvppgain = AD5933setVGain(CTRL_OUTPUT_RANGE_2);
  }
  else if (EisVPPlevel == 3) {
    
    setvppgain = AD5933setVGain(CTRL_OUTPUT_RANGE_3);

    if(setvppgain) {
      //sendResponse("GAIN WAS SET");
    } else {//sendResponse("GAIN WAS NOT SET");
    }

  }
  else if (EisVPPlevel == 4)  {
    setvppgain = AD5933setVGain(CTRL_OUTPUT_RANGE_4);
  }
   
  if (eisintgain == 0) {
    setgain = AD5933setPGAGain(PGA_GAIN_X1);
  }
  else  {
    setgain = AD5933setPGAGain(PGA_GAIN_X5);
  }
     
  if (!(AD5933setControlMode(CTRL_STANDBY_MODE) &&
        AD5933setInternalClock(internalclock) &&
        AD5933setStartFrequency(initfreq) &&
        AD5933setIncrementFrequency(0) &&
        AD5933setNumberIncrements(0) &&
        setvppgain &&
        setgain   //estaba en 30
       && AD5933setsettlingtime(50)))
  {
    //sendResponse("EIS:failed init");

  }
  else  {
    //sendResponse("EIS: Init succesfull");
  }

 AD5933frequencySweep();

}

void frequencySweepRaw() {
  
  AD5933reset();
  //AD5933setControlMode(CTRL_STANDBY_MODE);
  
  AD5933setStartFrequency(initfreq);
  AD5933setControlMode(CTRL_INIT_START_FREQ);
 
}

void sendeisdata() {

  char F[50] = "0";
  F[0] = 'H';
  ltoa(outlast_Inp, &F[1], 10);  //value, string result
  HWSERIAL.print(F);  
  HWSERIAL.println("F");
}

bool AD5933setPGAGain(byte gain) {
  // Get the current value of the control register
  byte val;
  if (!AD5933getByte(CTRL_REG1, &val))
    return false;

  // Clear out the bottom bit, D8, which is the PGA gain set bit
  val &= 0xFE;

  // Determine what gain factor was selected
  if (gain == PGA_GAIN_X1 || gain == 1) {
    // Set PGA gain to x1 in CTRL_REG1
    val |= PGA_GAIN_X1;
    return AD5933sendByte(CTRL_REG1, val);
  } else if (gain == PGA_GAIN_X5 || gain == 5) {
    // Set PGA gain to x5 in CTRL_REG1
    val |= PGA_GAIN_X5;
    return AD5933sendByte(CTRL_REG1, val);
  } else {
    return false;
  }
}

bool AD5933setVGain(byte vgain) {
  // Get the current value of the control register
  byte val;

  if (!AD5933getByte(CTRL_REG1, &val))
    return false;

  // Clear out bits 9 and 10, D9 and D10, which are the Vp excitation set bits
  val &= 0xF9;
  val |= vgain;
  return AD5933sendByte(CTRL_REG1, val);

}

int AD5933getByte(byte address, byte *value) {
  // Request to read a byte using the address pointer register
    
  Wire.beginTransmission(AD5933_ADDR);
  Wire.write(ADDR_PTR);
  Wire.write(address);
   
  // Ensure transmission worked
  if (byte res = Wire.endTransmission() != I2C_RESULT_SUCCESS) {
    *value = res;
    if (res==1) {sendResponse("data too long");}
    else if (res==2) {sendResponse("2:received NACK on transmit of address");}
    else if (res==3) {sendResponse("2:received NACK on transmit of data");}
    else if (res==4) {sendResponse("other error");}
   
    return false;
  }
  else {  }
         

  // Read the byte from the written address
  Wire.requestFrom(AD5933_ADDR, 1);
  if (Wire.available()) {
    *value = Wire.read();
    return true;
  } else {
    *value = 0;
    return false;
  }
}

bool AD5933sendByte(byte address, byte value) {
  // Send byte to address
  Wire.beginTransmission(AD5933_ADDR);
  Wire.write(address);
  Wire.write(value);

  // Check that transmission completed successfully
  if (byte res = Wire.endTransmission() != I2C_RESULT_SUCCESS) {
    return false;
  } else {
    return true;
  }
}

bool AD5933reset() {
  // Get the current value of the control register
  byte val;
  if (!AD5933getByte(CTRL_REG2, &val))
    return false;

  // Set bit D4 for restart
  val |= CTRL_RESET;

  // Send byte back
  return AD5933sendByte(CTRL_REG2, val);
}

bool AD5933setInternalClock(bool internal) {
  
  if (internal)
    return AD5933setClockSource(CLOCK_INTERNAL);
  else
    return AD5933setClockSource(CLOCK_EXTERNAL);
}

bool AD5933setClockSource(byte source) {
  // Determine what source was selected and set it appropriately
  switch (source) {
    case CLOCK_EXTERNAL:
      return AD5933sendByte(CTRL_REG2, CTRL_CLOCK_EXTERNAL);
    case CLOCK_INTERNAL:
      return AD5933sendByte(CTRL_REG2, CTRL_CLOCK_INTERNAL);
    default:
      return false;
  }
}

bool AD5933setStartFrequency(unsigned long start) {

  long freqHex = start;
  //long freqHex = 196608;
  if (freqHex > 0xFFFFFF) {
    return false;   // overflow
  }
  // freqHex should be a 24-bit value. We need to break it up into 3 bytes.
  byte high_Byte = (freqHex >> 16) & 0xFF;
  byte midByte = (freqHex >> 8) & 0xFF;
  byte low_Byte = freqHex & 0xFF;

  // Attempt sending all three bytes
  return AD5933sendByte(START_FREQ_1, high_Byte) &&
         AD5933sendByte(START_FREQ_2, midByte) &&
         AD5933sendByte(START_FREQ_3, low_Byte);
}

bool AD5933setIncrementFrequency(unsigned long increment) {

  long freqHex = increment;
  //long freqHex =32768;
  if (freqHex > 0xFFFFFF) {
    return false;   // overflow
  }

  // freqHex should be a 24-bit value. We need to break it up into 3 bytes.
  byte high_Byte = (freqHex >> 16) & 0xFF;
  byte midByte = (freqHex >> 8) & 0xFF;
  byte low_Byte = freqHex & 0xFF;

  // Attempt sending all three bytes
  return AD5933sendByte(INC_FREQ_1, high_Byte) &&
         AD5933sendByte(INC_FREQ_2, midByte) &&
         AD5933sendByte(INC_FREQ_3, low_Byte);
}

bool AD5933setNumberIncrements(unsigned int num) {
  // Check that the number sent in is valid.
  if (num > 511) {
    return false;
  }

  // Divide the 9-bit integer into 2 bytes.
  byte high_Byte = (num >> 8) & 0xFF;
  byte low_Byte = num & 0xFF;

  // Write to register.
  return AD5933sendByte(NUM_INC_1, high_Byte) &&
         AD5933sendByte(NUM_INC_2, low_Byte);
}

bool AD5933setPowerMode(byte level) {
  // Make the appropriate switch. TODO: Does no operation even do anything?
  switch (level) {
    case POWER_ON:
      return AD5933setControlMode(CTRL_NO_OPERATION);
    case POWER_STANDBY:
      return AD5933setControlMode(CTRL_STANDBY_MODE);
    case POWER_DOWN:
      return AD5933setControlMode(CTRL_POWER_DOWN_MODE);
    default:
      return false;
  }
}

bool AD5933setControlMode(byte mode) {
  // Get the current value of the control register
  byte val;
  if (!AD5933getByte(CTRL_REG1, &val))
    return false;

  // Wipe out the top 4 bits...mode bits are bits 5 through 8.
  val &= 0x0F;

  // Set the top 4 bits appropriately
  val |= mode;

  // Write back to the register
  return AD5933sendByte(CTRL_REG1, val);
}

byte AD5933readStatusRegister() {
  return AD5933readRegister(STATUS_REG);
}

byte AD5933readRegister(byte reg) {
  // Read status register and return it's value. If fail, return 0xFF.
  byte val;
  if (AD5933getByte(reg, &val)) {
    return val;
  } else {
    return STATUS_ERROR;
  }
}

bool AD5933setsettlingtime(unsigned int num) {
  // Check that the number sent in is valid.
  if (num > 511) {
    return false;
  }

  // Divide the 9-bit integer into 2 bytes.
  byte high_Byte = (num >> 8) & 0xFF;
  byte low_Byte = num & 0xFF;
  
  // Write to register.
  return AD5933sendByte(NUM_SCYCLES_2, low_Byte);
}

void AD5933frequencySweep() { 
    AD5933reset(); 
    AD5933setControlMode(CTRL_INIT_START_FREQ); // init start freq   
 }
