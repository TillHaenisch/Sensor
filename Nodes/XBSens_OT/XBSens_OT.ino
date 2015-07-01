
/**
 *  HYT939 Sensor with XBee data transmission
 *
 *  Data is captured in constant time intervals (every 4 seconds) and stored in the controller
 *  for a certain time (which may vary with battery status), default is 5 minutes.
 *  Then the data is transferred via XBee to a base station.
 *
 *  In addition to the measured temperature and humidity the supply voltage level is
 *  transferred to allow battery status monitoring
 * 
 *   XBee module is hibernated by pin mode (take pin 9 of module to low to force sleep).
 *
 *  V 0.1 Initial release August 2013
 *
 *  V 0.2 Adapted for slower transfer over I2C to be able to use longer lines April 2014
 *
 *  V 1.0 Consolidated Version, data is aggregated at the node and only the mean and stddev is
 *        transmitted to the base station. January 2015
 *  V 1.1 Also read data from two ADT7410 sensors and transmit them. March 2015
 *
 *  (c) Till Haenisch (till.haenisch@mac.com) 2013, 2014, 2015
 *
 **/

#define FIO 1

#define DEBUG_WITHOUT_XBEE true

#ifdef LEONARDO
#define SERIAL_BEGIN Serial1.begin
#define SERIAL_PRINT Serial1.print
#define SERIAL_PRINTLN Serial1.println
#define SERIAL_FLUSH Serial1.flush
#define SERIAL_READ Serial1.read
#define SERIAL_AVAILABLE Serial1.available
#else
#define SERIAL_BEGIN Serial.begin
#define SERIAL_PRINT Serial.print
#define SERIAL_PRINTLN Serial.println
#define SERIAL_FLUSH Serial.flush
#define SERIAL_READ Serial.read
#define SERIAL_AVAILABLE Serial.available
#endif

/*
* set I2C frequency to 10000 Hz instead of the default 100 kHz.
* We don't need the speed and may use longer lines (up to a few meters depending on cable).
* ATTENTION: This must be defined in the appropriate header file
* ./Arduino.app/Contents/Resources/Java/hardware/arduino/avr/libraries/Wire/utility/twi.h
*  #define TWI_FREQ 10000L
*  #define PRESCALER_10KHZ 4
* because it has to be used when compiling the corresponding library file
* (happens automatically when starting the IDE)
* see http://forum.arduino.cc/index.php/topic,16793.0.html
*
* T. Haenisch 25.4.2014
*/

#include <Wire.h>
#include "LowPower.h"


// Default number of controller sleep cycles (4 seconds each) before data transmission
// A value of 15 means a data transfer every minute
#define DATA_CACHE_SIZE 15

// After reset, the XBee module get's some time to join the network (in msec)
#define XBEE_RESET_DELAY 10000

// HYT 939 Humidity Sensor Address
#define I2C_SENS 0x28 

// HYT sensor is powered by digital port 6 to be able to switch off the sensor completely
// (shouldn't be neccessary cause the sensor doesn't need much power, but just in case ...)
#define PIN_SENS 6

// XBee sleep pin is connected to digital port 7
#define PIN_XBEE 7

// Status LED for signaling
#define PIN_LED 13

// Define ADT7410 address and Register Addresses. 
// With sensor board 1 A0 and A1 are grounded.
#define ADT7410Address 0x48
// For sensor board 2 A0 is high, A1 is ground
#define ADT7410Address2 0x49
// ADT7410 registers
#define ADT7410TempReg 0x00
#define ADT7410ConfigReg 0x03

// RAW 16-bit, signed data from ADT7410.
// (tempReading / 128) for positive temps in Celcius.
// ((tempReading - 65536) / 128) for negative temps in Celcius.
// For Farenheight ((ABOVERESULT * 1.8) + 32).
int tempReading = 0;
int tempReading2 = 0;


// Every sensor node needs a unique (in it's measurement network) ID
// The ID is configured in the XBee module (NI attribute)
// The ID is a two character string, for convenience with output we use a longer string here
// which can hold the separator (blank and string terminator)
char sensor_id[4] = {0,0,0,0};


// Data cache: The controller measures humidity and temperature every 4 seconds. These values are stored
// internally for a certain number of samples (DATA_CACHE_SIZE). When the cache is full, the data is transmitted
// via the XBee module. This reduces the power consumption of the system, DATA_CACHE_SIZE is the way to adjust
// battery life to the actual requirements, since the main power consumption is caused by the XBee module
static int cache_idx;

unsigned int temp_data[DATA_CACHE_SIZE];
unsigned int humidity_data[DATA_CACHE_SIZE];

long dummy = 0;

// Values from the ADT7410 sensors
int OT2_data[DATA_CACHE_SIZE];
int OT1_data[DATA_CACHE_SIZE];

// Number of the transmitted block (is sent with every block to detect missing data)
unsigned int block_number;

// Credit for readVcc to Scott Daniels
// http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

boolean get_sensor_id() {
  // read sensor ID (Node Identifier) from XBee module

  char tmp;

  // eat up characters in buffer
  while(SERIAL_AVAILABLE () > 0)
    tmp = SERIAL_READ ();

  // enter command mode
  SERIAL_PRINT ("+++");
  SERIAL_FLUSH ();

  delay(2000);
  while(SERIAL_AVAILABLE () > 0)
    tmp = SERIAL_READ ();

  delay(500);
  SERIAL_PRINTLN("ATNI");
  SERIAL_FLUSH ();

  delay(100);

  if (SERIAL_AVAILABLE () >=2 ) {
    sensor_id[0] = SERIAL_READ ();
    sensor_id[1] = SERIAL_READ ();

    sensor_id[2] = ' ';
    sensor_id[3] = 0;    
    return true;
  }
  return false;
}

void setup()
{
  // Turn on HYT power and let it run forever. Power consumption of the sensor can be neglected.
  // That might change in the future for extreme low power requirements.
  pinMode(PIN_SENS,OUTPUT);
  digitalWrite(PIN_SENS,HIGH);

  // Wake XBee module and give some time to join network
  pinMode(PIN_XBEE,OUTPUT);
  digitalWrite(PIN_XBEE,HIGH);
  
  // We wait a while to give the ZigBee module time to join the network
  // The better solution would be to monitor the XBEE ready pin .....
  // But in any case we need a delay esp. without a XBEE module plugged in to give
  // the Arduino IDE some time to connect to the board when uploading the code:
  // If we wait to short, than the module goes to sleep mode which would prevent uploading
  // new software. So it's better to wait a little bit.
  delay(XBEE_RESET_DELAY); 

  // Initialize status LED
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED,LOW);

  // For low level I2C (humidity sensor)
  Wire.begin();

  SERIAL_BEGIN (9600);
  delay(100);

  get_sensor_id();

  ADT7410INIT();

  // send XBEE to sleep
  digitalWrite(PIN_XBEE,LOW);

  cache_idx = 0;
  block_number = 0;
}




void loop()
{
  unsigned int humidity = 0;
  unsigned int temp = 0;
  
  long humidity_mean = 0;
  long temp_mean = 0;
  long humidity_std_dev = 0;
  long humidity_sq = 0;
  long temp_std_dev = 0;
  long temp_sq = 0;
  
  long OT1_mean = 0;
  long OT2_mean = 0;

  int i;
dummy++;
  // our sleep interval is 4 sec
  LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF); // Wollen wir wirklich BOD_OFF ?? HACK HACK HACK


  // signal, that we are awake
  digitalWrite(PIN_LED,HIGH);

  // read humidity sensor data
  Wire.beginTransmission(I2C_SENS);
  // send MR command --> write address of sensor
  Wire.write(I2C_SENS);
  Wire.endTransmission();

  // wait til sensor data available
  LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_OFF); // Wollen wir wirklich BOD_OFF ?? HACK HACK HACK

  Wire.beginTransmission(I2C_SENS);  
  Wire.requestFrom(I2C_SENS,4);
  unsigned char data[4];

  while(Wire.available()) {
    data[i++] = Wire.read();
  }
  humidity = data[0];
  humidity &= 0x3f;
  humidity *= 255;
  humidity += data[1];
  
//      Serial.print("Luftfeuchtigkeit: ");
//      Serial.println(100.0/16384.0*humidity);
  double dHumidity = 100.0/16384.0*humidity;

  temp = data[2];
  temp *= 255;
  temp += data[3];
  temp /= 4;
  double dTemp = 165.0/16384.0*temp-40;
  
//      Serial.print("Temperatur: ");
//      Serial.println(165.0/16384.0*temp-40);
  
  // Read data from the ADT7410 sensors
  ADT7410GetTemp();
  /*Serial.print("ADT7410: ");
  Serial.print(tempReading);
  Serial.print(" ");
  Serial.println(tempReading2); */

  
 

  // Data is stored in the cache and transmitted only, when the cache is full
  if (cache_idx < DATA_CACHE_SIZE) {
    temp_data[cache_idx] = temp;
    humidity_data[cache_idx] = humidity;
    
    OT1_data[cache_idx] = tempReading;
    OT2_data[cache_idx] = tempReading2;
    
  }
  else {
    // cache is full, data has to be transmitted

    // send data to XBee module
    // first wake it up
    pinMode(PIN_XBEE,OUTPUT);
    digitalWrite(PIN_XBEE,LOW);

    // wait a little bit
    delay(100);

    if (sensor_id[0] == 0)
      get_sensor_id();

    if ((sensor_id[0] != 0) || DEBUG_WITHOUT_XBEE) {
      // Send data
      // First the header
      SERIAL_PRINT (sensor_id);
      SERIAL_PRINT ("DATA ");
      SERIAL_PRINT (block_number);
      SERIAL_PRINT (" ");
      SERIAL_PRINT (DATA_CACHE_SIZE);
      SERIAL_PRINT (" ");
      SERIAL_PRINT (readVcc());
      
      // Now the data values
      for(i=0;i<DATA_CACHE_SIZE;i++) {
        /*SERIAL_PRINT (sensor_id);
        SERIAL_PRINT (temp_data[i]);
        SERIAL_PRINT (" ");
        SERIAL_PRINTLN(humidity_data[i]);
        */
        humidity_mean += humidity_data[i];
        temp_mean += temp_data[i];
        humidity_sq += 1L*humidity_data[i]*humidity_data[i];
        temp_sq += 1L*temp_data[i]*temp_data[i];
        
        OT1_mean += OT1_data[i];
        OT2_mean += OT2_data[i];
      }
      
      // Averaged values are multiplied by 100 before rounding to retain the additional precision from averaging
      // We don't want to use floating point numbers in our data transmission
      
      humidity_std_dev = (long) round(100.0*sqrt(1.0/(DATA_CACHE_SIZE-1) * (humidity_sq - 1.0*humidity_mean*humidity_mean/DATA_CACHE_SIZE)));
      temp_std_dev = (long) round(100.0*sqrt(1.0/(DATA_CACHE_SIZE-1) * (temp_sq - 1.0*temp_mean*temp_mean/DATA_CACHE_SIZE)));
      humidity_mean = (long) round(100.0*humidity_mean/DATA_CACHE_SIZE);
      temp_mean = (long) round(100.0*temp_mean/DATA_CACHE_SIZE);
      OT1_mean = (long) round(100.0*OT1_mean/DATA_CACHE_SIZE);
      OT2_mean = (long) round(100.0*OT2_mean/DATA_CACHE_SIZE);
      
      // If we have ADT7410 sensors connected, we transmit their values (means) instead of standard deviation
      // This has the advantage, that the ZigBee packet has the same size (and the same structure) as with the
      // standalone humidity sensors. No additional problems .... ;-)
      // March 2015
      SERIAL_PRINT (" ");
      SERIAL_PRINT (temp_mean);
      SERIAL_PRINT (" ");
      SERIAL_PRINT (humidity_mean);
      SERIAL_PRINT (" ");
//      SERIAL_PRINT (temp_std_dev);
      SERIAL_PRINT (OT1_mean);
//      SERIAL_PRINT (OT1_data[0]);
      SERIAL_PRINT (" ");
//      SERIAL_PRINT (humidity_std_dev);
//      SERIAL_PRINT (OT2_data[0]);
      SERIAL_PRINT (OT2_mean);
      SERIAL_PRINT (" ");
      SERIAL_PRINTLN ("END");  
      SERIAL_FLUSH ();
      
      // wait a little bit and send XBee to sleep again
      delay(500);
    }
    
    digitalWrite(PIN_XBEE,HIGH); // just in case
    // XBee pins are clamped internally
    pinMode(PIN_XBEE,INPUT);

    block_number++;
    cache_idx = 0;
    temp_data[cache_idx] = temp;
    humidity_data[cache_idx] = humidity;
    OT1_data[cache_idx] = tempReading;
    OT2_data[cache_idx] = tempReading2;
  }
  cache_idx++;
  // Signal that we are in sleep mode again
  digitalWrite(PIN_LED,LOW);
}

void ADT7410INIT() {
  //Initialization of the ADT7410 sets the configuration register based on input from the
  //Analog Devices datasheet page 14. 16-bit resolution selected.
  Wire.beginTransmission(ADT7410Address);
  Wire.write(0x03);
  Wire.write(B10000000);
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(ADT7410Address2);
  Wire.write(0x03);
  Wire.write(B10000000);
  Wire.endTransmission();
  delay(10);
}

void ADT7410GetTemp()
{
  byte MSB;
  byte LSB;
  delay(10);
  // Send request for temperature register.
  Wire.beginTransmission(ADT7410Address);
  Wire.write(ADT7410TempReg);
  Wire.endTransmission();
  // Listen for and acquire 16-bit register address.
  Wire.requestFrom(ADT7410Address, 2);
  MSB = Wire.read();
  LSB = Wire.read();
  // Assign global 'tempReading' the 16-bit signed value.
  tempReading = ((MSB << 8) | LSB);

  delay(10);
  Wire.beginTransmission(ADT7410Address2);
  Wire.write(ADT7410TempReg);
  Wire.endTransmission();
  // Listen for and acquire 16-bit register address.
  Wire.requestFrom(ADT7410Address2, 2);
  MSB = Wire.read();
  LSB = Wire.read();
  // Assign global 'tempReading' the 16-bit signed value.
  tempReading2 = ((MSB << 8) | LSB);
}


