/*
	Thing
 
 	Provides a short text message interface to the Arduino with
        four whitelisted numbers.
 
 	The circuit:
 	* I2C bus: Accelerometer, gyroscope, barometer, thermometer
 	* OneWire bus: 2 Dallas thermometers
        * 10,11 - SMS serial
        * 0,1 - Logging serial
 
 	Created 4 February 2019
 	By Nicholas Taylor
 	Modified 5 March 2019
 	By Nicholas Taylor
 
 	http://url/of/online/tutorial.cc
 	http://stackr.ca
 
        Note:
        /usr/share/arduino/libraries/SoftwareSerial
        SoftwareSerial.h
        #define _SS_MAX_RX_BUFF 128 // RX buffer size
 
 */

#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 8
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

#include <OneButton.h>
#include <Time.h>
#include <TimeLib.h>
#include <LiquidCrystal_I2C.h>

#include <Adafruit_BMP085.h>
//#include <L3G4200D.h>
#include <ADXL345.h>
#include <HMC5883L.h>
#include <Wire.h>

//#include <EEPROM.h>
#include <avr/wdt.h>

Adafruit_BMP085 bmp;
//L3G4200D gyroscope;
ADXL345 accelerometer;
HMC5883L compass;

OneButton button(2, false);

int error = 0;
//MagnetometerScaled valueOffset;

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); //

#include <SoftwareSerial.h>

SoftwareSerial portSMS(10, 11);

// software serial #2: RX = digital pin 8, TX = digital pin 9
// on the Mega, use other pins instead, since 8 and 9 don't work on the Mega
//SoftwareSerial portNMEA(12, 13);

char charBuf[11]; // For number to text conversion

char sms_serial_buffer[24]; //30>22

char nom_from[12];
char subject[10];

char instruction[5];

int sms_available_bytes = 0;
const byte sms_serial_buffer_length = sizeof(sms_serial_buffer) - 1;

char sms_whitelist[4][12] = {
  "XXXXXXXXXX",
  "XXXXXXXXXX",
  "XXXXXXXXXX",
  "XXXXXXXXXX"};

const byte PROGMEM cron_hours = 4;
  
const byte PROGMEM num_whitelist = sizeof(sms_whitelist) / sizeof(sms_whitelist[0]);

char search_for[11];

byte sms_pointer_write = 0;
byte sms_pointer_read = 0;

byte sms_budget = 0;
const byte PROGMEM sms_budget_max = 1;
byte sms_quota = 0;
const byte PROGMEM sms_quota_max = 3; // Allow user to send up to 3 messages a minute
byte priority_quota = 0;
const byte PROGMEM priority_quota_max = 1; // Allow user to send up to 1 priority messages per 80 bar
byte stack_quota = 0;
const byte PROGMEM stack_quota_max = 2; // Allow user to send up to 2 priority messages per 80 bar
byte display_budget = 0;
const byte PROGMEM display_budget_max = 1;

// Establish ticking clock
byte ticks = 0;
const byte PROGMEM tick_limit = 3;
byte bar = 0 ; 
const byte PROGMEM bar_limit = 80 ;

boolean flag_thing = false;

char response[144]; //80

char agent[16];
char prior_agent[16];

unsigned long current_millis = millis(); 
unsigned long tick_millis = current_millis;

unsigned long accelerometer_millis = current_millis;
//unsigned long split_millis = millis();

const int PROGMEM tick_interval = 100; // Crank up the tick

const byte PROGMEM screenWidth = 16;
const byte PROGMEM screenHeight = 2;

//const static char uuid[] PROGMEM = "bf7c";
const char uuid[] = "0000-0000-0000-0000-000000000000";
const char nuuid[] = "0000";

byte line0_index = 0;
byte line1_index = 0;

boolean flag_priority = false;
boolean flag_cron = false;

boolean state = false;

// Initialise Pitch, Roll and Yaw
float pitch = 0;
float roll = 0;
float yaw = 0;

float heading;

float voltage;
float tesla;

const float PROGMEM g = 9.81;

float depth;

// Initialize components of acceleration
double fXg[2], fYg[2], fZg[2]; 
float delta_t;

double vX[2], vY[2], vZ[2];
double sX[2], sY[2], sZ[2];

float heave;
float Z_e_peak = 0;
float Z_e_peak_temp = 0;

float cosRoll;
float sinRoll;
float cosPitch;
float sinPitch;

void setup()
{
  wdt_enable(WDTO_8S);
  
  lcd.begin(16,2);
  lcd.backlight();//Power on the back light
  
  // set up the serial console
  Serial.begin(4800);
  // Set up the connection to the modem
  portSMS.begin(9600);

  if (!bmp.begin()) {
    //Serial.println(F("Could not find a valid BMP085 sensor, check wiring!"));
    //while (1) {
    //}
  }

  // Set scale 2000 dps and 400HZ Output data rate (cut-off 50)
  //  if(!gyroscope.begin(L3G4200D_SCALE_2000DPS, L3G4200D_DATARATE_400HZ_50)) {
  //    Serial.println(F("Could not find a valid L3G4200D sensor, check wiring!"));
  //    while (1) {}
  //    delay(500);
  //  }

  error = compass.setScale(1.3);
  error = compass.setMeasurementMode(MEASUREMENT_CONTINUOUS);
  //compassCalibrate();

  accelerometer.begin();



  button.attachPress(doPress);

  sensors.begin(); 

  // Send initial text messages.
  // To first (primary) SMS number.
  strcpy(nom_from, sms_whitelist[0]);
  strcpy(agent, "start");
  strcpy(instruction, "+"); 

  doBudget();
  doStart();

  strcpy(nom_from, "NULL"); 
  
  doTick();
}

void loop()
{ 
  // Wonder if memory usage can be reduced.
  unsigned long current_millis = millis();

  button.tick();

  isAgent();

  // Read the USB serial port.
  // When plugged in you can send raw test to the Thing.
  /*
  if(Serial.available()) // Check for availablity of data at Serial Port
  {   

    flag_thing = false;

    int availableBytes = Serial.available();
    for(int i=0; i<availableBytes; i++)
    {
      //data[i] = Serial.read();
      Serial.read();
    }
  }

  meep:
  */

  if ((unsigned long)(current_millis - tick_millis) >= tick_interval) {
    doTick();
    tick_millis = current_millis;
  }
  wdt_reset();

}

// Private function: from http://arduino.cc/playground/Code/AvailableMemory  
int doMemory () {
  extern int __heap_start, *__brkval; 
  int v; 
  int memory = (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);

  strcpy(response, ""); 

  dtostrf(memory, 5, 0, charBuf);

  strcat(response, deblank(charBuf));
  strcat(response, " bytes");
  
  strcpy(agent, "memory");
  return memory;
}

void doStart()
{
  byte stored_state = EEPROM.read(7);

  // 255 no reading saved. Or failed.
  if (stored_state == 255) {state = false;}
  if (stored_state == 0) {state = true;EEPROM.write(7,255);}

  portSMS.print(F("AT"));
  portSMS.write(0x0D);  // Carriage Return in Hex
  portSMS.write(0x0A);  // Line feed in Hex

  portSMS.print(F("AT+CMGF=1"));
  portSMS.write(0x0D);  // Carriage Return in Hex
  portSMS.write(0x0A);  // Line feed in Hex

  if (state) {
    strcpy(response, "Started.");
  } else {
    strcpy(response, "Squawk.");
  }

  if (strstr(agent, "start")) {
    doMessage();
  }

  strcpy(agent, "start");
}

void doKaiju()
{
  if (bar == 0) {strcpy(response, "Started.");}
  if (bar == 21) {strcpy(response, "Looped.");}
  
  if (strstr(agent, "kaiju")) {
    doMessage();
  }
  strcpy(agent, "kaiju");
}

void doUuid()
{
  strcpy(response, "");
  strcat(response, uuid);
  
  if (strstr(agent, "uuid")) {
    doMessage();
  }
  strcpy(agent, "uuid");
}

void doEngine()
{
  sensors.requestTemperatures();
  float temperature = sensors.getTempCByIndex(1);
  strcpy(response, ""); 

  dtostrf(temperature, 5, 1, charBuf);

  strcat(response, deblank(charBuf));
  strcat(response, "C");
  
  if (strstr(agent,"engine")) {
    doMessage();
  }
  
  strcpy(agent, "engine");
}

void doThing()
{
  strcpy(response, ""); 

  strcat(response, nuuid);
  strcat(response, " ");
  
  numberChar(voltage, 5, 2);
    
  strcat(response, "V ");
  
  float temperature = bmp.readTemperature();  
  dtostrf(temperature, 5, 1, charBuf);
  strcat(response, deblank(charBuf));
  strcat(response, "C ");
  
  float pressure = bmp.readPressure();
  numberChar(pressure, 7, 0);
  strcat(response, "Pa ");
   
  numberChar(tesla / (10000/1000), 5, 2); 
  strcat(response, "uT ");
  
  numberChar(Z_e_peak, 5, 2); 
  strcat(response, "g");
  
  sensors.requestTemperatures();
  for (int i = 0; i < 3; i++) {
    strcat(response, " ");
    temperature = sensors.getTempCByIndex(i);
    numberChar(temperature, 5, 1);
    strcat(response, "C"); 
  }

  strcat(response, " ");    
  numberChar(depth, 5, 0);
  strcat(response, "mm");

  if (strstr(agent,"thing")) {
    doMessage();
  }
  
  strcpy(agent, "thing");
}

void doCockpit()
{
  sensors.requestTemperatures();
  float temperature = sensors.getTempCByIndex(0);

  strcpy(response, ""); 
  
  numberChar(temperature, 5, 1); 
  strcat(response, "C");      

  strcat(response, " roll ");
  numberChar(roll * 180 / M_PI, 4, 0);

  strcat(response, " ");
  
  numberChar(heading * 180/M_PI, 4, 0);
  strcat(response, "M");
  
  if (strstr(agent,"cockpit")) {
    doMessage();
  } 
  
  strcpy(agent, "cockpit");
}

void doRoll()
{
  strcpy(response, "");      
 
  numberChar(roll * 180 / M_PI, 4, 0);
  strcat(response, " degrees");
  
  strcpy(agent, "roll");
}

void doPitch()
{
  strcpy(response, "");      
  
  numberChar(pitch * 180 / M_PI, 4, 0);
  strcat(response, " degrees");
  
  strcpy(agent, "pitch");
}

void doCabin()
{
  sensors.requestTemperatures();
  float temperature = sensors.getTempCByIndex(2);
  strcpy(response, ""); 

  numberChar(temperature, 5, 1);

  strcat(response, "C");
  
  if (strstr(agent,"cabin")) {
    doMessage();
  }
  
  strcpy(agent, "cabin");
}

void doHeave()
{
  // Identify the peak vertical acceleration in the period.
  // Approximation metric for 'choppiness'.
  if ((bar % 5 == 0) and (ticks == 0)) {
    Z_e_peak = Z_e_peak_temp;
    Z_e_peak_temp = 0;
  }
 
  float a3 = -1 * sinPitch;
  float b3 = sinRoll * cosPitch;
  float c3 = cosRoll * cosPitch;
//  float Z_e = g * (1 - (a3*fXg[0]+b3*fYg[0]+c3*fZg[0]));

  float Z_e = (1 - (a3*fXg[0]+b3*fYg[0]+c3*fZg[0]));  
 
   if (Z_e < 0) {Z_e = -1 * Z_e;}
 
  if (Z_e > Z_e_peak_temp) {Z_e_peak_temp = Z_e;}
  
  strcpy(response, "");
  numberChar(Z_e_peak, 7, 2); 
  strcat(response, "g");
 
  if (strstr(agent,"heave")) {
    doMessage();
  } 
  
  strcpy(agent, "heave"); 
  
}

boolean doPriority()
{
  strcpy(agent, "priority");

  if ( doBilge() or doVoltage() ) {  
    // Turn off the flag
    append(response, "PRIORITY ", 0);
    // Allow one PRIORITY message to be sent on toggle.    

    ++priority_quota;
    if (priority_quota > priority_quota_max) {
      priority_quota = priority_quota_max;
      return true;
    }
 
    strcpy(instruction, "+");
    doBudget();    

    return true;
  } else {
    // Allow one PRIORITY message to be sent on toggle.

  }
  
  if (priority_quota < priority_quota_max) {
  }

  return false;

}

// https://stackoverflow.com/questions/7459259/inserting-characters-into-a-string
// inserts into subject[] at position pos
void append(char subject[], const char insert[], int pos) {
    char buf[100] = {}; // 100 so that it's big enough. fill with zeros
    // or you could use malloc() to allocate sufficient space
    // e.g. char *buf = (char*)malloc(strlen(subject) + strlen(insert) + 2);
    // to fill with zeros: memset(buf, 0, 100);

    strncpy(buf, subject, pos); // copy at most first pos characters
    int len = strlen(buf);
    strcpy(buf+len, insert); // copy all of insert[] at the end
    len += strlen(insert);  // increase the length by length of insert[]
    strcpy(buf+len, subject+pos); // copy the rest

    strcpy(subject, buf);   // copy it back to subject
    // Note that subject[] must be big enough, or else segfault.
    // deallocate buf[] here, if used malloc()
    // e.g. free(buf);
}

void doSMS()
{
  boolean do_not_send = false;
  
  if (sms_budget <= 0) {
    do_not_send = true;
  }

  if (sms_quota >= sms_quota_max) {
    do_not_send = true;
  }
  
  boolean match = false;
  for (int i = 0; i < num_whitelist; i++) {
    if (strstr(nom_from, sms_whitelist[i])) {match = true;}
  }

  if (!match) {
    do_not_send = true;
  }
  
  if ((stack_quota >= stack_quota_max) and (strstr(nom_from, sms_whitelist[1]))) {
	// quota for stack messages exceeded
    do_not_send = true;
  }
  
  if (do_not_send) {
    strcpy(nom_from, "NULL");
    strcpy(instruction, "-");
    doBudget();
    return;
  }

  // Authorized to send an SMS
  portSMS.print(F("AT+CMGS=\""));

  portSMS.print(nom_from);
  portSMS.println(F("\""));

  append(response, " | ", 0);
  append(response, strupr(agent), 0);

  unsigned long split_millis = millis();
  strcpy(search_for, ">");
  while (!doListen()) {
    if ((unsigned long)(millis() - split_millis) >= 2000) {
      break;
    }
    doHear();
  }

  portSMS.print(response);  //SMS body
  delay(500);
  portSMS.write(0x1A);  // sends ctrl+z end of message
  portSMS.write(0x0D);  // Carriage Return in Hex
  portSMS.write(0x0A);  // Line feed in Hex

  delay(500); // Test.

  // At this point an SMS has been sent.
  // Debit the thing account.
  strcpy(instruction, "-");
  doBudget();

  doQuota();

  strcpy(response, "");     
  strcat(response, "Sent.");

  // Turn off priority?  
  flag_priority = false;
  
  strcpy(agent, "sms"); 
  strcpy(nom_from, "NULL"); 

}

void doIndex(int index)
{

  if (bar == 0) {
    // Reset the number of allowed priority messages.
    priority_quota = 0;
    stack_quota = 0;
  }

  switch (index) {
  case 0:
  case 1:
    doWeather();
    break;
  case 2:
    doEngine();
    break;
  case 3:
    doCabin();
    break;
  case 4:
  case 5:
    doAccelerometer();
    break;
  case 6:
    doCompass();
    break;
  case 7:
    doVoltage();
    break;
  case 8:
    doTesla();
    break;
  case 9:
  case 10:
    // Run Cockpit for 4 bars.
    doCockpit();
    break;
  case 11:
    doRoll();
    break;
  case 12:
    doPitch();
    break;
  case 13:
    doFlag();
    break;
  case 14:
  case 15:
  case 16:
  case 17:
  case 18:
    doBilge();
    break;
  case 19: 
    // Integrity check.
    doMemory();
    break; 
  case 20:
    doClock();
    break;   
  case 21:
  	if (EEPROM.read(7) == 255) {EEPROM.write(7,0);}
    doBar();
    break;  
  default:
    // statements
    break;
  }  
}

void doAgent()
{   
  // Implement a simple round robin.
  // tick_interval determines how quickly the round is run.

  int index = bar % 21;

  if (bar == 0) {
    strcpy(instruction, "R");
    doQuota();
  }
  
  doIndex(index);
}


boolean doVoltage()
{

  int bl = false;
  // Tailoring.
  int max = 1023;
  int max_measurement = 23.05;
  int min = 0;
  int min_measurement = 0;
  
  // Read the sensor.  
  int sensorValue = analogRead(A1);
  //Serial.println(sensorValue);
  
  voltage = min_measurement + float (max_measurement - min_measurement) * (float (sensorValue - min) / float(max-min));

  strcpy(response, "");
  numberChar(voltage, 5, 2); 
  strcat(response, "V");
  
  if (voltage < 0) {
    strcpy(response, "X");
  } 

  if (voltage <= 11.50) {  
    flag_priority = true;
    bl = true;
  }  

  if (strstr(agent,"voltage")) {
    doMessage();
  } 

  strcpy(agent, "voltage");
    
  return bl;
}


void doTemperature()
{
  float temperature = bmp.readTemperature();

  strcpy(response, "");

  numberChar(temperature, 7, 2);  
  strcat(response, "C");
  
  strcpy(agent, "temperature"); 
}

void doWeather()
{
  sensors.requestTemperatures();
  float number = sensors.getTempCByIndex(0);

  strcpy(response, "");
  numberChar(number, 5, 1);
  
  strcat(response, "C ");

  float pressure = bmp.readPressure();
  numberChar(pressure/100, 7, 0);

  strcat(response, "mbar " );
  
  if (strstr(agent,"weather")) {
    doMessage();
  } 
  
  strcpy(agent, "weather"); 
}
/*
void doGyro()
 {
 //    if ((unsigned long)(current_millis - tick_millis) >= interval) {
 
 float timeStep = (current_millis - gyro_millis) / 1000;
 gyro_millis = current_millis;
 
 //Serial.println(timeStep);
 
 // Read normalized values
 Vector norm = gyroscope.readNormalize();
 
 // Calculate Pitch, Roll and Yaw
 
 //float pitch = pitch + norm.YAxis * timeStep ;
 //float roll = roll + norm.XAxis * timeStep ;
 float yaw = yaw + (norm.ZAxis * timeStep); 
 
 strcpy(response, "GYRO | "); 
 
 //dtostrf(pitch, 6, 2, charBuf);
 //strcat(response, "pitch ");
 //strcat(response, deblank(charBuf));
 
 //dtostrf(roll, 6, 2, charBuf);
 //strcat(response, " roll ");
 //strcat(response, deblank(charBuf));
 
 dtostrf(yaw, 6, 2, charBuf);
 strcat(response, " yaw ");
 strcat(response, deblank(charBuf)); 
 }
 */

 
void doAccelerometer() {
  const float alpha = 0.5;

  double Xg, Yg, Zg;
  accelerometer.read(&Xg, &Yg, &Zg);
  
  delta_t = (millis() - accelerometer_millis) / 1000;
  accelerometer_millis = millis();

  //Low Pass Filter
  fXg[0] = Xg * alpha + (fXg[0] * (1.0 - alpha));
  fYg[0] = Yg * alpha + (fYg[0] * (1.0 - alpha));
  fZg[0] = Zg * alpha + (fZg[0] * (1.0 - alpha));
  
  //Roll & Pitch Equations
  roll  = atan2(fYg[0], fZg[0]);
  pitch = atan2(fXg[0], sqrt(fYg[0]*fYg[0] + fZg[0]*fZg[0]));
  
  cosRoll = cos(roll);
  sinRoll = sin(roll);
  cosPitch = cos(pitch);
  sinPitch = sin(pitch);
  
  yaw = heading + PI; // Does this belong here.
  yaw = reduceDirection(yaw);
  
  doHeave();
  
  strcpy(response, ""); 

  numberChar(fXg[0], 5, 2);
  strcat(response, "g ");

  numberChar(fYg[0], 5, 2);
  strcat(response, "g ");
  
  numberChar(fZg[0], 5, 2);
  strcat(response, "g ");
  
  numberChar(Z_e_peak, 5, 2);
  strcat(response, "g ");

  numberChar(roll*180.0/M_PI, 5, 2);
  strcat(response, " ");

  numberChar(pitch*180.0/M_PI, 5, 2);
  strcat(response, " ");

  numberChar(yaw*180.0/M_PI, 5, 2);
 
  strcpy(agent, "accelerometer");
  doSerial();  
}

void numberChar(float number, int digits, int precision)
{
  char* thisString = dtostrf(number, digits, precision, charBuf);
  strcat(response, deblank(thisString));
}

void doPressure()
{
  float pressure = bmp.readPressure();
  
  numberChar(pressure, 5, 0);
  
  strcat(response, "Pa");
  
  strcpy(agent, "pressure");
}

void doAgents()
{
  strcpy(response, "");
  strcat(response, "Weather / Cockpit / Bilge / Cabin / Engine"); 

  if (strstr(agent,"agents")) {
    doMessage();
  } 
  strcpy(agent, "agents");
}

void doMessage()
{
  doSerial();   
  doDisplay();  
  doSMS();  
}

void doCrow()
{
  if (strstr(agent,"crow")) {
    //boolean match = false;
    for (int i = 0; i < num_whitelist; i++) {
      strcpy(nom_from, sms_whitelist[i]);
      
      strcpy(instruction, "+");
      doBudget();
      strcpy(response, "Awk.");
      strcpy(agent, "corvid");
      doMessage();
    }
  } 
  strcpy(agent, "crow");
  
}

void doSerial()
{   
  Serial.print(strupr(agent));
  Serial.print(" | ");
  Serial.println(response);
}

void doTesla()
{
  MagnetometerScaled scaled = compass.readScaledAxis(); //

  tesla = sqrt(scaled.XAxis * scaled.XAxis + scaled.YAxis * scaled.YAxis + scaled.ZAxis * scaled.ZAxis);

  strcpy(response, "");  
  numberChar(tesla / (10), 5, 2);
  strcat(response, "uT");

  if (tesla >= 80 / (10000/1000)) {  
    flag_priority = true; 
  }
  
  strcpy(agent, "tesla");
}

float reduceDirection(float direction)
{

  if (direction < 0) { 
    direction += 2 * PI; 
  }
  if (direction > 2 * PI) { 
    direction -= 2 * PI; 
  }  
  return direction;
}


void doCompass()
{
  // Retreive the scaled values from the compass (scaled to the configured scale).
  MagnetometerScaled scaled = compass.readScaledAxis(); //

  // Tilt compensation
  float Xh = scaled.XAxis * cosPitch + scaled.ZAxis * sinPitch; //XAxis
  float Yh = scaled.XAxis * sinRoll * sinPitch + scaled.YAxis * cosRoll - scaled.ZAxis * sinRoll * cosPitch; //YAxis
  
  heading = atan2(Yh, Xh); 
  heading = reduceDirection(heading);

  float heading_degrees = heading * 180/M_PI;

  strcpy(response, "");
  numberChar(heading_degrees, 4, 0);
  strcat(response, "M");

  float heading_true = (int (heading_degrees + 16.09) % 359);

  strcat(response, " ");
  numberChar(heading_true, 4, 0);
  strcat(response, "T");
  
  strcpy(agent, "compass");
}

boolean doBudget()
{
  if (strstr(instruction,"+")) {
    ++sms_budget;
    if (sms_budget >= sms_budget_max) {
      sms_budget = sms_budget_max;
    }
  } 

  // Stack budget. Always okay to say something.
  display_budget = 1;

  // Thing budget. There is a limit.
  if (strstr(instruction,"-")) {
    if (sms_budget == 0) {
      return false;
    }
    --sms_budget;
  } 
  strcpy(instruction, "");

  if (sms_budget > 0) {
    return true;
  }

  return false;
}

boolean doQuota()
{
  if (strstr(instruction,"R")) {
    sms_quota = 0;
  } 

  ++sms_quota;

  strcpy(instruction, ""); 
  if (sms_quota > sms_quota_max) {
    sms_quota = sms_quota_max; 
    return false;
  }

  return true;
}

void doPress()
{
  strcpy(instruction, "+");
  doBudget();

  strcpy(response, "");
  strcat(response, "Pressed.");
  strcpy(agent, "press");
}

// Character and character string manipulation help.
char * charPad(char* str){
  for(int i = 0; i < (15 - sizeof(str)); i++) {
    strcat(str, " "); 
  }
  return str;
}

char * spacePad(char* input_str, int before, int characters){

  char * output_str = input_str+2;

  int right_pad_length = (characters - sizeof(output_str)-1);

  char * pad_str;

  for(int i = 0; i <= right_pad_length; i++) {
    strcat(pad_str, "X"); 
  }    
  return output_str;
}

char* booleanChar(boolean var) {

  if (var == true) {
    return "1";
  } 
  else {
    return "0";
  }

}

void doFlag() {
  strcpy(response, "");

  flag_thing = doOK();

  if (flag_thing) {
    strcat(response, "Red.");
  } 
  else {
    strcat(response, "Green.");
  }

  if (strstr(agent,"flag")) {
    doMessage();
  }
  
  strcpy(agent, "flag");
}


boolean randomBoolean()
{
  long randNumber = random(0,255);

  if (randNumber > 128) {
    return true;
  } 
  else {
    return false;
  } 
}

void doLCD(char* line0, char* line1)
{

  lcd.setCursor(0, 0);

  byte length = lcd.print(line0 + line0_index);

  for(byte i = length; i < 16; i++){
    lcd.write(' ');
  }

  lcd.setCursor(0, 1);

  if (strlen(line1) < screenWidth) {
    line1_index = 0;
  }
  
  length = lcd.print(line1 + line1_index);
  for(byte i = length; i < 16; i++){
    lcd.write(' ');
  }
  
  line1_index = (line1_index + 1) % (strlen(line1) - screenWidth +  1);
}

char* subStr (char* input_string, char *separator, int segment_number)
{
  char *act, *sub, *ptr;
  static char copy[100];
  int i;

  strcpy(copy, input_string);
  for (i = 1, act = copy; i <= segment_number; i++, act = NULL)
  {
    sub = strtok_r(act, separator, &ptr);
    if (sub == NULL) break;
  }
  return sub;
}

void doDisplay() 
{
  if (display_budget <= 0) {
    return;
  }

  char * line[2];

  line[0] = strupr(agent);
  line[1] = response;

  doLCD(line[0],line[1]);
}

char* deblank(char* input)                                        
{
  int i,j;
  char *output=input;
  for (i = 0, j = 0; i<strlen(input); i++,j++)         
  {
    if (input[i]!=' ')                          
      output[j]=input[i];                    
    else
      j--;                                    
  }
  output[j]=0;
  return output;
}

char* depad(char* input)                                        
{
  int i,j;
  char *output=input;
  for (i = 0, j = 0; i<strlen(input); i++,j++)         
  {
    if (input[i]!=' ')                          
      output[j]=input[i];                    
    else
      j--;                                    
  }
  output[j]=0;
  return output;
}

// https://stackoverflow.com/questions/9072320/split-string-into-string-array
void doTick()
{

  doAccelerometer();
  doCompass();
  doTesla();
  
  strcpy(response, "tick "); 
  numberChar(ticks, 5, 0);
  
  ticks++;
 

  if (ticks>tick_limit) {
    ticks = 0;
    doBar();
  }  
  
  if (!doPriority()) {  doAgent(); }

  doMessage();
  strcpy(agent, "tick");
}

void doPing()
{

  strcpy(response, "");
  strcat(response, "Pong.");

  if (strstr(agent,"ping")) {
    doMessage();
  }  

  strcpy(agent, "ping");

}

void doClock()
{ 
  char timestr[9];
  timestr[0] = '0' + hour() / 10;
  timestr[1] = '0' + hour() % 10;
  timestr[2] = ':';
  timestr[3] = '0' + minute() / 10;
  timestr[4] = '0' + minute() % 10;
  timestr[5] = ':';
  timestr[6] = '0' + second() / 10;
  timestr[7] = '0' + second() % 10;
  timestr[8] = '\0';

  strcpy(response,"");
  strcat(response, deblank(timestr)); 

  strcpy(agent, "clock"); 
}

void doBar()
{
  bar++;

  if (bar>bar_limit) {  
    bar = 0;
  }

  numberChar(bar, 5, 0);

  doCron(); // Check the clock each and every bar.  Minimum.

  strcpy(prior_agent, agent);
  strcpy(agent, "bar");
}

boolean doBilge()
{
  int bl = false;
  // Tailoring.
  int max = 935;
  int max_measurement = 350 + 75;
  // 38,39 at bottom jumps to 595
  int min = 595;
  int min_measurement = 75;

  // Read the sensor.  
  int sensorValue = analogRead(A0);
  depth = min_measurement + float (max_measurement - min_measurement) * (float (sensorValue - min) / float(max-min));

  strcpy(response, "");
  numberChar(depth, 5, 0); 

  if (depth < 0) {
    depth = 0;
    strcpy(response, "<75");
  } 
  strcat(response, "mm");
  
  if (depth >= 400) {  
    flag_priority = true;
    bl = true;
  }  

  if (strstr(agent,"bilge")) {
    doMessage();
  } 

  strcpy(agent, "bilge");
  return bl;
}

void doForget()
{
  delay(1500);

  portSMS.println("AT+CMGD=0,4");

  strcpy(response, "");     
  strcat(response, "Forgot messages.");
  
  strcpy(agent, "forget");
}

void doHayes()
{

  if (strstr(agent,"AT")) {
    strcpy(response, "OK");
  }

  if (strstr(agent,"ATI")) {
    strcpy(response, "OK");
  }

  if (strstr(agent,">")) {
    portSMS.print(response);  //SMS body
    delay(500);
    portSMS.write(0x1A);  // sends ctrl+z end of message
    portSMS.println();
    delay(500);
  }


  if (strstr(agent,"+CMTI")) {
    int number = 0;
    int n = 0;
    int fp = 0;

    strcpy(response, "Received.");
    strcpy(instruction, "+");
    doBudget();

    //Listen for numbers
    number = doNumber();

    portSMS.print(F("AT+CMGR="));
    portSMS.print(number);
    portSMS.write(0x0D);  // Carriage Return in Hex

    if (isAgent()) {
    }        
  }

  if (strstr(agent,"+SMS FULL")) {
    strcpy(response, "SMS Full.");
    strcpy(instruction, "+");
    doForget();
  }

}

int doNumber()
{
  int number = NULL;

  unsigned long sms_millis = millis();  
  boolean flag = false;
  int fp = sms_pointer_write;
  while ((unsigned long)(millis() - sms_millis) <= 2000) { // Listen to the buffer 10000
    doHear();

    fp = (fp + 1) % (sms_serial_buffer_length);

    if (isDigit(sms_serial_buffer[fp])){
      flag = true;
      int digit = sms_serial_buffer[fp] - '0';
      number = 10 * number + digit ;
    }

    if ((flag == true) and ( (isSpace(sms_serial_buffer[fp])) or (sms_serial_buffer[fp]== 88))) {
      return number;
    }
  }

  return NULL;
}

boolean isHit()
{
  if (doListen()) {
    doRespond();
    return true;  
  }  
  return false;
}

boolean isAgent()
{
  // Quickly establish if stream/channel mentions an agent we are watching for.
  int number = NULL;

  unsigned long sms_millis = millis();  
  boolean flag = false;
  int fp = sms_pointer_write;
  while ((unsigned long)(millis() - sms_millis) <= 2000) { // Listen to the buffer 2000
    button.tick();
    doHear();
    
    for (int i = 0; i < num_whitelist; i++) {
      strcpy(search_for, sms_whitelist[i]);

      if (doListen()) {
        doRespond(); 
      }

    }
  
    strcpy(search_for, "+CMTI");
    if (isHit()) {return true;}

    strcpy(search_for, "+SMS FULL");
    if (isHit()) {return true;}
  
    strcpy(search_for, "ping");
    if (isHit()) {return true;}

    strcpy(search_for, "bilge");
    if (isHit()) {return true;}

    strcpy(search_for, "weather");
    if (isHit()) {return true;}

    strcpy(search_for, "cockpit");
    if (isHit()) {return true;}

    strcpy(search_for, "cabin");
    if (isHit()) {return true;}
  
    strcpy(search_for, "crow");
    if (isHit()) {return true;}
       
    strcpy(search_for, "engine");
    if (isHit()) {return true;}

    strcpy(search_for, "flag");
    if (isHit()) {return true;}

    strcpy(search_for, "kaiju");    
    if (isHit()) {return true;}
    
    strcpy(search_for, "thing");    
    if (isHit()) {return true;}

    strcpy(search_for, "agents");
    if (isHit()) {return true;}

    strcpy(search_for, "uuid");
    if (isHit()) {return true;}
    
    strcpy(search_for, "heave");
    if (isHit()) {return true;}

  }

  return NULL;
}

void doStatus()
{   
  strcpy(response, "");
  strcat(response, "OK");
  
  strcpy(agent, "status");
}

void doCron()
{  
   
  if ( (hour() % cron_hours == 0) and (flag_cron == false)) { 
    strcpy(instruction, "+"); 
    doBudget(); 
    strcpy(nom_from,sms_whitelist[1]);
    doStack();
    flag_cron = true;
  }

  if ( (hour() % cron_hours != 0) and (flag_cron == true)) {
    stack_quota = 0;
    flag_cron = false;
  }

}

void doStack()
{    
  if (strstr(nom_from, sms_whitelist[1])) {
    ++stack_quota;  
  }
  
  if (stack_quota > stack_quota_max) {
    stack_quota = stack_quota_max;
  } 

  doThing();
  doSMS();

  strcpy(agent, "stack");

}

boolean doListen()
{
  boolean match = false;

  // Raw listening
  for( int i = sms_serial_buffer_length-strlen(search_for); i >0;  --i ) {  
    match = true;
    int fp = sms_pointer_write + i;
    for (int j = 0; j < strlen(search_for); ++j) {
      if (tolower(sms_serial_buffer[ (fp + j) % (sms_serial_buffer_length)]) != tolower(search_for[j])) {   
        match = false;
        // Take the first match.
        continue;
      }
    }

    // Action on the first text match.

    if (match) { 
      strcpy(agent, search_for);

      // Erase command from stream.  So we don't respond to it twice.
      // Actually X it out.
      for (int k = 0; k < strlen(search_for); ++k) {
        sms_serial_buffer[ ((fp + k) % (sms_serial_buffer_length)) ] = 88; 
      }
      break;
    }
  }     
  return match;
}

void doRespond()
{
  boolean flag = false;
  // Check if a whitelisted number has appeared.
  for (int i = 0; i < num_whitelist; i++) {
    if (strstr(agent,sms_whitelist[i])) {
      strcpy(nom_from, sms_whitelist[i]);
    }
  }

  // Check if the stack number has appeared.
  if (strstr(nom_from,sms_whitelist[1])) {
    doStack();
  }

  if (strstr(agent,"ping")) {
    doPing();
    flag = true;
  }

  if (strstr(agent,"weather")) {
    doWeather();
    flag = true;
  }
  
  if (strstr(agent,"cockpit")) {
    doCockpit();
    flag = true;
  }

  if (strstr(agent,"cabin")) {
    doCabin();
    flag = true;
  }

  if (strstr(agent,"bilge")) {
    doBilge();
    flag = true;
  }
  
  if (strstr(agent,"crow")) {
    doCrow();
    flag = true;
  }
 
  if (strstr(agent,"kaiju")) {
    doKaiju();
    flag = true;
  }
  
  if (strstr(agent,"flag")) {
    doFlag();
  }
  
  if (strstr(agent,"engine")) {
    doEngine();
    flag = true;
  }
  
  if (strstr(agent,"thing")) {
    doThing();
    flag = true;
  }
  
  if (strstr(agent,"agents")) {
    doAgents();
    flag = true;
  }
  
  if (strstr(agent,"uuid")) {
    doUuid();
    flag = true;
  }
  
  if (strstr(agent,"heave")) {
    doHeave();
    flag = true;
  }
  
  if (strstr(agent,"+CMTI")) {
    doHayes();
  }

  if (strstr(agent,"+SMS FULL")) {
    doHayes();
  }

  if (strstr(agent,">")) {
    doHayes();
  }

  strcpy(agent, "");
}

boolean doHear()
{
  portSMS.listen();

  if(portSMS.available()) // Check for availablity of data at Serial Port
  {
    sms_available_bytes = portSMS.available();
    int n;
    n = sms_available_bytes;
    if (n > 5) {n = 5;}

    for(int i=0; i<n; i++)
    {
      sms_pointer_write = (sms_pointer_write + 1) % (sms_serial_buffer_length);
      sms_serial_buffer[sms_pointer_write] = portSMS.read();
    }
  }
  sms_available_bytes = portSMS.available();
}

boolean doOK()
{
  unsigned long split_millis = millis();

  strcpy(search_for, "OK");
  while (!doListen()) {
    if ((unsigned long)(millis() - split_millis) >= 2000) {
      return false;
    }
    doHear();
  }
  return true;
}
