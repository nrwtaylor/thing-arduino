/*
	Thing
 
 	Connects an arduino to a SMS HAYES command set based modem.
 	Responds to text messages with sensed and derivedinformation
	about the Thing.
 
 	The circuit:
 	* I2C bus: Accelerometer, gyroscope, barometer, thermometer
 	* OneWire bus: 2 Dallas thermometers
        * 10,11 - SMS serial
        * 0,1 - Logging serial
 
 	Created 4 February 2019
 	By Nicholas Taylor
 	Modified 19 February 2019
 	By Nicholas Taylor
 
 	Tutorial pending
 	http://stackr.ca

	Edit this line to increase the incoming software serial buffer. 

 	/usr/share/arduino/libraries/SoftwareSerial
 	SoftwareSerial.h
 	#define _SS_MAX_RX_BUFF 160 // RX buffer size
 
 */

#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 8

// include the GSM library

OneWire oneWire(ONE_WIRE_BUS);
/********************************************************************/
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

char nom_from[12];

char sms_serial_buffer[24]; //30>22
char subject[10];
char instruction[5];

int sms_available_bytes = 0;

const byte sms_serial_buffer_length = sizeof(sms_serial_buffer) - 1;

char sms_whitelist[4][12] = {
  "XXXXXXXXXX",
  "XXXXXXXXXX",
  "XXXXXXXXXX",
  "XXXXXXXXXX"};
  
const byte PROGMEM num_whitelist = sizeof(sms_whitelist) / sizeof(sms_whitelist[0]);

char search_for[11];

byte sms_pointer_write = 0;
byte sms_pointer_read = 0;

byte sms_budget = 0;
const byte PROGMEM sms_budget_max = 1;

byte sms_quota = 0;
const byte PROGMEM sms_quota_max = 20; // Allow user to send up to N messages a minute

byte priority_quota = 0;
const byte PROGMEM priority_quota_max = 1; // Allow user to send up to N priority messages per total bar count

byte stack_quota = 0;
const byte PROGMEM stack_quota_max = 2; // Allow user to send up to N priority messages per total bar count

byte display_budget = 0;
const byte PROGMEM display_budget_max = 1;

// Establish ticking clock
byte ticks = 0;
const byte PROGMEM tick_limit = 3;
byte bar = 0 ; 
const byte PROGMEM bar_limit = 80 ;

boolean flag_thing = false;

char response[60]; //80

char agent[16];
char prior_agent[16];

unsigned long current_millis = millis(); 
unsigned long tick_millis = current_millis;
//unsigned long split_millis = millis();

const int PROGMEM tick_interval = 100; // Crank up the tick

const byte PROGMEM screenWidth = 16;
const byte PROGMEM screenHeight = 2;

byte line0_index = 0;
byte line1_index = 0;

boolean flag_priority = false;
boolean flag_cron = false;

// Calculate Pitch, Roll and Yaw
float pitch = 0;
float roll = 0;
float yaw = 0;

float heading;

double fXg, fYg, fZg; 

void setup()
{

  // set up the serial console
  Serial.begin(4800);
  // Set up the connection to the modem
  portSMS.begin(9600);

  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP085 sensor, check wiring!"));
    while (1) {
    }
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

  lcd.begin(16,2);
  lcd.backlight();//Power on the back light

  button.attachPress(doPress);

  sensors.begin(); 

  strcpy(nom_from, sms_whitelist[0]);
  strcpy(agent, "start");
  strcpy(instruction, "+"); 

  doBudget(); // Plus for stack
  doStart();
  doMessage();

  strcpy(nom_from, sms_whitelist[0]);
  strcpy(agent, "kaiju");
  strcpy(instruction, "+"); 

  doBudget(); // Plus for stack
  doKaiju();
  doMessage();
  
  strcpy(nom_from, "NULL");

  strcpy(nom_from, sms_whitelist[0]);
  doTick();

}

void loop()
{ 
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

*/

  if ((unsigned long)(current_millis - tick_millis) >= tick_interval) {
    doTick();
    tick_millis = current_millis;
  }

}

void doBuffer()
{
  Serial.print(F("Agent \"Buffer\" says "));
  // Just prints it to Serial monitor for now.
  Serial.print(F("sms_serial_buffer [ "));

  for(int i=0; i<sms_serial_buffer_length; i++)
  {
    int fp = (sms_pointer_write - sms_serial_buffer_length + i) % (sms_serial_buffer_length);

    if (!((sms_serial_buffer[fp] == 0xD) or (sms_serial_buffer[fp] == 0xA))) {
      Serial.print(sms_serial_buffer[fp]);
    }
  }

  Serial.print(F(" ] "));  

  char* thisString = dtostrf(sms_pointer_write, 4, 0, charBuf);
  Serial.print(F(" sms_pointer_write "));
  Serial.print(deblank(thisString));

  thisString = dtostrf(sms_pointer_read, 4, 0, charBuf);
  Serial.print(F(" sms_pointer_read "));
  Serial.print(deblank(thisString));  

  thisString = dtostrf(sms_available_bytes, 4, 0, charBuf);
  Serial.print(F(" sms_available_bytes "));
  Serial.print(deblank(thisString)); 
  Serial.print(F("/"));
  Serial.print(_SS_MAX_RX_BUFF);
  Serial.println();


}

// Private function: from http://arduino.cc/playground/Code/AvailableMemory  
int doMemory () {
  extern int __heap_start, *__brkval; 
  int v; 
  //int memory;
  int memory = (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);

  strcpy(response, ""); 

  dtostrf(memory, 5, 0, charBuf);

  strcat(response, deblank(charBuf));
  strcat(response, " bytes available");
  
  strcpy(agent, "memory");
  return memory;
}


void doStart()
{
  portSMS.print(F("AT"));
  portSMS.write(0x0D);  // Carriage Return in Hex
  portSMS.write(0x0A);  // Line feed in Hex

  doOK();

  portSMS.print(F("AT+CMGF=1"));
  portSMS.write(0x0D);  // Carriage Return in Hex
  portSMS.write(0x0A);  // Line feed in Hex
  //Serial.println("Sending SMS");
  doOK();
  
  //  delay(1500);
  strcpy(response, "Started.");

  if (strstr(agent, "start")) {
    doSMS();
  }

  strcpy(agent, "start");

}

void doKaiju()
{
  doAccelerometer();
  if (bar == 0) {strcpy(response, "Hello.");}
  
  if (strstr(agent, "kaiju")) {
    doSMS();
  }
  strcpy(agent, "kaiju");
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
    doSMS();
  }
  
  strcpy(agent, "engine");
}

void doCockpit()
{
  sensors.requestTemperatures();
  float temperature = sensors.getTempCByIndex(0);

  strcpy(response, ""); 

  dtostrf(temperature, 5, 1, charBuf);

  strcat(response, deblank(charBuf));
  strcat(response, "C");      

  float roll_degrees = roll * 180 / M_PI;
  dtostrf(roll_degrees, 4, 0, charBuf);
  strcat(response, " roll ");
  strcat(response, deblank(charBuf));
  strcat(response, "");

  float heading_degrees = heading * 180/M_PI;
  
  char* thisString = dtostrf(heading_degrees, 4, 0, charBuf);
  strcat(response, " compass ");
  strcat(response, deblank(thisString));
  strcat(response, " mag");

  if (strstr(agent,"cockpit")) {
    doSMS();
  } 
  
  strcpy(agent, "cockpit");
}

void doRoll()
{
  strcpy(response, "");      
  dtostrf(roll * 180 / M_PI, 4, 0, charBuf);
  strcat(response, deblank(charBuf));
  strcat(response, " degrees ");
  
  strcpy(agent, "roll");
}

void doPitch()
{
  strcpy(response, "");      
  dtostrf(pitch * 180 / M_PI, 4, 0, charBuf);
  strcat(response, deblank(charBuf));
  strcat(response, " degrees");
  
  strcpy(agent, "roll");
}

void doCabin()
{
  float temperature = bmp.readTemperature();

  strcpy(response, "");      
  dtostrf(temperature, 5, 1, charBuf);
  strcat(response, deblank(charBuf));
  strcat(response, "C");
  
  if (strstr(agent,"cabin")) {
    doSMS();
  }
  strcpy(agent, "cabin");
}

boolean doPriority()
{
  strcpy(agent, "priority");

  if ( doBilge()  ) {  
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

  if (sms_budget <= 0) {
    return;
  }

  if (sms_quota >= sms_quota_max) {
    return;
  }
  
  boolean match = false;
  for (int i = 0; i < num_whitelist; i++) {
    if (strstr(nom_from, sms_whitelist[i])) {match = true;}
  }
  if (!match) {return;}
  
  if ((stack_quota >= stack_quota_max) and (strstr(nom_from, sms_whitelist[1]))) {
    return;
  }

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

  delay(5000); // Might be able to reduce this.
  // Blocks for 5 seconds.

  // At this point an SMS has been sent.
  // Debit the thing account.
  strcpy(instruction, "-");
  doBudget();

  doQuota();

  Serial.print(F("Agent \"SMS\" says, \""));
  Serial.print(response);
  Serial.print(F("\""));
  Serial.println();

  strcpy(response, "");     
  strcat(response, "Sent.");

  // Turn off priority?  
  flag_priority = false;
  
  strcpy(agent, "sms"); 
  strcpy(nom_from, NULL); 

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
    doTesla();
    break;
  case 4:
    doCabin();
    break;
  case 5:
    doAccelerometer();
    break;
  case 6:
    doCompass();
    break;
  case 7:
  case 8:
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
    doClocktime();
    break;   
  case 21:
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

void doScience()
{
  doAccelerometer();
  /*
  doTesla();
  
  if (strstr(agent,"science")) {
    doSMS();
  }
  
  strcpy(agent, "science");
*/
}


void doTest() {
}

void doTemperature()
{
  float temperature = bmp.readTemperature();
  dtostrf(temperature, 7, 2, charBuf);
  strcpy(response, "");
  strcat(response, deblank(charBuf));
  strcat(response, "C");
  
  strcpy(agent, "temperature"); 
}

void doWeather()
{
  sensors.requestTemperatures();
  float number = sensors.getTempCByIndex(0);

  dtostrf(number, 5, 1, charBuf);
  strcpy(response, "");
  strcat(response, deblank(charBuf));
  strcat(response, "C ");

  number = bmp.readPressure();

  dtostrf(number, 7, 0, charBuf);
  strcat(response, deblank(charBuf));
  strcat(response, "Pa " );
  
  if (strstr(agent,"weather")) {
    doSMS();
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

  //Low Pass Filter
  fXg = Xg * alpha + (fXg * (1.0 - alpha));
  fYg = Yg * alpha + (fYg * (1.0 - alpha));
  fZg = Zg * alpha + (fZg * (1.0 - alpha));
  
  //Roll & Pitch Equations
  roll  = atan2(fYg, fZg);
  //roll  = atan2(fYg, -fZg);
  pitch = atan2(fXg, sqrt(fYg*fYg + fZg*fZg));
  
  yaw = heading + PI; // Does this belong here.

 /*
  if (yaw < 0) { 
    yaw += 2 * PI; 
  }
  if (yaw > 2 * PI) { 
    yaw -= 2 * PI; 
  }
*/
  
  strcpy(response, ""); 

  char* thisString = dtostrf(Xg, 5, 3, charBuf);
  strcat(response, thisString);

  strcat(response, " ");

  thisString = dtostrf(Yg, 5, 3, charBuf);
  strcat(response, thisString);

  strcat(response, " ");

  thisString = dtostrf(Zg, 5, 3, charBuf);
  strcat(response, thisString);

  strcat(response, " ");

  thisString = dtostrf(roll*180.0/M_PI, 5, 3, charBuf);
  strcat(response, thisString);

  strcat(response, " ");

  thisString = dtostrf(pitch*180.0/M_PI, 5, 3, charBuf);
  strcat(response, thisString); 
 /* 
  strcat(response, " ");

  thisString = dtostrf(yaw*180.0/M_PI, 5, 3, charBuf);
  strcat(response, thisString); 
 */ 
  strcpy(agent, "accelerometer");
  
}

void doPressure()
{
  float pressure = bmp.readPressure();
  dtostrf(pressure, 7, 0, charBuf);
  strcpy(response, "");
  strcat(response, deblank(charBuf));
  strcat(response, "Pa");
  
  strcpy(agent, "pressure");
}

void doAgents()
{
  strcpy(response, "Ping / Weather / Cockpit / Cabin / Bilge / Crow"); 

  if (strstr(agent,"agents")) {
    doSMS();
  } 
  strcpy(agent, "agents");
}

void doMessage()
{
  doSerial();   
  doDisplay();  
  doSMS();   
}

void doSubject()
{
 strcpy(subject, "Hello."); 
  
}

void doCrow()
{
  strcpy(response, "Awk.");

 
  if (strstr(agent,"crow")) {
    doSMS();
  } 
  strcpy(agent, "crow");
  
}
/*
void doPigeon()
{
  strcpy(response, "PIGEON | ");
  strcat(response, "Coo.");

 
  if (strstr(agent,"pigeon")) {
    doSMS();
  } 
  strcpy(agent, "pigeon");
  
}
*/
/*
void doSeagull()
{
  strcpy(response, "SEAGULL | ");
  strcat(response, "Squawk.");

 
  if (strstr(agent,"seagull")) {
    doSMS();
  } 
  strcpy(agent, "seagull");
  
}
*/
void doSerial()
{   
  Serial.println();

  Serial.print(millis());
  Serial.print(F("ms "));

  Serial.print(F("sms_quota "));
  Serial.print(sms_quota);
  Serial.print(F("/"));
  Serial.print(sms_quota_max);

  Serial.print(F(" sms_budget "));
  Serial.print(sms_budget);
  
  Serial.print(F(" sms_available_bytes "));
  Serial.print(sms_available_bytes);
  
  Serial.println();

  Serial.print(F("display_budget "));
  Serial.print(display_budget);

  Serial.print(F(" flag_priority "));
  Serial.print(flag_priority);
  
  Serial.println();

  Serial.print(F("bar "));
  Serial.print(bar);

  Serial.print(F(" subject "));
  Serial.print(subject);

  Serial.print(F(" nom_from "));
  Serial.print(nom_from);

  Serial.print(F(" agent "));
  Serial.print(agent);

  Serial.println();

  doBuffer();

  Serial.println();

  Serial.print(F(""));
  Serial.print(response);

  Serial.println();

  Serial.println();   
}

void doTesla()
{
  MagnetometerScaled scaled = compass.readScaledAxis(); //

  //float milli_gauss = sqrt(scaled.XAxis * scaled.XAxis + scaled.YAxis * scaled.YAxis + scaled.ZAxis * scaled.ZAxis);
  float nano_tesla = sqrt(scaled.XAxis * scaled.XAxis + scaled.YAxis * scaled.YAxis + scaled.ZAxis * scaled.ZAxis) / (10000/(1000));

  char* thisString = dtostrf(nano_tesla, 9, 3, charBuf);
  strcpy(response, "");
  strcat(response, deblank(thisString));
  strcat(response, "uT");

  if (nano_tesla >= 80) {  
    flag_priority = true; 
  }
  
    strcpy(agent, "tesla");
}

void doCompass()
{
  // Retrived the scaled values from the compass (scaled to the configured scale).
  MagnetometerScaled scaled = compass.readScaledAxis(); //

  float cosRoll = cos(roll);
  float sinRoll = sin(roll);
  float cosPitch = cos(pitch);
  float sinPitch = sin(pitch); 

  // Tilt compensation
  // Test
  float Xh = scaled.XAxis * cosPitch + scaled.ZAxis * sinPitch; //XAxis
  float Yh = scaled.XAxis * sinRoll * sinPitch + scaled.YAxis * cosRoll - scaled.ZAxis * sinRoll * cosPitch; //YAxis
  

  heading = atan2(Yh, Xh);
  
  if (heading < 0) { 
    heading += 2 * PI; 
  }
  if (heading > 2 * PI) { 
    heading -= 2 * PI; 
  }

  float heading_degrees = heading * 180/M_PI;

  char* thisString = dtostrf(heading_degrees, 4, 0, charBuf);
  strcpy(response, "");
  strcat(response, deblank(thisString));
  strcat(response, " MAGNETIC");

  float heading_true = (int (heading_degrees + 16.09) % 359);

  thisString = dtostrf(heading_true, 4, 0, charBuf);
  strcat(response, " ");
  strcat(response, deblank(thisString));
  strcat(response, " TRUE");
  
  strcpy(agent, "compass");
}

boolean doBudget()
{
  Serial.print(F("Agent \"Budget\" says, \""));
  Serial.print(sms_budget);
  Serial.print(F("/"));
  Serial.print(sms_budget_max);
  Serial.print(F("\""));
  Serial.println();
  
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
  Serial.print(F("Agent \"Quota\" says, \""));
  Serial.print(sms_quota);
  Serial.print(F("/"));
  Serial.print(sms_quota_max);
  Serial.print(F("\""));
  Serial.println();

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
  strcat(response, "Button pressed.");
  strcpy(agent, "press");
}

/*
void doSelect()
{
  strcpy(response, "SELECT | ");
  strcat(response, "Selected.");
}

void doSignal(int signal_number)
{

}
*/
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
  Serial.print(F("Agent \"Flag\" says, \""));
  strcpy(response, "");

  flag_thing = doOK();

  if (flag_thing) {
    strcat(response, "Red.");
    Serial.print(F("Red."));    
  } 
  else {
    strcat(response, "Green.");
    Serial.print(F("Green"));
  }

  Serial.print(F("\""));
  Serial.println();

  if (strstr(agent,"flag")) {
    doSMS();
  }
  
  strcpy(agent, "flag");

}

/*
void doProgram(int number) {
}
*/
boolean randomBoolean()
{
  long randNumber = random(0,255);
  //Serial.print(randNumber);
  //Serial.println();
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

  // String splitting on vertical bar.
  //line[0] = subStr(response, "|", 1);
  //line[1] = subStr(response, "|", 2);
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
  
  Serial.print(F("Agent \"Tick\" counts "));
  Serial.print(ticks);
  Serial.print(F("."));
  Serial.println();
  
  ticks++;

  char* thisString = dtostrf(bar, 4, 0, charBuf);
  strcpy(response, "");
  strcat(response, "Ticked.");

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
  Serial.print(F("Agent \"Ping\" says, \"Hello.\""));
  Serial.print(agent);
  Serial.println();

  strcpy(response, "");
  strcat(response, "Pong.");

  if (strstr(agent,"ping")) {
    doSMS();
  }  

  strcpy(agent, "ping");
}

void doPong()
{
  strcpy(response, "");
  strcat(response, "Ping.");
 
   strcpy(agent, "pong"); 
}

void doClocktime()
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

  strcpy(agent, "clocktime"); 
}

void doBar()
{
  bar++;

  if (bar>bar_limit) {  
    bar = 0;
  }

  dtostrf(bar, 5, 0, charBuf);
  strcpy(response, "");

  strcat(response, "BAR | ");
  strcat(response, deblank(charBuf));


  doCron(); // Check the clock each and every bar.  Minimum.

  strcpy(prior_agent, agent);
  strcpy(agent, "bar");
}

boolean doBilge()
{
  int bl = false;
  // Vessel tailoring.
  int max = 935;
  int max_measurement = 350 + 75;
  int min = 595;
  int min_measurement = 75;

  // Read the sensor.  
  int sensorValue = analogRead(A0);
  float depth = min_measurement + float (max_measurement - min_measurement) * (float (sensorValue - min) / float(max-min));

  dtostrf(depth, 5, 0, charBuf);
  if (depth < 0) {
    strcpy(charBuf, "<75");
  } 

  strcpy(response, "");
  if (depth >= 400) {  
    flag_priority = true;
    bl = true;
  }  
  strcat(response, "");
  strcat(response, deblank(charBuf));
  strcat(response, "mm");

  if (strstr(agent,"bilge")) {
    doSMS();
  } 

  strcpy(agent, "bilge");
  return bl;
}

void doForget()
{
  delay(1500);

  portSMS.println("AT+CMGD=0,4");

  strcpy(response, "");     
  strcat(response, "All messages forgotten.");
  
  strcpy(agent, "forget");
}

void doHayes()
{
  if (strstr(agent,"AT")) {
    strcpy(response, "OK");
  }

  if (strstr(agent,"ATI")) {
    strcpy(response, "This is a Thing.");
  }

  if (strstr(agent,">")) {
    portSMS.print(response);  //SMS body
    delay(500);
    portSMS.write(0x1A);  // sends ctrl+z end of message
    // portSMS.write(0x0D);  // Carriage Return in Hex
    // portSMS.write(0x0A);  // Line feed in Hex
    portSMS.println();
    delay(5000);
  }

  if (strstr(agent,"+CMTI")) {
    int number = 0;
    int n = 0;
    int fp = 0;
    strcpy(response, "SMS received.");
    strcpy(instruction, "+");
    doBudget();

    //Listen for numbers
    number = doNumber();

    portSMS.print(F("AT+CMGR="));
    portSMS.print(number);
    portSMS.write(0x0D);  // Carriage Return in Hex
    //portSMS.write(0x0A);  // Line feed in Hex

    if (isAgent()) {
      //Serial.println(F("HAYES"));
    }        
  }

  if (strstr(agent,"+SMS FULL")) {
    strcpy(response, "SMS Full. PRIORITY.");
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
  while ((unsigned long)(millis() - sms_millis) <= 10000) { // Listen to the buffer 10000
    doHear();
    //doRead(); // Listen for another char
    fp = (fp + 1) % (sms_serial_buffer_length);

    Serial.print(F("sms_serial_buffer["));
    Serial.print(fp);    
    Serial.print(F("] = "));
    Serial.print(sms_serial_buffer[fp]);
    Serial.println();

    if (isDigit(sms_serial_buffer[fp])){

      flag = true;

      int digit = sms_serial_buffer[fp] - '0';
      number = 10 * number + digit ;
      //++n;
    }

    if ((flag == true) and ( (isSpace(sms_serial_buffer[fp])) or (sms_serial_buffer[fp]== 88))) {
      return number;
    }
  }

  return NULL;
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
    if (doListen()) {
      doRespond(); 
    }

    strcpy(search_for, "+SMS FULL");
    if (doListen()) {
      doRespond(); 
    }

    strcpy(search_for, "ping");
    if (doListen()) {
      doRespond();
      return true;
    }

    strcpy(search_for, "bilge");
    if (doListen()) {
      doRespond();
      return true;  
    }    
    
    strcpy(search_for, "weather");
    if (doListen()) {
      doRespond();
      return true;  
    }  

    strcpy(search_for, "cockpit");
    if (doListen()) {
      doRespond();
      return true;  
    }   

    strcpy(search_for, "cabin");
    if (doListen()) {
      doRespond();
      return true;  
    }   

    strcpy(search_for, "crow");
    if (doListen()) {
      doRespond();
      return true;  
    } 
       
    strcpy(search_for, "engine");
    if (doListen()) {
      doRespond();
      return true;  
    } 

/*   
    strcpy(search_for, "science");
    if (doListen()) {
      doRespond();
      return true;  
    }
*/

/*    
    strcpy(search_for, "pigeon");
    if (doListen()) {
      doRespond();
      return true;  
    } 
    
    strcpy(search_for, "seagull");
    if (doListen()) {
      doRespond();
      return true;  
    } 
*/   
    strcpy(search_for, "flag");
    if (doListen()) {
      doRespond();
      return true;  
    }  

    strcpy(search_for, "kaiju");    
    if (doListen()) {
      doRespond();
      return true;  
    } 
    
/*   
    strcpy(search_for, "agents");    
    if (doListen()) {
      doRespond();
      return true;  
    } 
*/

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
  
  if ( (minute() == 2) and (flag_cron == false)) {
    Serial.print(F("Agent \"Cron\" triggered."));
    strcpy(instruction, "+"); 
    doBudget();
    strcpy(nom_from,sms_whitelist[1]);
    doStack();
    flag_cron = true;
  }

  if ( (minute() != 2) and (flag_cron == true)) {
    flag_cron = false;
  }
}

void doStack()
{ 
  Serial.print(F("Agent \"Stack\" called."));
   
  if (strstr(nom_from, sms_whitelist[1])) {
    ++stack_quota;
    if (stack_quota > stack_quota_max) {
      stack_quota = stack_quota_max;
      return;
    }   
  }
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
    if (match == true) { 

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

  for (int i = 0; i < num_whitelist; i++) {
    if (strstr(agent,sms_whitelist[i])) {
      strcpy(nom_from, sms_whitelist[i]);
    }
  }

  if (strstr(nom_from,sms_whitelist[1])) {
    doStack();
  }

  if (strstr(agent,"ping")) {
    doPing();
  }

  if (strstr(agent,"weather")) {
    doWeather();
  }
  
  if (strstr(agent,"cockpit")) {
    doCockpit();
  }

  if (strstr(agent,"cabin")) {
    doCabin();
  }

  if (strstr(agent,"bilge")) {
    doBilge();
  }
  
  if (strstr(agent,"crow")) {
    doCrow();
  }
/*  
  if (strstr(agent,"pigeon")) {
    doPigeon();
  }

  if (strstr(agent,"seagull")) {
    doSeagull();
  }
*/  
  if (strstr(agent,"kaiju")) {
    doKaiju();
  }
  
  if (strstr(agent,"flag")) {
    doFlag();
  }
  
  if (strstr(agent,"engine")) {
    doEngine();
  }
  
/*  
  if (strstr(agent,"agents")) {
    doAgents();
  }
*/


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
