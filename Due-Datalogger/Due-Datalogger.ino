/** 
  * Due Datalogger for VAST.
  * Year: 2015
  * Joe Higley, Ben Bolton, Sanjar Rahimov
  **/

#include <Adafruit_GPS.h>
// SD Reader
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <MS5803_I2C.h>

const int chipSelect = 13;
File logfile;
char fName[20];
String message = "";
bool writing = false;
bool SD_Passed = false;
  
// LED pins
#define LED_RDY 47  //flight ready led
#define LED_SD  49  //sd card ready led
#define LED_GPS 51  //gps ready led
#define LED_BT  53  //blue-tooth connected led

// Geiger Counter
#define SIZE 30
unsigned long previous_time;
unsigned long ticks[SIZE];
int current_index;

// Pressure Sensor
MS5803 sensor(ADDRESS_HIGH); // 0x76
float temperature_c, temperature_f;
double pressure_abs, pressure_relative, altitude_delta, pressure_baseline;
double base_altitude = 1655.0;  // Altitude of SparkFun's HQ in Boulder, CO. in (m)

// Solar panel
#define SOLAR 56
float voltage = 0;
float count = 0;

#define GPS_FIX 8   //gps fix pin

// Hardware serial
#define mySerial Serial3
Adafruit_GPS GPS(&mySerial);

#define GPSECHO false

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy


/***********************|
  Setup the datalogger  |
 ***********************/
void setup() {
  // Initialize Serial
  Serial.begin(115200);
  
  // Setup sensor
  Wire.begin();
  sensor.reset();
  sensor.begin();
  
  // Solar Panel setup
  pinMode( SOLAR, INPUT );
  analogReadResolution(13);
  
  LED_Setup();
  SD_Setup();
  GPS_Setup();
  Geiger_Setup();
  
}


#ifdef __AVR__
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

/**********************|
  Not used on the Due  |
 **********************/
void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
#endif //#ifdef__AVR__


/********************|
  Main program loop  |
 ********************/
uint32_t timer = millis();
void loop() {
  // Write only every second
  if (millis() - timer > 1000) {
    timer = millis();
    logfile = SD.open(fName, FILE_WRITE);
    writing = true;
  }
  
  // Read data as often as possible
  // Writes only if writing is true
  readGPS();
  PT_Read();
  Geiger_Read();
  Solar_Read();
  
  // After reading and writing data switch to not writing.
  if (writing) {
    writing = false; //Reset writing flag
    logfile.println();
    logfile.close();
  }
  
  // Update lights
  if (SD_Passed) digitalWrite(LED_SD, LOW);
  else digitalWrite(LED_SD, HIGH);
  
  if (SD_Passed && GPS.fix) digitalWrite(LED_RDY, HIGH);
  else digitalWrite(LED_RDY, LOW);
   
}


/****************|
  LED pin setup  |
 ****************/
void LED_Setup() {
  // Set LED pin modes
  pinMode(LED_RDY, OUTPUT);
  pinMode(LED_SD,  OUTPUT);
  pinMode(LED_GPS, OUTPUT);
  pinMode(LED_BT,  OUTPUT);
  pinMode(GPS_FIX, INPUT);
  
  delay(4000);
  
  digitalWrite(LED_RDY, LOW);
  digitalWrite(LED_SD, LOW);
  digitalWrite(LED_GPS, LOW);
  digitalWrite(LED_BT, LOW);
}


/******************|
  Setup SD reader  |
 ******************/
void SD_Setup() {
  Serial.print("Initializing SD card...");
  // Set chipselect pin to output.
  pinMode(chipSelect, OUTPUT);
  
  // See if the card is present and can be initialized
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // light error LED and don't do anything
    SD_Passed = false;
  }
  else {
    Serial.println("Card initialized");
    SD_Passed = true;
    
    // Store filename in fName
    strcpy(fName, "DatLog00.csv");
    
    // Increment file name postfix if files already exist
    for (int i = 0; i < 100; i++) {
      fName[6] = '0' + i/10;
      fName[7] = '0' + i%10;
      
      if (! SD.exists(fName)) {
        break;
      }
    }
    delay(500);
    // Create file with incremental filename
    logfile = SD.open(fName, FILE_WRITE);
    
    // Check is file was created
    if(!logfile) {
      Serial.print("\r\nCouldn't create ");
      Serial.println(fName);
      SD_Passed = false;
    }
    else { // File was created
      Serial.print("\r\nWriting to ");
      Serial.println(fName);
      Serial.println("\r\nReady!");
      SD_Passed = true;
      
      // Write header line to file
      logfile.print("Time,Date,Fix,Latitude,Longitude,Altitude,");
      logfile.print("Speed (knots),Angle,Satellites,,");
      logfile.print("Temp C, Temp F, P abs (mbar), P rel (mbar), Alt change (m),,");
      logfile.print("Millis Between,Ticks per Second,Ticks per Minute,,");
      logfile.println("Solar Power");
      logfile.close();
    }
  }
}


/************|
  Setup GPS  |
 ************/
void GPS_Setup() {
  // 9600 NMEA is the default baud rate for MTK - some use 4800
  GPS.begin(9600);
  mySerial.begin(9600);
  
  // Turn on RMC (recommended minimum and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // Set update rate to 5 Hz
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  
  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  #ifdef __arm__
    usingInterrupt = false;  //NOTE - we don't want to use interrupts on the Due
  #else
    useInterrupt(true);
  #endif
  
  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
}


/**************************|
  Reads data from GPS and  |
  blinks LED accordingly   |
 **************************/
void readGPS() {
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (writing) {   
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    
    // If the file is available, write to it
    if (logfile) {
      logfile.print(GPS.hour, DEC);    logfile.print(':');
      logfile.print(GPS.minute, DEC);  logfile.print(':');
      logfile.print(GPS.seconds, DEC); logfile.print('.');
      logfile.print(GPS.milliseconds); logfile.print(',');
      
      logfile.print(GPS.day, DEC);     logfile.print('/');
      logfile.print(GPS.month, DEC);   logfile.print("/20");
      logfile.print(GPS.year, DEC);    logfile.print(',');
      
      logfile.print(GPS.fix);          logfile.print(" quality: "); 
      logfile.print(GPS.fixquality);   logfile.print(',');
      SD_Passed = true;
    }
    else SD_Passed = false;
      
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      
      if (logfile) {
        logfile.print(GPS.latitude, 4);  logfile.print(GPS.lat);
        logfile.print(',');
        logfile.print(GPS.longitude, 4); logfile.print(GPS.lon);
        logfile.print(','); 
        logfile.print(GPS.altitude);     logfile.print(',');
        
        logfile.print(GPS.speed);        logfile.print(',');
        logfile.print(GPS.angle);        logfile.print(',');
        logfile.print(GPS.satellites);   logfile.print(",,");
        SD_Passed = true;
      }
      else SD_Passed = false;
    }
    else logfile.print(",,,,,,,");
  }
    
  // Blink as the GPS led blinks
  if (GPS.fix)
    digitalWrite(LED_GPS, LOW);
  else 
    digitalWrite(LED_GPS, HIGH);
    
}


/***********************|
  Geiger counter setup  |
 ***********************/
void Geiger_Setup() {
  previous_time = millis();
 
  for (int i = 0; i < SIZE; i++) {
    ticks[i] = 0;
  }
 
  Serial2.begin(4800); 
}


/***************************|
  Read from geiger counter  |
 ***************************/
void Geiger_Read() {
  
  if (Serial2.available())  {
    char c = Serial2.read();
        Serial.print(c);
    double millis_between;
    double seconds_between;
    double ticks_per_second;
    double ticks_per_minute;
    boolean event = false; // flag for writing events
		
    if (c == '1')  {
      event = true; // geiger event flagged
      unsigned long current_time = millis();
      ticks[current_index] = current_time - previous_time;
      previous_time = current_time;
      Serial.print("time since last: ");
      Serial.println(ticks[current_index]);
			
      current_index++;
      if (current_index == SIZE)  {
	current_index = 0;
      }
			
      millis_between = avg_time();
      Serial.print("millis_between: ");   
      Serial.println(millis_between);
			
      seconds_between = millis_between / 1000;
      ticks_per_second = 1 / seconds_between;
      ticks_per_minute = ticks_per_second * 60;
			
      Serial.print("Ticks per second: ");
      Serial.println(ticks_per_second);
      Serial.print("Ticks per minute: ");
      Serial.println(ticks_per_minute);
			
      Serial.println();
    }
    if(writing && logfile) {
      if(event) { //if event has occurred since last write
        logfile.print(millis_between);   logfile.print(',');
        logfile.print(ticks_per_second); logfile.print(',');
        logfile.print(ticks_per_minute); logfile.print(',,');
        event = false; //reset event flag
      }
      else logfile.print(",,,,"); //don't write if no new event.
    }
  } 
  else if(writing && logfile) {
    logfile.print(",,,,");
  }
}


/************************|
  Calculate the average  |
  time between ticks     |
 ************************/
double avg_time()  {
  double total_time = 0;
  for (int i = 0; i < SIZE; i++)  {
    total_time += ticks[i];
  }
	
  return (total_time / SIZE);
}

/*******************|
  Get Pressure and  |
  Temperature data  |
  from sensor       |
 *******************/
void PT_Read() {
  // To measure to higher degrees of precision use the following sensor settings:
  // ADC_256 
  // ADC_512 
  // ADC_1024
  // ADC_2048
  // ADC_4096
  // Read temperature from the sensor in deg C. This operation takes about 
  temperature_c = sensor.getTemperature(CELSIUS, ADC_2048);
  
  // Read temperature from the sensor in deg F. Converting
  // to Fahrenheit is not internal to the sensor.
  // Additional math is done to convert a Celsius reading.
  temperature_f = sensor.getTemperature(FAHRENHEIT, ADC_2048);
  
  // Read pressure from the sensor in mbar.
  pressure_abs = sensor.getPressure(ADC_4096);
  
  // Let's do something interesting with our data.
  
  // Convert abs pressure with the help of altitude into relative pressure
  // This is used in Weather stations.
  pressure_relative = sealevel(pressure_abs, base_altitude);
  
  // Taking our baseline pressure at the beginning we can find an approximate
  // change in altitude based on the differences in pressure.   
  altitude_delta = altitude(pressure_abs , pressure_baseline);
  
  if (writing) {
    // Report values via UART
    Serial.print("Temperature C = ");
    Serial.println((temperature_f - 42)/1.8);
    logfile.print((temperature_f - 42)/1.8); logfile.print(',');
    
    Serial.print("Temperature F = ");
    Serial.println(temperature_f - 10);
    logfile.print(temperature_f - 10); logfile.print(',');
    
    Serial.print("Pressure abs (mbar)= ");
    Serial.println(pressure_abs);
    logfile.print(pressure_abs); logfile.print(',');
     
    Serial.print("Pressure relative (mbar)= ");
    Serial.println(pressure_relative); 
    logfile.print(pressure_relative); logfile.print(',');
    
    Serial.print("Altitude change (m) = ");
    Serial.println(altitude_delta); 
    logfile.print(altitude_delta); logfile.print(',,');
  }
}

 double sealevel(double P, double A)
// Given a pressure P (mbar) taken at a specific altitude (meters),
// return the equivalent pressure (mbar) at sea level.
// This produces pressure readings that can be used for weather measurements.
{
	return(P/pow(1-(A/44330.0),5.255));
}


double altitude(double P, double P0)
// Given a pressure measurement P (mbar) and the pressure at a baseline P0 (mbar),
// return altitude (meters) above baseline.
{
	return(44330.0*(1-pow(P/P0,1/5.255)));
}

/*******************|
  Get voltage from  |
  the solar panel   |
 *******************/
void Solar_Read() {
   voltage += Getsolar();
   
   if (writing) {
     voltage /= count;
     count = 0;
     Serial.print("Voltage: ");
     Serial.println( voltage );
     voltage = 0;
   }
}

float Getsolar() {
  float v;
  v = (float)analogRead(SOLAR);
  v *= 100.00;
  v /= 1241.00;
  count++;
  return v;
}


