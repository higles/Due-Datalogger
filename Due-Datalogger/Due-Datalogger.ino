/** 
  * Due Datalogger for VAST.
  * Year: 2015
  * Joe Higley, Ben Bolton, Sanjar Rahimov
  **/

#include <Adafruit_GPS.h>
  
// LED pins
#define LED_RDY 47  //flight ready led
#define LED_SD  49  //sd card ready led
#define LED_GPS 51  //gps ready led
#define LED_BT  53  //blue-tooth connected led

#define GPS_FIX 8   //gps fix pin

// Hardware serial
#define mySerial Serial3
Adafruit_GPS GPS(&mySerial);

#define GPSECHO true

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

void setup() {
  // Initialize Serial
  Serial.begin(115200);
  
  GPS_Setup();
  
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
  
  LED_Setup();
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

uint32_t timer = millis();
void loop() {
  readGPS();
}


// LED pin setup
void LED_Setup() {
  // Set LED pin modes
  pinMode(LED_RDY, OUTPUT);
  pinMode(LED_SD,  OUTPUT);
  pinMode(LED_GPS, OUTPUT);
  pinMode(LED_BT,  OUTPUT);
  pinMode(GPS_FIX, INPUT);
  digitalWrite(LED_RDY, LOW);
  digitalWrite(LED_SD, LOW);
  digitalWrite(LED_GPS, LOW);
  digitalWrite(LED_BT, LOW);
}

// Setup GPS
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
}

// Reads data from GPS and blinks LED accordingly
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
  
  // if we get a fix, turn off the led
  if (digitalRead(GPS_FIX)) //reads a 1 for no fix
    digitalWrite(LED_GPS, HIGH);
  else 
    digitalWrite(LED_GPS, LOW);
    
}
