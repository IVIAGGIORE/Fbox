
// Imports
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include "./TinyGPS.h"                 // Use local version of this library
#include "./CorytoDefinitions.h"
#include <NewPing.h>

// GPS
TinyGPS GpsParser;

// Lid
Servo lidServo;
CoolerLid lidState = CLOSE;
GeoLoc PhoneLoc;
unsigned long startSonar;

// Master Enable
bool enabled = false;
bool IsDriveA = false;
bool IsDriveB = false;

// Serial components
SoftwareSerial BluetoothSerial(BLUETOOTH_TX_PIN, BLUETOOTH_RX_PIN);
SoftwareSerial GpsSerialPort(GPS_TX_PIN, 255);            // TXD to digital pin 6

/* Compass */
Adafruit_HMC5883_Unified Compass = Adafruit_HMC5883_Unified(12345);

NewPing Sonar(SONAR_TRIGGER_PIN, SONAR_ECHO_PIN, SONAR_MAX_DISTANCE);

GeoLoc checkGPS() {
  Serial.println("Reading onboard GPS: ");

  bool newdata = false;
  unsigned long start = millis();
  while (millis() - start < GPS_UPDATE_INTERVAL) {
    if (feedgps())
      newdata = true;
  }
 
  if (newdata) {
    return gpsdump(GpsParser);
  }

  GeoLoc coolerLoc;
  coolerLoc.lat = 0.0;
  coolerLoc.lon = 0.0;
  
  return coolerLoc;
}

// Get and process GPS data
GeoLoc gpsdump(TinyGPS &gpsParser) {
  float flat, flon;
  unsigned long age;
  
  gpsParser.f_get_position(&flat, &flon, &age);

  GeoLoc coolerLoc;
  coolerLoc.lat = flat;
  coolerLoc.lon = flon;

  Serial.print(coolerLoc.lat, 7); Serial.print(", "); Serial.println(coolerLoc.lon, 7);

  return coolerLoc;
}

// Feed data as it becomes available 
bool feedgps() {
  unsigned long start = millis();
  while (GpsSerialPort.available() && (millis() - start < GPS_UPDATE_INTERVAL)) {
    if (GpsParser.encode(GpsSerialPort.read()))
      return true;
  }
  return false;
}

// Lid Hook
void CommandLidState(CoolerLid ls) {
  switch (ls) {
    case OPEN:
      if (lidState != ls) setServo(SERVO_LID_OPEN);
      break;
    case CLOSE:
      if (lidState != ls) setServo(SERVO_LID_CLOSE);
      break;
  }
  lidState = ls;  
}

// Killswitch Hook
void CommandEnable(void) {
  
  enabled = !enabled;
  
  //Stop the wheels
  stop();
}

// GPS Streaming Hook
void CommandDriveTo(GeoLoc &phoneLoc) {
  
  Serial.println("Received remote GPS: ");
  
  // Print 7 decimal places for Lat
  Serial.print(phoneLoc.lat, 7); Serial.print(", "); Serial.println(phoneLoc.lon, 7);

  driveTo(phoneLoc, GPS_STREAM_TIMEOUT);
}

// Terminal Hook
void CommandDriveToPoints(String points) {
  
  Serial.print("Received Text: ");
  Serial.println(points);

  String rawInput(points);
  int colonIndex;
  int commaIndex;
  
  do {
    commaIndex = rawInput.indexOf(',');
    colonIndex = rawInput.indexOf(':');
    
    if (commaIndex != -1) {
      String latStr = rawInput.substring(0, commaIndex);
      String lonStr = rawInput.substring(commaIndex+1);

      if (colonIndex != -1) {
         lonStr = rawInput.substring(commaIndex+1, colonIndex);
      }
    
      float lat = latStr.toFloat();
      float lon = lonStr.toFloat();
    
      if (lat != 0 && lon != 0) {
        GeoLoc waypoint;
        waypoint.lat = lat;
        waypoint.lon = lon;
    
        Serial.print("Waypoint found: "); Serial.print(lat); Serial.println(lon);
        driveTo(waypoint, GPS_WAYPOINT_TIMEOUT);
      }
    }
    
    rawInput = rawInput.substring(colonIndex + 1);
    
  } while (colonIndex != -1);
}

void displayCompassDetails(void)
{
  sensor_t sensor;
  Compass.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

#ifndef DEGTORAD
#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
#endif

float geoBearing(struct GeoLoc &a, struct GeoLoc &b) {
  float y = sin(b.lon-a.lon) * cos(b.lat);
  float x = cos(a.lat)*sin(b.lat) - sin(a.lat)*cos(b.lat)*cos(b.lon-a.lon);
  return atan2(y, x) * RADTODEG;
}

float geoDistance(struct GeoLoc &a, struct GeoLoc &b) {
  const float R = 6371000; // km
  float p1 = a.lat * DEGTORAD;
  float p2 = b.lat * DEGTORAD;
  float dp = (b.lat-a.lat) * DEGTORAD;
  float dl = (b.lon-a.lon) * DEGTORAD;

  float x = sin(dp/2) * sin(dp/2) + cos(p1) * cos(p2) * sin(dl/2) * sin(dl/2);
  float y = 2 * atan2(sqrt(x), sqrt(1-x));

  return R * y;
}

float geoHeading() {
  /* Get a new sensor event */ 
  sensors_event_t event; 
  Compass.getEvent(&event);

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);

  // Offset
  heading -= DECLINATION_ANGLE;
  heading -= COMPASS_OFFSET;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 

  // Map to -180 - 180
  while (headingDegrees < -180) headingDegrees += 360;
  while (headingDegrees >  180) headingDegrees -= 360;

  return headingDegrees;
}

void setServo(int pos) {
  lidServo.attach(SERVO_PIN);
  lidServo.write(pos);
  delay(2000);
  lidServo.detach();
}

void setSpeedMotorA(int speed) {
  digitalWrite(MOTOR_A_IN_1_PIN, LOW);
  digitalWrite(MOTOR_A_IN_2_PIN, HIGH);
  
  // set speed to 200 out of possible range 0~255
  analogWrite(MOTOR_A_EN_PIN, speed + MOTOR_A_OFFSET);

  IsDriveA = speed > 0;
}

void setSpeedMotorB(int speed) {
  digitalWrite(MOTOR_B_IN_1_PIN, LOW);
  digitalWrite(MOTOR_B_IN_2_PIN, HIGH);
  
  // set speed to 200 out of possible range 0~255
  analogWrite(MOTOR_B_EN_PIN, speed + MOTOR_B_OFFSET);

  IsDriveB = speed > 0;
}

void stop() {
  // now turn off motors
  digitalWrite(MOTOR_A_IN_1_PIN, LOW);
  digitalWrite(MOTOR_A_IN_2_PIN, LOW);  
  digitalWrite(MOTOR_B_IN_1_PIN, LOW);
  digitalWrite(MOTOR_B_IN_2_PIN, LOW);
  IsDriveA = false;
  IsDriveB = false;
}

void drive(int distance, float turn) {
  int fullSpeed = 250;
  int stopSpeed = 0;

  // drive to location
  int s = fullSpeed;
  if ( distance < 8 ) {
    int wouldBeSpeed = s - stopSpeed;
    wouldBeSpeed *= distance / 8.0f;
    s = stopSpeed + wouldBeSpeed;
  }
  
  int autoThrottle = constrain(s, stopSpeed, fullSpeed);
  autoThrottle = 230;

  float t = turn;
  while (t < -180) t += 360;
  while (t >  180) t -= 360;
  
  Serial.print("turn: ");
  Serial.println(t);
  Serial.print("original: ");
  Serial.println(turn);
  
  float t_modifier = (180.0 - abs(t)) / 180.0;
  float autoSteerA = 1;
  float autoSteerB = 1;

  if (t < 0) {
    autoSteerB = t_modifier;
  } else if (t > 0){
    autoSteerA = t_modifier;
  }

  Serial.print("steerA: "); Serial.println(autoSteerA);
  Serial.print("steerB: "); Serial.println(autoSteerB);

  int speedA = (int) (((float) autoThrottle) * autoSteerA);
  int speedB = (int) (((float) autoThrottle) * autoSteerB);
  
  setSpeedMotorA(speedA);
  setSpeedMotorB(speedB);
}

void driveTo(struct GeoLoc &loc, int timeout) {
  
  GpsSerialPort.listen();
  GeoLoc coolerLoc = checkGPS();
  BluetoothSerial.listen();

  if (coolerLoc.lat != 0 && coolerLoc.lon != 0 && loc.lat != 0 && loc.lon != 0 && enabled) {
    float d = 0;
    //Start move loop here
    do {
      GpsSerialPort.listen();
      coolerLoc = checkGPS();
      BluetoothSerial.listen();
      
      d = geoDistance(coolerLoc, loc);
      float t = geoBearing(coolerLoc, loc) - geoHeading();
      
      Serial.print("Distance: ");
      Serial.println(geoDistance(coolerLoc, loc));
    
      Serial.print("Bearing: ");
      Serial.println(geoBearing(coolerLoc, loc));

      Serial.print("heading: ");
      Serial.println(geoHeading());
      
      drive(d, t);
      timeout -= 1;
    } while (d > 3.0 && enabled && timeout>0);

    stop();
  }
}

void setupCompass() {
   /* Initialise the compass */
  if(!Compass.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
  
  /* Display some basic information on this sensor */
  displayCompassDetails();
}

void setup()
{
  //Debugging via serial
  Serial.begin(9600);
  
  // Compass
  setupCompass();

  // Motor pins
  pinMode(MOTOR_A_EN_PIN, OUTPUT);
  pinMode(MOTOR_B_EN_PIN, OUTPUT);
  pinMode(MOTOR_A_IN_1_PIN, OUTPUT);
  pinMode(MOTOR_A_IN_2_PIN, OUTPUT);
  pinMode(MOTOR_B_IN_1_PIN, OUTPUT);
  pinMode(MOTOR_B_IN_2_PIN, OUTPUT);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  //GPS
  GpsSerialPort.begin(9600);

  //Bluetooth
  BluetoothSerial.begin(9600);
  /*BluetoothSerial.println("AT");
  delay(500);
  BluetoothSerial.println("AT+NAME=FBox");
  delay(500);
  BluetoothSerial.println("AT+UART?");
  delay(500);*/
  
  startSonar = millis();
}

// Testing
void testDriveNorth() {
  float heading = geoHeading();
  int testDist = 10;
  Serial.println(heading);
  
  while(!(heading < 5 && heading > -5)) {
    drive(testDist, heading);
    heading = geoHeading();
    Serial.println(heading);
    delay(500);
  }
  
  stop();
}

void loop()
{

  if (IsDriveA == false && IsDriveB == false && (millis() - startSonar > 250))
  {
    int space = Sonar.ping_cm();
    if (space == 0 ) space = SONAR_MAX_DISTANCE;
    if (space < 40)
    {
      CommandLidState(OPEN);
    }
    else
    {
      CommandLidState(CLOSE);
    }
    startSonar = millis();
  }

  enabled = false;
  if (BluetoothSerial.available()){
    int byteReadCount = 0;
    while (byteReadCount < 4) {
      if (BluetoothSerial.available()){
        u.b[byteReadCount] = BluetoothSerial.read();
        byteReadCount++;
      }
    }
    PhoneLoc.lat = u.fval;
    
    byteReadCount = 0;
    while (byteReadCount < 4) {
      if (BluetoothSerial.available()){
        u.b[byteReadCount] = BluetoothSerial.read();     
        byteReadCount++;
      }
    }
    PhoneLoc.lon = u.fval;
    enabled = true;
    CommandDriveTo(PhoneLoc);
  }

}
