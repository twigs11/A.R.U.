//libraries needed for this code to fuction
#include <TinyGPS++.h>
#include <Adafruit_BNO08x.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <Adafruit_BMP085.h>

#define LAT 32.34346 //////////////////////////// Update DAS Location Here ///////////////////////////////////
#define LON -110.97558 //////////////////////////// Update DAS Location Here ///////////////////////////////////

//Stepper Driver Pin Definitions
#define enPin 7
#define dirPin 6
#define stepPin 5
#define ARRAY_SIZE 1
#define MAX_COORD_LEN 10  // Maximum length of latitude or longitude string

// Stepper Paramters
#define stepsPerRevolution 6400
#define OneRevolution 360


// IMU Parameters
#define BNO08X_RESET -1

#ifdef FAST_MODE
  // Top frequency is reported to be 1000Hz (but freq is somewhat variable)
  sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
  long reportIntervalUs = 2000;
#else
  // Top frequency is about 250Hz but this report is more accurate
  sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
  long reportIntervalUs = 5000;
#endif

// structure definition for IMU
struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;


/// *** GLOBALS *** ///

// User Input Vars
int Arr[1];
char readString[3];
char LatSerial;
char LonSerial;
int CoorValue = 10;
char a;
char b;
char c;
char d;
String readAdjustment;
String readLat;
String readLon;
String readFirst;
String readSecond;
String readThrid;
String readFourth;

// Debug Vars
char WAY_Buffer[40];
int strcount = 0;

// Stepper Motor Vars
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);
int StepperDegrees = 0;
int Adjustment = 0;
int First;
int Second;
int CW = 1;
int CCW = 0;
int WAY;
int n;
int position = 0;
int angle = 0;
float stepPerAngle = 1.8; 
int numstep;

// GPS
TinyGPSPlus gps;
volatile float minutes, seconds;
volatile int degree, secs, mins;
bool loc_valid, alt_valid, time_valid;
uint8_t hr_val, min_val, sec_val;
double lat_val, lng_val, alt_m_val;
unsigned long start = millis();

// IMU
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
double azmth;
const double cRadiusOfEarth = 3963.0; //Radius of earth in miles
const double cFeetToMi = 0.00018939;   // Conversion factor from feet to miles
//Exmaple Point B coordinates
double lat2 = LAT;
double lon2 = LON;
double f1; 
double f2;
double delta_f; 
double delta_g;
double ac;
double ca;
double magneticDeclination = 9.13;
double brng_r;
double brng_d;
double dist_Mi;
double y;
double x;
long autosteps;
double PPR = 6400.0;
double Steps = 0.05625;

//LED
const int RED_PIN = 11;
const int GREEN_PIN = 10;
const int BLUE_PIN = 9;

// Temp Sensor
// Adafruit_BMP085 bmp;

//Packet Headers 
const byte Direction = 1;
const byte Angle_Adjustment = 2;
const byte Common_Angle = 3;
const byte Set_Zero = 4;
const byte Azimuth = 5;
// const byte Temperature = 6;
// const byte LED_RED = 9;

// IMU Functions

void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

//Converts our quaternion angles to Euler by using a formula for each axis
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

//Convert from quaternion angles to Euler angles using a rotation sensor
void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

//Convert from quaternion angles to Euler angles using a gyro sensor
void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

// GPS Functions
static void smartDelay(unsigned long ms) //a delay that during it will check and collect GPS data
{
  start = millis();
  do
  {
    while (Serial3.available())  /* Encode data read from GPS while data is available on serial port */
     gps.encode(Serial3.read());
     /* Encode basically is used to parse the string received by the GPS and to store it in a buffer so that information can be extracted from it */
  } while (millis() - start < ms);
      lat_val, lng_val, alt_m_val;
      hr_val, min_val, sec_val;
      loc_valid, alt_valid, time_valid;
      lat_val = gps.location.lat(); /* Get latitude data */
      loc_valid = gps.location.isValid(); /* Check if valid location data is available */
      lng_val = gps.location.lng(); /* Get longtitude data */
      alt_m_val = gps.altitude.meters();  /* Get altitude data in meters */
      alt_valid = gps.altitude.isValid(); /* Check if valid altitude data is available */
      hr_val = gps.time.hour(); /* Get hour */
      min_val = gps.time.minute();  /* Get minutes */
      sec_val = gps.time.second();  /* Get seconds */
      time_valid = gps.time.isValid();  /* Check if valid time data is available */
}
 
void DegMinSec( double tot_val)   /* Convert data in decimal degrees into degrees minutes seconds form */
{
  degree = (int)tot_val;
  minutes = tot_val - degree;
  seconds = 60 * minutes;
  minutes = (int)seconds;
  mins = (int)minutes;
  seconds = seconds - minutes;
  seconds = 60 * seconds;
  secs = (int)seconds;
}



void setup() {
  // declare pins as output:
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin,OUTPUT);
  // pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200); //baud rate for Serial Monitor and everything included.
  digitalWrite(RED_PIN, HIGH);
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(BLUE_PIN, LOW);
  Serial3.begin(9600); /* Define baud rate for software serial communication  for the gps*/


  Serial.println("Checking for IMU BMO08x startup");

  // Tries to initialize IMU module
  if (!bno08x.begin_I2C()) 
  {
    Serial.println("Failed to find BNO08x chip");//if it isnt connected or wired correctly it will show this message.
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!"); //IMU is working

  setReports(reportType, reportIntervalUs); //sets what data to look for the IMU.

  Serial.println("Reading events");
  delay(100);
  
  stepper.setMaxSpeed(400);
  stepper.setAcceleration(300);

  delay(60000);

Serial3.read();
delay(1000);
Serial3.read();
delay(1000);
Serial3.read();
delay(1000);
Serial3.read();
delay(1000);
Serial3.read();





}

void loop() 
{
 
  //once there is some input, the led will go from red to green to show connection
  digitalWrite(RED_PIN, LOW);
  digitalWrite(GREEN_PIN, HIGH);
  digitalWrite(BLUE_PIN, LOW);

  smartDelay(1000); /* wait will we collect gps info for 5 seconds*/

  
  //Checks and informs user if GPS is working or not, if working, it will show the coordinates of the user's position
  if (!loc_valid)
  {
    Serial.print("Latitude : ");
    Serial.println("*****");
    Serial.print("Longitude : ");
    Serial.println("*****");
  }
  else
  {
    DegMinSec(lat_val);
    Serial.print("Latitude in Decimal Degrees : ");
    Serial.println(lat_val, 6);

    DegMinSec(lng_val); /* Convert the decimal degree value into degrees minutes seconds form */
    Serial.print("Longitude in Decimal Degrees : ");
    Serial.println(lng_val, 6);       
  }
  
  Serial.print(F("  Date/Time: ")); // will also inform the time and date 
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
  
  //now checks to see if imu works and in order to get it working correctly, the imu has to be reset each time its powered up.
  if (bno08x.wasReset()) 
  {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }
  
  if (bno08x.getSensorEvent(&sensorValue)) 
  {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) 
    {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
            break;
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }
    static long last = 0;
    long now = micros();
    Serial.print(now - last);             Serial.print("\t");
    last = now;

    //prints out all of the values of the imu sensors.
    Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
    Serial.print(ypr.yaw);                Serial.print("\t");
    Serial.print(ypr.pitch);              Serial.print("\t");
    Serial.println(ypr.roll);
  }

  // does all the math that will help us find azimuth with such precision 
  f1 = radians(lat_val);
  f2 = radians(lat2);
  delta_f = radians(lat2 - lat_val);
  delta_g = radians(lon2 - lng_val);
  
  
  ac = sin(delta_f / 2) * sin(delta_f / 2) + cos(f1) * cos(f2) * sin(delta_g / 2) * sin(delta_g / 2);
  ca = 2 * atan2(sqrt(ac), sqrt(1 - ac));
  dist_Mi = cRadiusOfEarth * ca;      

  brng_r = atan2( sin(lon2 - lng_val) * cos(lat2), cos(lat_val) * sin(lat2) - sin(lat_val) * cos(lat2) * cos(lon2 - lng_val));

  brng_d = (360.0 - degrees(brng_r)) / 360.0;
  y = sin(delta_g) * cos(f2);
  x = cos(f1) * sin(f2) - sin(f1) * cos(f2) * cos(delta_g);
  azmth = atan2(y, x) * 180 / M_PI;

  magneticDeclination += azmth; //takes into account the magnetic declination in the users position and substracts it for a better and precise connection between point A and point B.
  
  if (azmth < 0)
  {
    azmth += 360.0;
  }
  else if (azmth >= 360.0)
  {
    azmth -= 360.0;
  }
  
  // Print the results to the serial monitor
  Serial.print("Distance (Mi): ");
  Serial.println(dist_Mi);
  Serial.print("Bearing (degrees): ");
  Serial.println(brng_d);
  Serial.print("Azimuth ( CW degrees): ");
  Serial.println(azmth);

  autosteps = azmth * (PPR / 360.0); //360.0 is 1 revolution

  stepper.moveTo(autosteps);

  while (stepper.distanceToGo() != 0)
  {
    digitalWrite(RED_PIN, LOW);
    digitalWrite(GREEN_PIN, LOW);
    digitalWrite(BLUE_PIN, HIGH);
    delayMicroseconds(1000);
    stepper.runToPosition();
  }

  digitalWrite(RED_PIN, LOW);
  digitalWrite(GREEN_PIN, HIGH);
  digitalWrite(BLUE_PIN, LOW);

}