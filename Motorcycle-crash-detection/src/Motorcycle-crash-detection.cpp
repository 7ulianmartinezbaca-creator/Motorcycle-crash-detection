/* 
 * Project motorcycle-crash-detection
 * Author: julian
 * Date: 12/2/25
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "HC_SR04.h"
#include "neopixel.h"
#include "IoTclassroom_CNM.h"
#include "Colors.h"
#include <Adafruit_GPS.h>
#include "Adafruit_SSD1306.h"
#include "adafruit_GFX.h"
#include "Math.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "credentails.h"

byte accel_x_h,accel_x_l;
byte accel_y_h,accel_y_l;
byte accel_z_h,accel_z_l;

int16_t accel_x;
int16_t accel_y;
int16_t accel_z;

Adafruit_GPS GPS(&Wire);

float shockArray[500];
float xAxis_Gs,yAxis_Gs,zAxis_Gs;
float lat, lon, alt,spe,spee;
float Atot,greatestPick;
float lastPick = 0.0;
double inches = 0.0,ft;
int arrCounter;
int sat;
const int trigpin = D17;
const int echopin = D15;
const int TIMEZONE = -6;
const int pixcount = 46;
const int OLED_RESET=-1;
const int MPUADDRESS = 0x68;
unsigned int lastAccel;
unsigned int lastGPS;
unsigned int lastTime;
const unsigned int UPDATE = 30000;
bool printData;

Adafruit_SSD1306 display(OLED_RESET);
Adafruit_NeoPixel pixel(pixcount, SPI1, WS2812B);
HC_SR04 rangefinder = HC_SR04(trigpin, echopin);
TCPClient TheClient;
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SEVERPORT,AIO_USERNAME,AIO_KEY);
Adafruit_MQTT_Subscribe subFeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/"); 
Adafruit_MQTT_Publish pubFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/");

SYSTEM_THREAD(ENABLED);

void MQTT_connect();
bool MQTT_ping();
void getShock();
void pixelFill(int startP, int endP, int color);
void getGPS(float *latitude, float *longitude, float *altitude, int *satellites, float *speed);

SYSTEM_MODE(SEMI_AUTOMATIC);

void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected,10000);

  new Thread("shockThread",getShock);

  pixel.begin();
  pixel.setBrightness(75);
  pixel.show();

  display.setRotation(2);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // initalization 
  display.clearDisplay();
  display.display();

  // WiFi.on();
  // WiFi.connect();
  // while(WiFi.connecting()) {
  //   Serial.printf(".");
  // }
  // mqtt.subscribe(&subFeed);

  Wire.begin();
  Wire.beginTransmission(MPUADDRESS);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  //Initialize GPS
  GPS.begin(0x10);  // The I2C address to use is 0x10
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  GPS.println(PMTK_Q_RELEASE);

  printData = false;
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {

  // MQTT_connect();
  // MQTT_ping();

  //Serial.printf("Inches: %0.2lf\nFeet: %0.2f\n",inches,ft);

  if ((millis()- lastAccel)>10) {
    getShock();
  }

  if (printData) {
    Serial.printf("sh: %0.4lf\n",greatestPick);
  }

  getGPS(&lat,&lon,&alt,&sat,&spe);
  GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) {
      return;
    }   
  }

  inches = rangefinder.getDistanceInch();
  ft = inches / 12.0;
  spee = spe * 1.15078;
  
  // Adafruit_MQTT_Subscribe *subscription;
  // while ((subscription = mqtt.readSubscription(100))) {
  //   if (subscription == &subFeed) {
  //     i  = atoi((char *)subFeed.lastread);
  //   }
  // } 
  
  // if((millis()-lastTime > 10000000)) {
  //   if(mqtt.Update()) {
  //     pubFeed.publish();
  //   } 
  //   lastTime = millis();
  // }

  // display.clearDisplay();
  // display.setTextSize(1);
  // display.setTextColor(WHITE);
  // display.setCursor(0,0);
  // //display.printf("Lat: %0.6f, Lon: %0.6f, Alt: %0.6f\n, Satellites: %i, speed in knots %f\n",lat, lon, alt, sat, spee);
  // //display.printf("Shock data: %f\n",);
  // display.setTextColor(BLACK, WHITE); // 'inverted' text
  // display.display();

  pixelFill(0,46,green);

}

void pixelFill(int startP, int endP, int color){
  int i;
  for(i = startP; i <= endP; i++){
    pixel.setPixelColor(i,color);
  }
  pixel.show();
  pixel.clear();
}

void getGPS(float *latitude, float *longitude, float *altitude, int *satellites, float *speed){
  int theHour;

  theHour = GPS.hour + TIMEZONE;
  if(theHour < 0) {
    theHour = theHour + 24;
  }
    
  // Serial.printf("Time: %02i:%02i:%02i:%03i\n",theHour, GPS.minute, GPS.seconds, GPS.milliseconds);
  // Serial.printf("Dates: %02i-%02i-20%02i\n", GPS.month, GPS.day, GPS.year);
  // Serial.printf("Fix: %i, Quality: %i",(int)GPS.fix,(int)GPS.fixquality);
    if (GPS.fix) {
      *latitude = GPS.latitudeDegrees;
      *longitude = GPS.longitudeDegrees; 
      *altitude = GPS.altitude;
      *satellites = (int)GPS.satellites;
      *speed = GPS.speed;
      // Serial.printf("Lat: %0.6f, Lon: %0.6f, Alt: %0.6f\n",*latitude, *longitude, *altitude);
      // Serial.printf("Speed (m/s): %0.2f\n",GPS.speed/1.944);
      // Serial.printf("Angle: %0.2f\n",GPS.angle);
      // Serial.printf("Satellites: %i\n",*satellites);
    }
}

void MQTT_connect() {
  int8_t ret;
 
  // Return if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds...\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");
}

bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;

  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      pingStatus = mqtt.ping();
      if(!pingStatus) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
  return pingStatus;
}

void getShock() {
  printData = !printData;
  while (arrCounter < 500) {
    arrCounter = 0;
    // point to acceleromitor registor
    Wire.beginTransmission(MPUADDRESS);
    Wire.write(0x3B);
    Wire.requestFrom(MPUADDRESS, 6, true);
    Wire.endTransmission(false);
      // Read X, Y, and Z data
    accel_x_h = Wire.read();
    accel_x_l = Wire.read();
    accel_y_h = Wire.read();
    accel_y_l = Wire.read();
    accel_z_h = Wire.read();
    accel_z_l = Wire.read();
      // Join hish and low bytes to get 16 bit measurment -- Convert to G's using +/- 2g's
    accel_x = accel_x_h << 8 | accel_x_l ;
    xAxis_Gs  = accel_x / 16384.0;
      //Serial.printf("%f    ",xAxis_Gs);

    accel_y = accel_y_h << 8 | accel_y_l;
    yAxis_Gs = accel_y / 16384.0;
      //Serial.printf("%f    ",yAxis_Gs);

    accel_z = accel_z_h << 8 | accel_z_l;
    zAxis_Gs = accel_z / 16384.0;
      //Serial.printf("%f    ",zAxis_Gs);

      // Calculate total accelration 
    Atot = sqrt((xAxis_Gs*xAxis_Gs)+(yAxis_Gs*yAxis_Gs)+(zAxis_Gs*zAxis_Gs));
      // Store total accelration in array
    shockArray[arrCounter] = Atot;
    arrCounter++;
    lastAccel = millis();
  }
  for (int i = 0; i < 500 ; i++){
      if (greatestPick > shockArray[i]) {
        greatestPick = shockArray[i];
      }
      printData = TRUE;
  }
}