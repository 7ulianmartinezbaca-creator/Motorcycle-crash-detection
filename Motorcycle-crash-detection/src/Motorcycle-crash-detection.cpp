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

SYSTEM_MODE(SEMI_AUTOMATIC);

double cm = 0.0;
double inches = 0.0;

const int trigpin = D17;
const int echopin = D15;

const int pixcount = 46;
Adafruit_NeoPixel pixel(pixcount, SPI1, WS2812B);
HC_SR04 rangefinder = HC_SR04(trigpin, echopin);

void pixelFill(int startP, int endP, int color);

void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected,10000);

  pixel.begin();
  pixel.setBrightness(75);
  pixel.show();
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  inches = rangefinder.getDistanceInch();
  delay(100);

  Serial.printf("%lf. %lf\n",cm, inches);
}

void pixelFill(int startP, int endP, int color){
  int i;
  for(i = startP; i <= endP; i++){
    pixel.setPixelColor(i,color);
  }
  pixel.show();
  pixel.clear();
}