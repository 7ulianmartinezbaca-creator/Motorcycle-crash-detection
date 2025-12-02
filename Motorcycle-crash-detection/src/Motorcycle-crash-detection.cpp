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

SYSTEM_MODE(SEMI_AUTOMATIC);

double cm = 0.0;
double inches = 0.0;

int trigpin = D14;
int echopin = D19;

HC_SR04 rangefinder = HC_SR04(trigpin, echopin);

void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected,10000);

  Spark.variable("cm", &cm, DOUBLE);
  Spark.variable("inches", &inches, DOUBLE);
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  cm = rangefinder.getDistanceCM();
  inches = rangefinder.getDistanceInch();  

  Serial.printf("%i\n",inches);
}

