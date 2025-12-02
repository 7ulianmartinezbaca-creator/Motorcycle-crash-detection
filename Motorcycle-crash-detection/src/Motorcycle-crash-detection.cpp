/* 
 * Project motorcycle-crash-detection
 * Author: julian
 * Date: 12/2/25
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"


SYSTEM_MODE(SEMI_AUTOMATIC);

long microsecondsToInches(long microseconds);

const int trigpin = D14;
const int echopin = D19;

int duration, inches;

void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected,10000);

  pinMode(trigpin,INPUT);
  pinMode(echopin,OUTPUT);
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  digitalWrite(echopin, LOW);
  delayMicroseconds(2);
  digitalWrite(echopin, HIGH);
  delayMicroseconds(5);
  digitalWrite(echopin, LOW);

  duration = pulseIn(trigpin, HIGH);

  inches = microsecondsToInches(duration);

  Serial.printf("%i\n",inches);
}

long microsecondsToInches(long microseconds) {
  // According to Parallax's datasheet for the PING))), there are 73.746
  // microseconds per inch (i.e. sound travels at 1130 feet per second).
  // This gives the distance travelled by the ping, outbound and return,
  // so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}