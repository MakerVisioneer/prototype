#include <Wire.h>
 
int accPinX = 2;                          // accelerometer pin of X-Axis
int accPinY = 3;                          // accelerometer pin of Y-Axis
int sonarPin = A0;                        // sonar pin - reading analog value from A0
int sonarValue = 0 ; 
int vibratorPinLeft = 6;                  // left vibrator pin
int vibratorPinRight = 9;                 // right vibrator pin

int debugger = 0;                         // turn debugger on(1) and off(0)
int plotter = 0 ;                         // turn plotter on(1) and off(0)

void setup() {
    Serial.begin(9600);
    Wire.begin(0x04);
    pinMode(accPinX, INPUT);
    pinMode(accPinY, INPUT);
    pinMode(vibratorPinLeft, OUTPUT);
    pinMode(vibratorPinRight, OUTPUT);
}

void loop() {
    int accX = pulseIn(accPinX, HIGH)/10;     
    int accY = pulseIn(accPinY, HIGH)/10;     
    sonarValue  = analogRead ( sonarPin ) ;     
    if ( accY < 300 ) {
      Wire.write(1); // cross street mode activiated
    } else if( sonarValue < 180) {
        vibrate (sonarValue); 
        Serial.println(0);
    } else {
        Serial.println(0);
        delay(100);
    }
    
    if ( debugger ) {
      if (!plotter) {  Serial.print("Sonar = "); Serial.print(sonarValue); 
      Serial.print("\tAccl(x,y) = "); }
      Serial.print(accX) ; Serial.print("\t"); Serial.print(accY);
      Serial.println();
    }
}

void vibrate(int sonar){
  int waitFactor = sonar < 75 ? 10 : 8 ; 
  int waitTime = ( sonar - 50 ) * waitFactor ;
  digitalWrite (vibratorPinLeft , HIGH) ; 
  digitalWrite (vibratorPinRight, HIGH) ; 
  delay(waitTime) ; 
  digitalWrite(vibratorPinLeft , LOW) ; 
  digitalWrite(vibratorPinRight, LOW) ; 
  delay(waitTime);
}

