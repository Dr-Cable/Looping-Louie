#include <Adafruit_NeoPixel.h>
#include <Bounce2.h>
#include <math.h>


#define LEDStrip 10                     //Signal NeoPixel LED strip
#define dcdcOn 52                       //5V DCDC LED & iPhone enable
#define tempSensorNTC A1                //NTC resistor 6k8 Ohms
#define powersupplySensorPin A0         //connected to powersupply plug to dedect battery or powersupply mode
#define LENGTH 2

const int buttonTurbo1 = 23;            //buttons and LED's
const int buttonTurboLED1 = 25;
const int buttonTurbo2 = 27;
const int buttonTurboLED2 = 29;
const int buttonTurbo3 = 31;
const int buttonTurboLED3 = 33;
const int buttonTurbo4 = 35;
const int buttonTurboLED4 = 37;
const int buttonTurbo5 = 39;
const int buttonTurboLED5 = 41;
const int buttonTurbo6 = 43;
const int buttonTurboLED6 = 45;
const int buttonTurbo7 = 47;
const int buttonTurboLED7 = 49;
const int buttonTurbo8 = 51;
const int buttonTurboLED8 = 53;
const int buttonStart = 22;
const int buttonStartLED = 24;
const int buttonReset = 26;
const int buttonResetLED = 28;

const int motorPin1A = 46;              //12V DC motor game
const int motorPin2A = 44;
const int motorEnablePin = 12;

const int fanPin3A = 48;                //5V DC FAN
const int fanPin4A = 50;
const int fanEnablePin = 11;

unsigned long currentTime;
unsigned long loopTime;
unsigned long loopTime1;
unsigned long loopTime2;
unsigned long loopTime3;
unsigned long loopTime4;
unsigned long loopTime5;
unsigned long intervalTimeSpeedChange;
unsigned long intervalTimeDirectionChange;
unsigned long intervalTempMeasurement;
unsigned long backTime;
unsigned long stopTime;
unsigned long turboTime;
unsigned long turboResetTime;

int backTimeCounter;
int stopTimeCounter;
int turboTimeCounter;
int motorSpeedPWM;
int turboSpeedPWM;
int motorSpeed;
int turboSpeed;
int motorSpeedMinPWM = 38;
int motorSpeedMaxPWM = 160;
int backSpeedPWM = 45;
int fanSpeedPWM = 110;
int speedSteps = 7;
int speedFactor;
int speedRandom;
int randomDirectionMotor;
int speed;
int turboResetMode;
int temperature;
int tempMaxMotorOn = 28;
int tempMaxMotorOff = 35;
int tempMin = 23;

boolean battery = false;            //Battery mode or power supply mode
boolean firstStart = false;         //initialization done
boolean motorOn;                    //motor turns
boolean forward = true;             //motor direction
boolean speedMode;                  //speed mode false = fixed motor speed, true = random motor speed
boolean backEnable;                 //motor direction back enabled if true
boolean backOn;                     //motor direction back active
boolean stopEnable;                 //motor stop enabled if true
boolean stopOn;                     //motor stop active
boolean turboEnable;                //motor turbo speed enabled if true
boolean turboOn;                    //motor turbo speed active
boolean fanOn;                      //fan On

int usedTurbo1 = 0;                 //counts turbo usage of each player reseted with reset button
int usedTurbo2 = 0;
int usedTurbo3 = 0;
int usedTurbo4 = 0;
int usedTurbo5 = 0;
int usedTurbo6 = 0;
int usedTurbo7 = 0;
int usedTurbo8 = 0;

//Temperatur sensor
int NTCResistanceNominal = 6800;    // resistance at 25 degrees C
int tempNominal = 25;               // temp. for nominal resistance 
#define numSamples 5                 // how many samples to take and average
int bCoefficient = 3950;            // The beta coefficient of the thermistor (usually 3000-4000)
int seriesResistor = 6800;          // the value of the 'other' resistor
int samples[numSamples];
int tempSampleCount = 0;

int rxBuffer[128];
int rxIndex = 0;

Bounce debouncerTurbo1 = Bounce();  //debouncer for bouttons
Bounce debouncerTurbo2 = Bounce();
Bounce debouncerTurbo3 = Bounce();
Bounce debouncerTurbo4 = Bounce();
Bounce debouncerTurbo5 = Bounce();
Bounce debouncerTurbo6 = Bounce();
Bounce debouncerTurbo7 = Bounce();
Bounce debouncerTurbo8 = Bounce();
Bounce debouncerStart = Bounce();
Bounce debouncerReset = Bounce();

Adafruit_NeoPixel strip = Adafruit_NeoPixel(72, LEDStrip, NEO_GRB + NEO_KHZ800);
// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial1.println("Arduino reset");
  strip.begin();
  strip.show();                     // Initialize all pixels to 'off'
  pinMode(dcdcOn, OUTPUT);
  pinMode(buttonTurbo1, INPUT);
  pinMode(buttonTurboLED1, OUTPUT);
  pinMode(buttonTurbo2, INPUT);
  pinMode(buttonTurboLED2, OUTPUT);
  pinMode(buttonTurbo3, INPUT);
  pinMode(buttonTurboLED3, OUTPUT);
  pinMode(buttonTurbo4, INPUT);
  pinMode(buttonTurboLED4, OUTPUT);
  pinMode(buttonTurbo5, INPUT);
  pinMode(buttonTurboLED5, OUTPUT);
  pinMode(buttonTurbo6, INPUT);
  pinMode(buttonTurboLED6, OUTPUT);
  pinMode(buttonTurbo7, INPUT);
  pinMode(buttonTurboLED7, OUTPUT);
  pinMode(buttonTurbo8, INPUT);
  pinMode(buttonTurboLED8, OUTPUT);
  pinMode(buttonStart, INPUT);
  pinMode(buttonStartLED, OUTPUT);
  pinMode(buttonReset, INPUT);
  pinMode(buttonResetLED, OUTPUT);
  pinMode(motorEnablePin, OUTPUT);
  pinMode(motorPin1A, OUTPUT);
  pinMode(motorPin2A, OUTPUT);
  pinMode(fanPin3A, OUTPUT);
  pinMode(fanPin4A, OUTPUT);
  pinMode(fanEnablePin, OUTPUT);
  pinMode(tempSensorNTC, INPUT);
  pinMode(powersupplySensorPin, INPUT);
  pinMode(A2, INPUT);                   //for randomSeed function
  
  debouncerTurbo1.attach(buttonTurbo1);
  debouncerTurbo2.attach(buttonTurbo2);
  debouncerTurbo3.attach(buttonTurbo3);
  debouncerTurbo4.attach(buttonTurbo4);
  debouncerTurbo5.attach(buttonTurbo5);
  debouncerTurbo6.attach(buttonTurbo6);
  debouncerTurbo7.attach(buttonTurbo7);
  debouncerTurbo8.attach(buttonTurbo8);
  debouncerStart.attach(buttonStart);
  debouncerReset.attach(buttonReset);  
  debouncerTurbo1.interval(5);
  debouncerTurbo2.interval(5);
  debouncerTurbo3.interval(5);
  debouncerTurbo4.interval(5);
  debouncerTurbo5.interval(5);
  debouncerTurbo6.interval(5);
  debouncerTurbo7.interval(5);
  debouncerTurbo8.interval(5);
  debouncerStart.interval(5);
  debouncerReset.interval(5);
  
  motorSpeed = 2; //value 0 - 6          all values should be set via iPhone and stored if in case iPhone is disconnected
  turboSpeed = 3; //value 0 - 6
  speedFactor = ((motorSpeedMaxPWM - motorSpeedMinPWM) / (speedSteps -1));
  motorSpeedPWM = (motorSpeedMinPWM + (motorSpeed * speedFactor));
  turboSpeedPWM = (motorSpeedMinPWM + (turboSpeed * speedFactor));
 
  speedMode = 1;
  turboResetMode = 2;
  backEnable = true;
  stopEnable = true;
  turboEnable = true;
  turboOn = false;
  fanOn = false;
  
  backTime = 2500;
  stopTime = 1000;
  turboTime = 1000;
  turboResetTime = 4000;
  intervalTimeSpeedChange = 3000;
  intervalTimeDirectionChange = 2000;
  intervalTempMeasurement = 1000;
  
  randomSeed (analogRead(A2));
  
  currentTime = millis();
  loopTime = currentTime;
  loopTime1 = currentTime;
  loopTime2 = currentTime;
  loopTime3 = currentTime;
  loopTime4 = currentTime;
  loopTime5 = currentTime;
  
  backTimeCounter = backTime;
  turboTimeCounter = turboTime;
  stopTimeCounter = stopTime;
  
  analogReference(EXTERNAL);
  temperature = 0;
  
  if (analogRead(powersupplySensorPin) == 0) battery = true;
  else battery = false;
   
  //5V DCDC on or off, supplies LED's and Iphone. Dependence on battery or wall connecter still need to be written
  if (battery == true) {
    digitalWrite(dcdcOn, HIGH);
  }
  else {
    digitalWrite(dcdcOn, LOW);
  }
  
}

void loop() {
  //read serial connection to Iphone to get information about speedMode, speedMax, backEnable, stopEnable, tuboEnable.
  //store last values if no for no serial connection
  
  if (Serial1.available() > 0) {

   rxBuffer[rxIndex++] = Serial1.read();
   if (rxIndex == LENGTH) {

       Serial.print( "Set: " );
       Serial.print( rxBuffer[0], DEC );
       Serial.print( " to: " );
       Serial.println( rxBuffer[1], DEC );

       if( rxBuffer[1] == 0 ) {
         //digitalWrite((int)rxBuffer[0], LOW);
       } else {
         //digitalWrite((int)rxBuffer[0], HIGH);
       }
       rxIndex = 0;
   }
 }
 //delay(10);




  
  
  currentTime = millis(); // Time counter in milli seconds
  
  
  //Startup
  if (firstStart == false) {
    if (battery == false) {                                //power supply pluged in 
      colorWipe(strip.Color(255, 0, 0), 50); // Red
      colorWipe(strip.Color(0, 255, 0), 50); // Green
      colorWipe(strip.Color(0, 0, 255), 50); // Blue
      colorWipe(strip.Color(127, 127, 127), 50); //White
      rainbowCycleOne(5);
      digitalWrite(buttonStartLED, HIGH);
      digitalWrite(buttonResetLED, HIGH);
      digitalWrite(buttonTurboLED1, HIGH);
      digitalWrite(buttonTurboLED2, HIGH);
      digitalWrite(buttonTurboLED3, HIGH);
      digitalWrite(buttonTurboLED4, HIGH);
      digitalWrite(buttonTurboLED5, HIGH);
      digitalWrite(buttonTurboLED6, HIGH);
      digitalWrite(buttonTurboLED7, HIGH);
      digitalWrite(buttonTurboLED8, HIGH);
      firstStart = true;
    }
    else {                                                //Battery mode no LED's and power for iPhone
      digitalWrite(buttonStartLED, HIGH);
      digitalWrite(buttonResetLED, HIGH);
      delay (100);
      digitalWrite(buttonTurboLED1, HIGH);
      delay (100);
      digitalWrite(buttonTurboLED2, HIGH);
      delay (100);
      digitalWrite(buttonTurboLED3, HIGH);
      delay (100);
      digitalWrite(buttonTurboLED4, HIGH);
      delay (100);
      digitalWrite(buttonTurboLED5, HIGH);
      delay (100);
      digitalWrite(buttonTurboLED6, HIGH);
      delay (100);
      digitalWrite(buttonTurboLED7, HIGH);
      delay (100);
      digitalWrite(buttonTurboLED8, HIGH);
      delay (1000);
      firstStart = true;
    }
  }
  
  else {
    if (fanOn == true) {
      fan1(fanSpeedPWM);
    }
    else {
      fan1(0);
    }
    if(motorOn == 0 && currentTime >= (loopTime + 600)) {
      digitalWrite(buttonStartLED, !digitalRead(buttonStartLED));           // flash startbutton when paused 
      loopTime = currentTime;
    }
    if(tempSampleCount < numSamples && currentTime >= (loopTime5 + intervalTempMeasurement)){
      samples[tempSampleCount] = analogRead(tempSensorNTC);
      tempSampleCount++;
      loopTime5 = currentTime;
    }
    if(tempSampleCount >= numSamples){
      tempCalculation();
      tempSampleCount = 0;
      if(temperature >= tempMaxMotorOff && fanOn == false) {  //fan on if Motor is off
        fanOn = true;
      } 
      if(temperature >= tempMaxMotorOn && fanOn == false && motorOn == true) {
        fanOn = true;
      }
      if(temperature <= tempMin && fanOn == true) {          //fan off
        fanOn = false;        
      }
    }
    // debouncer Start and Reset Button
    if (debouncerStart.update() == 1 || debouncerReset.update() == 1) {
      if (debouncerStart.read() == HIGH && firstStart == true){
        digitalWrite(buttonStartLED, LOW);
        motorOn = !motorOn;
        Serial.print("PowersupplySensor: ");
        Serial.println(analogRead(powersupplySensorPin));
      }
      if (debouncerReset.read() == HIGH && firstStart == true){
        digitalWrite(buttonResetLED, LOW);
        colorWipe(strip.Color(0, 0, 255), 10);
        digitalWrite(buttonResetLED, HIGH);
        digitalWrite(buttonTurboLED1, HIGH);
        digitalWrite(buttonTurboLED2, HIGH);
        digitalWrite(buttonTurboLED3, HIGH);
        digitalWrite(buttonTurboLED4, HIGH);
        digitalWrite(buttonTurboLED5, HIGH);
        digitalWrite(buttonTurboLED6, HIGH);
        digitalWrite(buttonTurboLED7, HIGH);
        digitalWrite(buttonTurboLED8, HIGH);
        motorOn = false;
        backOn = false;
        stopOn = false;
        turboOn = false;
      }
    }
  
  //Turbo  
    if (motorOn == true) {
      if (turboEnable == true && (debouncerTurbo1.update() == 1 || debouncerTurbo2.update() == 1 || debouncerTurbo3.update() == 1 || debouncerTurbo4.update() == 1 || debouncerTurbo5.update() == 1 || debouncerTurbo6.update() == 1 || debouncerTurbo7.update() == 1 || debouncerTurbo8.update() == 1)) {
        if (debouncerTurbo1.read() == HIGH && digitalRead(buttonTurboLED1) == HIGH && turboOn == 0) {
          turboTimeCounter = turboTime;
          loopTime = currentTime;
          digitalWrite(buttonTurboLED1, LOW);
          turboOn = true;
          usedTurbo1++;
          colorWipeSingle(strip.Color(255, 0, 0), 10, 1);
        }
   
        if (debouncerTurbo2.read() == HIGH && digitalRead(buttonTurboLED2) == HIGH && turboOn == 0) {
          turboTimeCounter = turboTime;
          loopTime = currentTime;
          digitalWrite(buttonTurboLED2, LOW);
          turboOn = true;
          usedTurbo2++;
          colorWipeSingle(strip.Color(255, 0, 0), 10, 2);
        }
   
        if (debouncerTurbo3.read() == HIGH && digitalRead(buttonTurboLED3) == HIGH && turboOn == 0) {
          turboTimeCounter = turboTime;
          loopTime = currentTime;
          digitalWrite(buttonTurboLED3, LOW);
          turboOn = true;
          usedTurbo3++;
          colorWipeSingle(strip.Color(255, 0, 0), 10, 3);
        }
   
        if (debouncerTurbo4.read() == HIGH && digitalRead(buttonTurboLED4) == HIGH && turboOn == 0) {
          turboTimeCounter = turboTime;
          loopTime = currentTime;
          digitalWrite(buttonTurboLED4, LOW);
          turboOn = true;
          usedTurbo4++;
          colorWipeSingle(strip.Color(255, 0, 0), 10, 4);
        }
   
        if (debouncerTurbo5.read() == HIGH && digitalRead(buttonTurboLED5) == HIGH && turboOn == 0) {
          turboTimeCounter = turboTime;
          loopTime = currentTime;
          digitalWrite(buttonTurboLED5, LOW);
          turboOn = true;
          usedTurbo5++;
          colorWipeSingle(strip.Color(255, 0, 0), 10, 5);
        }
   
        if (debouncerTurbo6.read() == HIGH && digitalRead(buttonTurboLED6) == HIGH && turboOn == 0) {
          turboTimeCounter = turboTime;
          loopTime = currentTime;
          digitalWrite(buttonTurboLED6, LOW);
          turboOn = true;
          usedTurbo6++;
          colorWipeSingle(strip.Color(255, 0, 0), 10, 6);
        }
   
        if (debouncerTurbo7.read() == HIGH && digitalRead(buttonTurboLED7) == HIGH && turboOn == 0) {
          turboTimeCounter = turboTime;
          loopTime = currentTime;
          digitalWrite(buttonTurboLED7, LOW);
          turboOn = true;
          usedTurbo7++;
          colorWipeSingle(strip.Color(255, 0, 0), 10, 7);
        }
   
        if (debouncerTurbo8.read() == HIGH && digitalRead(buttonTurboLED8) == HIGH && turboOn == 0) {
          turboTimeCounter = turboTime;
          loopTime = currentTime;
          digitalWrite(buttonTurboLED8, LOW);
          turboOn = true;
          usedTurbo8++;
          colorWipeSingle(strip.Color(255, 0, 0), 10, 8);
        }
      }
      //Turbo Reset depending on turboResetMode
      if (turboResetMode == 2 && turboOn == 0 && usedTurbo1 == 1 && usedTurbo2 == 1 && usedTurbo3 == 1 && usedTurbo4 == 1 && usedTurbo5 == 1 && usedTurbo6 == 1 && usedTurbo7 == 1 && usedTurbo8 == 1) {
        if (currentTime >= (loopTime + turboResetTime)) {
          digitalWrite(buttonTurboLED1, HIGH);
          digitalWrite(buttonTurboLED2, HIGH);
          digitalWrite(buttonTurboLED3, HIGH);
          digitalWrite(buttonTurboLED4, HIGH);
          digitalWrite(buttonTurboLED5, HIGH);
          digitalWrite(buttonTurboLED6, HIGH);
          digitalWrite(buttonTurboLED7, HIGH);
          digitalWrite(buttonTurboLED8, HIGH);
          usedTurbo1 = 0;
          usedTurbo2 = 0;
          usedTurbo3 = 0;
          usedTurbo4 = 0;
          usedTurbo5 = 0;
          usedTurbo6 = 0;
          usedTurbo7 = 0;
          usedTurbo8 = 0;
          loopTime = currentTime;
        }
      }
      if (turboResetMode == 3 && turboOn == 0 && (usedTurbo1 == 1 || usedTurbo2 == 1 || usedTurbo3 == 1 || usedTurbo4 == 1 || usedTurbo5 == 1 || usedTurbo6 == 1 || usedTurbo7 == 1 || usedTurbo8 == 1)) {
        if (currentTime >= (loopTime + turboResetTime)) {
          digitalWrite(buttonTurboLED1, HIGH);
          digitalWrite(buttonTurboLED2, HIGH);
          digitalWrite(buttonTurboLED3, HIGH);
          digitalWrite(buttonTurboLED4, HIGH);
          digitalWrite(buttonTurboLED5, HIGH);
          digitalWrite(buttonTurboLED6, HIGH);
          digitalWrite(buttonTurboLED7, HIGH);
          digitalWrite(buttonTurboLED8, HIGH);
          usedTurbo1 = 0;
          usedTurbo2 = 0;
          usedTurbo3 = 0;
          usedTurbo4 = 0;
          usedTurbo5 = 0;
          usedTurbo6 = 0;
          usedTurbo7 = 0;
          usedTurbo8 = 0;
          loopTime = currentTime;
        }
      }  
  // Some example procedures showing how to display to the pixels:
  //colorWipe(strip.Color(255, 0, 0), 100); // Red
  //colorWipe(strip.Color(0, 255, 0), 100); // Green
  //colorWipe(strip.Color(0, 0, 255), 100); // Blue
  //colorWipe(strip.Color(127, 127, 127), 100); //White
  // Send a theater pixel chase in...

  //theaterChase(strip.Color(127, 127, 127), 50); // White
  //theaterChase(strip.Color(127,   0,   0), 50); // Red
  //theaterChase(strip.Color(  0,   0, 127), 50); // Blue

  //rainbow(50);
  //rainbowCycle(1);
  //theaterChaseRainbow(50);
  
  //Motor
      if (turboOn == true) {
        if(forward == false && currentTime >= loopTime + 100) {
          speed = 0;
          forward == true;
          loopTime = currentTime;
        }
        if(forward == true && turboTimeCounter >= 50 && currentTime >= (loopTime + 100)) {
          turboSpeedPWM = (motorSpeedMinPWM + (turboSpeed * speedFactor));
          speed = turboSpeedPWM;
          turboTimeCounter = turboTimeCounter - 100;
          loopTime = currentTime;
        }
        if (turboTimeCounter <= 50 ) {
          motorSpeedPWM = (motorSpeedMinPWM + (motorSpeed * speedFactor));
          speed = motorSpeedPWM;
          rainbowOne(5);
          turboOn = false;
          
       }
    }
    
    if(speedMode == 1) {
      if(currentTime >= (loopTime1 + intervalTimeSpeedChange)) {
        speedRandom = random(0, motorSpeed);
        motorSpeedPWM = (motorSpeedMinPWM + (speedRandom * speedFactor));
        speed = motorSpeedPWM;
        loopTime1 = currentTime;
      }
    }
    else {
      motorSpeedPWM = (motorSpeedMinPWM + (motorSpeed * speedFactor));
      speed = motorSpeedPWM;
    }
    
    if(backEnable == true && stopEnable == false) {
      if(currentTime >= (loopTime2 + intervalTimeDirectionChange)) {
        randomDirectionMotor = random (0,9);
        loopTime2 = currentTime;
      }
    }
    if(stopEnable == true && backEnable == false) {
      if(currentTime >= (loopTime2 + intervalTimeDirectionChange)) {
        randomDirectionMotor = random (10,19);
        loopTime2 = currentTime;
      }
    }
    if(backEnable == true && stopEnable == true) {
      if(currentTime >= (loopTime2 + intervalTimeDirectionChange)) {
        randomDirectionMotor = random (0,19);
        loopTime2 = currentTime;
       /* Serial.write("speed: ");
        Serial.println(speed);
        Serial.write("speedRandom: ");
        Serial.println(speedRandom);
        Serial.write("randomDirectionMotor: ");
        Serial.println(randomDirectionMotor);
        Serial.write("backOn: ");
        Serial.println(backOn);
        Serial.write(fanOn);
        Serial.println(fanOn);
   */   }
    }
    
    if(backOn == true) {
      if(forward == false && backTimeCounter >= 50 && currentTime >= (loopTime3 + 100)) {
        speed = backSpeedPWM;
        backTimeCounter = backTimeCounter - 100;
        loopTime3 = currentTime;
      }
      if(forward == true && backTimeCounter >= 50) {
        speed = 0;
        forward = false;
        loopTime3 = currentTime;
        //backTimeCounter = backTime;
      }
      if(forward == false && backTimeCounter <= 50 && speed != 0) {
        speed = 0;
        loopTime3 = currentTime;
      }
      if(forward == false && backTimeCounter <= 50 && speed == 0 && currentTime >= (loopTime3 + 100)) {
        forward = true;
        speed = motorSpeedPWM;
        backTimeCounter = backTime;
        randomDirectionMotor = 8;
        backOn = false;
      }
    }
    
    if(stopOn == true) { 
      if(stopTimeCounter >= 50 && currentTime >= (loopTime3 + 100)) {
        speed = 0;
        stopTimeCounter = stopTimeCounter - 100;
        loopTime3 = currentTime;
      }
      if(stopTimeCounter <= 50) {
        speed = motorSpeedPWM;
        stopTimeCounter = stopTime;
        randomDirectionMotor = 11;
        stopOn = false;
      }
    }
    if(backEnable == true || stopEnable == true) {
      switch (randomDirectionMotor) {
        case 9: //back
          backOn = true;
          break;
        case 10: //stop
          stopOn = true;
          break;  
      }
    }
    
      motor1(speed, forward);
    }
    else {
      motor1(0, forward);
    }
  }  
}

//Motor
void motor1 (int speed, boolean direction) {
analogWrite(motorEnablePin, speed);
digitalWrite(motorPin1A, direction);
digitalWrite(motorPin2A, !direction);
}

//Fan
void fan1 (int fanSpeed) {
analogWrite(fanEnablePin, fanSpeed);
digitalWrite(fanPin3A, LOW);
digitalWrite(fanPin4A, HIGH);
}


// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  int a;
  for(uint16_t i=0; i<=8; i++) {
      strip.setPixelColor(i, c);
      strip.setPixelColor(9+i, c);
      strip.setPixelColor(18+i, c);
      strip.setPixelColor(27+i, c);
      strip.setPixelColor(36+i, c);
      strip.setPixelColor(45+i, c);
      strip.setPixelColor(54+i, c);
      strip.setPixelColor(63+i, c);
      strip.show();
      delay(wait);
  }
  //delay(2000);
}
void colorWipeSingle(uint32_t c, uint8_t wait, int player) {
  for(uint16_t i=0; i<=8; i++) {
      strip.setPixelColor(((player - 1) * 9) + i, c);
      strip.show();
      delay(wait);
  }
  //delay(2000);
}

void rainbow(uint8_t wait) {
  uint16_t i, j;
  for(j=0; j<256; j++) {
    for(i=0; i<9; i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
      strip.setPixelColor(9+i, Wheel((i+j) & 255));
      strip.setPixelColor(18+i, Wheel((i+j) & 255));
      strip.setPixelColor(27+i, Wheel((i+j) & 255));
      strip.setPixelColor(36+i, Wheel((i+j) & 255));
      strip.setPixelColor(45+i, Wheel((i+j) & 255));
      strip.setPixelColor(54+i, Wheel((i+j) & 255));
      strip.setPixelColor(63+i, Wheel((i+j) & 255));
    }
    strip.show();
    delay(wait);
  }
}
void rainbowOne(uint8_t wait) {
  uint16_t i, j;
  for(j=0; j<256; j++) {
    for(i=0; i< 9; i++) {
      strip.setPixelColor(9+i, Wheel(((i * 256 / 9) - j) & 255));
      strip.setPixelColor(18+i, Wheel(((i * 256 / 9) - j) & 255));
      strip.setPixelColor(27+i, Wheel(((i * 256 / 9) - j) & 255));
      strip.setPixelColor(36+i, Wheel(((i * 256 / 9) - j) & 255));
      strip.setPixelColor(45+i, Wheel(((i * 256 / 9) - j) & 255));
      strip.setPixelColor(54+i, Wheel(((i * 256 / 9) - j) & 255));
      strip.setPixelColor(63+i, Wheel(((i * 256 / 9) - j) & 255));
      strip.setPixelColor(i, Wheel(((i * 256 / 9) - j) & 255));
        
    }
    strip.show();
    //delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;
  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< 9; i++) {
      strip.setPixelColor(9+i, Wheel(((i * 256 / 9) - j) & 255));
      strip.setPixelColor(18+i, Wheel(((i * 256 / 9) - j) & 255));
      strip.setPixelColor(27+i, Wheel(((i * 256 / 9) - j) & 255));
      strip.setPixelColor(36+i, Wheel(((i * 256 / 9) - j) & 255));
      strip.setPixelColor(45+i, Wheel(((i * 256 / 9) - j) & 255));
      strip.setPixelColor(54+i, Wheel(((i * 256 / 9) - j) & 255));
      strip.setPixelColor(63+i, Wheel(((i * 256 / 9) - j) & 255));
      strip.setPixelColor(i, Wheel(((i * 256 / 9) - j) & 255));
    }
    strip.show();
    delay(wait);
  }
  //delay(2000);
}
void rainbowCycleOne(uint8_t wait) {
  uint16_t i, j;colorWipe(strip.Color(0, 255, 0), 10);
  for(j=0; j<256; j++) { // 1 cycles of all colors on wheel
    for(i=0; i< 9; i++) {
      strip.setPixelColor(9+i, Wheel(((i * 256 / 9) - j) & 255));
      strip.setPixelColor(18+i, Wheel(((i * 256 / 9) - j) & 255));
      strip.setPixelColor(27+i, Wheel(((i * 256 / 9) - j) & 255));
      strip.setPixelColor(36+i, Wheel(((i * 256 / 9) - j) & 255));
      strip.setPixelColor(45+i, Wheel(((i * 256 / 9) - j) & 255));
      strip.setPixelColor(54+i, Wheel(((i * 256 / 9) - j) & 255));
      strip.setPixelColor(63+i, Wheel(((i * 256 / 9) - j) & 255));
      strip.setPixelColor(i, Wheel(((i * 256 / 9) - j) & 255));
    }
    strip.show();
    delay(wait);
  }
  //delay(2000);
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (int i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, c);    //turn every third pixel on
      }
      strip.show();
     
      delay(wait);
     
      for (int i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
        for (int i=0; i < strip.numPixels(); i=i+3) {
          strip.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
        }
        strip.show();
       
        delay(wait);
       
        for (int i=0; i < strip.numPixels(); i=i+3) {
          strip.setPixelColor(i+q, 0);        //turn every third pixel off
        }
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else if(WheelPos < 170) {
    WheelPos -= 85;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}

void tempCalculation(void) {// average all the samples out
  int i;
  float average = 0;
  
  for (i=0; i< numSamples; i++) {
    average += samples[i];
  }
  
  average /= numSamples;
  //Serial1.print("Average analog reading ");
  //Serial1.println(average);
  
  average = 1023 / average - 1;                      // convert the value to resistance
  average = seriesResistor / average;
  //Serial1.print("Thermistor resistance ");
  //Serial1.println(average);
  
  float steinhart;
  
  steinhart = NTCResistanceNominal / average; // (R/Ro)
  steinhart = log(steinhart); // ln(R/Ro)
  steinhart /= bCoefficient; // 1/B * ln(R/Ro)
  steinhart += 1.0 / (tempNominal + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart; // Invert
  steinhart -= 273.15; // convert to C
  Serial1.print("Temperature ");
  Serial1.print(steinhart);
  Serial1.println(" *C");
  temperature = int(steinhart);
  //Serial1.print("Temperature int");
  //Serial1.print(temperature);
  //Serial1.println(" *C");
}
