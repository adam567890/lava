#include <Arduino.h>
#include "Adafruit_VL53L0X.h"
#include <Wire.h>
#include "Adafruit_TCS34725.h"
// Motor A
#define Ain1 7
#define Ain2 6
// Motor B
#define Bin1 8
#define Bin2 9
//PWMs
#define pwm1 10 //A
#define pwm2 11 //B
//Sensors
#define LOX1_ADDRESS 0x31 //address for wheel side
#define LOX2_ADDRESS 0x32 //address for other side
// set the pins to shutdown
#define SHT_LOX1 26 //shutdown 
#define SHT_LOX2 27 //shutdown
#define ledint 39 //LED interrupt pin for color sensor

// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
//color sensor object
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

/*
    Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
    Keep sensor #1 awake by keeping XSHUT pin high
    Put all other sensors into shutdown by pulling XSHUT pins low
    Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
    Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
 */
void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // activating LOX1 and resetting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);

  // initing LOX1
  if(!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if(!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }
  
}

void read_dual_sensors() {
  
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!

  // print sensor one reading
  Serial.print(F("1: "));
  if(measure1.RangeStatus != 4) {     // if not out of range
    Serial.print(measure1.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }
  
  Serial.print(F(" "));

  // print sensor two reading
  Serial.print(F("2: "));
  if(measure2.RangeStatus != 4) {
    Serial.print(measure2.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }
  
  Serial.println();
}

void setup() {
    // Initialize the motor control pins as outputs
  pinMode(Ain1, OUTPUT);
  pinMode(Ain2, OUTPUT);
  pinMode(Bin1, OUTPUT);
  pinMode(Bin2, OUTPUT);
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  //pinMode(scl2, OUTPUT);
  analogWriteFrequency(10, 375000); // Teensy 3.0 pin 3 also changes to 375 kHz
 // analogWriteResolution(4096);  // analogWrite value 0 to 4095, or 4096 for high
  analogWriteFrequency(11, 375000); // Teensy 3.0 pin 3 also changes to 375 kHz
  Serial.begin(115200);
  
  // wait until serial port opens for native USB devices
  while (! Serial) { delay(1); }
//color sensor init
  if (tcs.begin(0x29, &Wire1)) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);

  Serial.println(F("Shutdown pins inited..."));

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);

  Serial.println(F("Both in reset mode...(pins are low)"));
  
  
  Serial.println(F("Starting..."));
  setID();
  
 
}

void loop() {
  read_dual_sensors();
  delay(100);
}

void straight() {
  digitalWrite(Ain1, LOW);
  digitalWrite(Ain2, HIGH);
  digitalWrite(Bin1, HIGH);
  digitalWrite(Bin2, LOW);
  analogWrite(11, 256); //Set PWM to max
  analogWrite(10,256); //Set PWM to max
}

void moveBackward() {
  digitalWrite(Ain1, HIGH);
  digitalWrite(Ain2, LOW);
  digitalWrite(Bin1, LOW);
  digitalWrite(Bin2, HIGH);
}

void turnLeftFWD(int turn_rate) {
  digitalWrite(Ain1, LOW);
  digitalWrite(Ain2, HIGH);
  digitalWrite(Bin1, HIGH);
  digitalWrite(Bin2, LOW);
  analogWrite(11, turn_rate);
}

void turnRightFWD(int turn_rate) {
  digitalWrite(Ain1, LOW);
  digitalWrite(Ain2, HIGH);
  digitalWrite(Bin1, HIGH);
  digitalWrite(Bin2, LOW);
  analogWrite(10, turn_rate);
}

void stopMotors() {
  digitalWrite(Ain1, LOW);
  digitalWrite(Ain2, LOW);
  digitalWrite(Bin1, LOW);
  digitalWrite(Bin2, LOW);
}

void getColor()
{
  uint16_t r, g, b, c, colorTemp, lux;

  tcs.getRawData(&r, &g, &b, &c);
  // colorTemp = tcs.calculateColorTemperature(r, g, b);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(r, g, b);

  Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
  Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
  Serial.println(" ");
}
