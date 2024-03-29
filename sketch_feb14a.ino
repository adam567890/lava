//#include <Arduino.h>
#include "Adafruit_VL53L0X.h"
//#include <Wire.h>
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
#define LOX1_ADDRESS 0x30 //address for wheel side
#define LOX2_ADDRESS 0x31 //address for other side

// set the pins to shutdown ToF
#define SHT_LOX1 26 //shutdown ToF 1
#define SHT_LOX2 27 //shutdown ToF 2
//#define colorVin 39 //VIN pin for color sensor
#define LEDint 41 //Interrupt for color sensor
#define LEDpin 40 //LED pin for color sensor

#define blackValue 50 //Black color reading (constant, change for new surfaces)
#define whiteValue 460 // White color reading (constant, change for new surfaces)

float blackThreshold = (blackValue + whiteValue)/2; //threshold for line following
double PROPORTIONAL_GAIN = 1.1; //Proportional gain for turn rate

// objects for the vl53l0x laser sensor
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

//object for color sensor
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_1X);

// this holds the measurement for ToF sensors
VL53L0X_RangingMeasurementData_t meas1;
VL53L0X_RangingMeasurementData_t meas2;

/*
    Reset all ToF sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
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
  if(!lox1.begin(LOX1_ADDRESS, false, &Wire)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  delay(10);
  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if(!lox2.begin(LOX2_ADDRESS, false, &Wire)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }  
  // Now we're ready to get readings!
}



void setup() {
  // Initialize the motor control pins as outputs
  pinMode(Ain1, OUTPUT);
  pinMode(Ain2, OUTPUT);
  pinMode(Bin1, OUTPUT);
  pinMode(Bin2, OUTPUT);
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  //pinMode(colorVin, OUTPUT);
  pinMode(LEDint, OUTPUT);
  pinMode(LEDpin, OUTPUT);

  //Setting analogWrite parameters for PWM control of motors
  analogWriteFrequency(10, 100000); // The maximum PWM frequency for the TB6612 is 100khz
  analogWriteFrequency(11, 100000);
  analogWriteResolution(11);  // analogWrite resolution set to 11 bits, maximum setting is 2047. Motors move above ~600

  Serial.begin(115200); //open serial port for USB debugging
  
  // wait until serial port opens for native USB devices
  delay(1000);

  //color sensor init, address for this one is 0x29 and cannot be reprogrammed. Using Wire1 bus instead of Wire bus
  if (tcs.begin(0x29, &Wire1)) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }

  pinMode(SHT_LOX1, OUTPUT); //setting pinmode for shutdown pins for ToF
  pinMode(SHT_LOX2, OUTPUT);

  Serial.println(F("Shutdown pins inited..."));

  digitalWrite(SHT_LOX1, LOW); //Setting shutdownpins to initialize reset and address assignment process
  digitalWrite(SHT_LOX2, LOW);

  Serial.println(F("Both in reset mode...(pins are low)"));
  Serial.println(F("Starting..."));
  setID(); //Call setID to assign I2C addresses to sensors
  
 
}

void loop() {
   
   /*for(int i = 0; i < 70; i++)     //this is commented out because it breaks VL53L0X readings and sends them to zero
   {
    followLine(); // follow line needs to be called continuously for given timeframe or until a given condition is met
   }
   */
   straight();
   delay(100);
   stopMotors(); // make sure to call stopmotors when done to stop forward motion as adjustments will no longer be made once outside follow line
   read_dual_sensors(); //still having trouble with this one
   //delay(100);
   //getColor(); //working as intended, utilized in follow line, but called here for printout to serial (commented out because it sends VL53L0X readings to zero)
   //delay(100);
 
  
}

void followLine()
{
  uint16_t r, g, b, c; //RGB values and conglomerated color value all as unsigned 16 bit ints
  tcs.getRawData(&r, &g, &b, &c); //Retreive sensor raw data
  float deviation = c - blackThreshold; //finding deviation of color data from black threshold
  int turn_rate = PROPORTIONAL_GAIN * deviation; //determine turn rate based on sign of deviation, deviation, and proportional gain
  drive(35, turn_rate); //pass in driving speed, turn rate into drive function
}

void drive(float drive_speed, int turn_rate)
{
  //Set motor driver logic pins for forward motion
  digitalWrite(Ain1, LOW);
  digitalWrite(Ain2, HIGH);
  digitalWrite(Bin1, HIGH);
  digitalWrite(Bin2, LOW);
  
  // Map the speed to a PWM value within the range of 0 to 2047
  int baseSpeedPWM = map(drive_speed, 0, 100, 0, 2047); // Adjust source range based on your requirements

  // Calculate the maximum allowed PWM based on 1.5 times the base speed
  int maxPWM = baseSpeedPWM * 3;

  // Calculate the adjustment for the motor speed based on the turn rate
  // This implementation assumes turn_rate in the range -100 to 100
  int speedAdjustment = map(abs(turn_rate), 0, 100, 0, baseSpeedPWM / 2);

  // Initialize motor speeds to base speed
  int motorASpeedPWM = baseSpeedPWM;
  int motorBSpeedPWM = baseSpeedPWM;
  //int motorASpeedPWMbase = baseSpeedPWM;

  // Adjust the speed of Motor A for turning
  if (turn_rate > 0) {
    // Left turn: Increase Motor A's speed, but not beyond 1.5 times the base speed
    motorASpeedPWM = min(baseSpeedPWM + speedAdjustment, maxPWM);
  } else if (turn_rate < 0) {
    // Right turn: Decrease Motor A's speed
    motorASpeedPWM = max(baseSpeedPWM - speedAdjustment, 0);
    
  }
  /*if(motorASpeedPWM <= (0.3 * 2047)) { // Adjust the threshold as needed
    // Reverse Motor A's direction
    digitalWrite(Ain1, HIGH);
    digitalWrite(Ain2, LOW);
    //digitalWrite(Bin1, LOW);
    //digitalWrite(Bin2, HIGH);
    analogWrite(11, baseSpeedPWM); // Maintain speed for Motor B
    analogWrite(10, motorBSpeedPWM);
  }*/
  
     // Apply the calculated PWM values to the motors
    analogWrite(11, motorASpeedPWM);
    analogWrite(10, motorBSpeedPWM); // Motor B always runs at base speed
  
 
  
  }
  


void read_dual_sensors() {
  
  lox1.rangingTest(&meas1, true); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&meas2, true); // pass in 'true' to get debug data printout!

  // print sensor one reading
  Serial.print(F("1: "));
  if(meas1.RangeStatus != 4) {     // if not out of range
    Serial.print(meas1.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }
  
  Serial.print(F(" "));

  // print sensor two reading
  Serial.print(F("2: "));
  if(meas2.RangeStatus != 4) {
    Serial.print(meas2.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }
  
  Serial.println();
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


void straight() {
  digitalWrite(Ain1, LOW); //Motor driver logic set up for forward motion on both motors
  digitalWrite(Ain2, HIGH);
  digitalWrite(Bin1, HIGH);
  digitalWrite(Bin2, LOW);
  analogWrite(11, 2047); //Set PWM to max
  analogWrite(10, 2047); //Set PWM to max
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


