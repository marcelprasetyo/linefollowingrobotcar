#include <QTRSensors.h>

// Comments

#define Kp 1.25
#define Kd 0.45
#define Ki 0	// not used and not suitable for the current PID device
#define leftMotorBaseSpeed 50
#define rightMotorBaseSpeed 50

#define maxSpeed 255
#define minSpeed 30

#define threshold 950
//threshold is LDR threshold for ON or BRIGHT

#define LDRpin A5

QTRSensors qtr;

const uint8_t SensorCount = 4;
uint16_t sensorValues[SensorCount];

//double weightage[SensorCount] = {25,5,-5,-25};
double weightage[SensorCount] = {25,5,-5,-25};
double error = 0;
double errorSum = 0;
double lastError = 0;

int leftMotorSpeed;
int rightMotorSpeed;
int leftPrevMotorSpeed;
int rightPrevMotorSpeed;

void setup()
{
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A1,A2, A3, A4}, SensorCount);
  qtr.setEmitterPin(2);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode (4,OUTPUT);
  pinMode (5,OUTPUT);
  pinMode (6,OUTPUT);
  pinMode (7,OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  /* for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
  */
}

bool started;
int LDRvalue;

void loop()
{ 
  if (!started) {
    LDRvalue = analogRead(LDRpin);
    Serial.println(LDRvalue);
    if (LDRvalue > threshold ) started = !started;
  }
  else {
    qtr.readCalibrated(sensorValues);
  
    for (int i=0;i<SensorCount;i++)
    {
      Serial.print("Sensor");
      Serial.print(i);
      Serial.print (": ");
      Serial.println(sensorValues[i]);
    }
  
    error = weightedAverage();
  
    int calibration = 1 * (int)round(PID(error));
  
    Serial.print ("Calibration: ");
    Serial.println (calibration);
  
    leftMotorSpeed = leftMotorBaseSpeed - calibration;
    rightMotorSpeed = rightMotorBaseSpeed + calibration;
  
    //blank space logic
  
    int sumSensor = 0;
    for (int i=0; i<4; i++) {
      sumSensor += sensorValues[i]; 
    }
    if (sumSensor < 85) { //white space; keep previous
      leftMotorSpeed = leftPrevMotorSpeed;
      rightMotorSpeed = rightPrevMotorSpeed;
    }
    else {
  
    // normal logic
      if (leftMotorSpeed >= 0 && rightMotorSpeed >= 0)
      {
        leftMotorSpeed = constrain(leftMotorSpeed, 0, maxSpeed);
        rightMotorSpeed = constrain(rightMotorSpeed, 0, maxSpeed);
    
        analogWrite (5, leftMotorSpeed);
        digitalWrite (4, LOW); //LOW is forward
    
        analogWrite (6, rightMotorSpeed);
        digitalWrite (7, LOW);
      }
      else if (leftMotorSpeed < 0 && rightMotorSpeed > 0)
      {
        leftMotorSpeed = constrain(leftMotorSpeed, 0, minSpeed);
        rightMotorSpeed = constrain(rightMotorSpeed, 0, maxSpeed);
    
        analogWrite (5, leftMotorSpeed);
        digitalWrite (4, HIGH);
    
        analogWrite (6, rightMotorSpeed);
        digitalWrite (7, LOW);
      }
      else if (leftMotorSpeed > 0 && rightMotorSpeed < 0)
      {
        leftMotorSpeed = constrain(leftMotorSpeed, 0, maxSpeed); 
        rightMotorSpeed = constrain(rightMotorSpeed, 0, minSpeed);   
    
        analogWrite (5, leftMotorSpeed);
        digitalWrite (4, LOW);
    
        analogWrite (6, rightMotorSpeed);
        digitalWrite (7, HIGH);
      }
      
      rightPrevMotorSpeed = rightMotorSpeed;
      leftPrevMotorSpeed = leftMotorSpeed;
    }
    
  }
}

double weightedAverage (void)
{
  int sum = 0;
  double weightedSum = 0;
  for (int i = 0; i<SensorCount; i++)
  {
    weightedSum += sensorValues[i]*weightage[i];
    sum += sensorValues[i];
  }
  
  int weightedAverage = round( weightedSum/(double) sum); 

  Serial.print ("Weighted Average: ");
  Serial.println (weightedAverage);
  
  return weightedAverage;
}

double PID (int error)
{
  errorSum += error;
  double differentialError = error - lastError;
  lastError = error;

  double output = Kp*error + Kd*differentialError + Ki*errorSum;
  
  return output;
}
