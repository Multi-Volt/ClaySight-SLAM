
//Copyright (c) 2023, John Simonis and The Ohio State University
//This code was written by John Simonis for the ClaySight research project at The Ohio State University.

#include <CheapStepper.h> //This is a custom library specifically for the stepper motor we are using. This is pretty much necessary as it enables half stepping and thus slightly greater torque for our gearing. https://github.com/tyhenry/CheapStepper
#include <Wire.h> //Standard wire library for I2C. 
#include <TFLI2C.h> //This is a library for the TF Luna lidar module which is used in this project. https://github.com/budryerson/TFLuna-I2C

CheapStepper StepperMotor(21, 20, 19, 18); //Here we are creating the Stepper and lidar objects respectively which are used in the rest of the code
TFLI2C LidarSensor;

int PWMPins[] = {0, 1, 2, 3}; //This array is dynamic within the rest of the code, adding more or less pins will change the amount of motors called for in the rest of the program
const int Deadzone = 10; //In cm
const int MaxDist = 255; //In cm
const int MotorSpeed = 4; //In RPM
const int TotalMotorSteps = 4096;
const float GearRatio = 0.4f;
const int EffectiveMotorSteps = TotalMotorSteps * GearRatio;
const int AmountOfMotors = sizeof(PWMPins) / sizeof(PWMPins[0]); //The sizeof() function in C++ provides an easy way to find the length of an array
const bool DebugMode = true;
int SensorValues[EffectiveMotorSteps] = {}; //Total steps per rotation
int MotorVals[AmountOfMotors] = {255, 255, 255, 255}; //The amount of motors we have is determined by the size of the PWMPins array
int tempTotal;
int16_t Distance; //Distance and I2C address for the lidar sensor
int16_t DebugTemp,DebugFlux;
const int16_t I2CAddress = 0x10;

int MapToPWM(int DistanceIn, bool Invert) { //This function is a custom function that takes the distances measured from the lidar sensor and converts the value into a usable 8 bit integer value.
  int FuncVar;
  double ConvVal = MaxDist / 255;
  if (DistanceIn >= Deadzone && DistanceIn < MaxDist) {
    FuncVar = abs((DistanceIn - Deadzone) / (ConvVal) - (255*Invert));
  } else if (DistanceIn < Deadzone || DistanceIn > MaxDist) {
    FuncVar = 0 + 255 * !Invert;
  }
  return FuncVar;
}

void setup() { //All code run on Core0 (this one) is specifically for controlling the haptic motors.
  if (!DebugMode){
    for (int i = 0; i < AmountOfMotors; i++) {
      pinMode(PWMPins[i], OUTPUT);
    }
  }
}

void loop() {
  if (!DebugMode){
    for (int i = 0; i < AmountOfMotors; i++) {
      analogWrite(PWMPins[i], MotorVals[i]);
    }
  }
}


void setup1() { //Core1 is used to control the stepper motor and lidar module 
  StepperMotor.setRpm(MotorSpeed);
  Serial.begin(57600);
  Wire.begin();
}

void loop1() {
  if (!DebugMode){
    StepperMotor.stepCCW(); //This makes the motor step once per loop
    if (LidarSensor.getData(Distance, I2CAddress)) {
      SensorValues[StepperMotor.getStep() % EffectiveMotorSteps] = Distance; //Here we are mapping every distance value measured to an array of sensor values
    } 
    else {Serial.println(); LidarSensor.printStatus(); Serial.println(); Distance = 0;}  // else, print error.

    for (int i = 0; i < AmountOfMotors; i++){ //This double for loop may seem complicated but its rather quite simple, we are iterating through every single motor defined and then all recorded sensor values.
      for (int q = 0; q <= EffectiveMotorSteps/AmountOfMotors; q++){ //In this case all of our sensor values are divided per motor, the length of an array in C++ can be found using the sizeof() function as seen here and in code above.
        tempTotal += MapToPWM(SensorValues[q+(EffectiveMotorSteps/AmountOfMotors*i)],true); //Here we are appending the value to a temporary total variable which will be used at the end of the loop.
        if (q == EffectiveMotorSteps/AmountOfMotors){ //Once we have recorded all of the sensor values applicable we find the average by dividing the temp total by the total size of the array divided per motor.
          MotorVals[i] = tempTotal/(EffectiveMotorSteps/AmountOfMotors);
          Serial.println("Motor " + String(i) + " " + String(MotorVals[i])); //This is for debugging purposes so we can see the value going to each motor.
          tempTotal = 0; //Resets the value for the next loop in the initially declared for loop.
        }
      }
    }
  }
  else if (DebugMode){ //This mode is exclusive for use with the ClaySight datalogger application
    while(Serial.available()==0){} //Before any sequence is run in loop there must be no serial input.
    bool input = Serial.readStringUntil('\n');
    if (input){//Upon serial input this sequence will begin
      for(int i = 1; i <= 3; i++){//Here the lidar sensor will rotate 3 times and it will log all data in CSV format per amount of TotalMotorSteps.
        Serial.println("Rotation: " + String(i));
        Serial.println("Distance(M),Flux(Q),Temperature(C),DistPWM(byte)");
        for(int q = 0; q <= EffectiveMotorSteps; q++){
          LidarSensor.getData(Distance, DebugFlux, DebugTemp, I2CAddress);
          Serial.println(String(Distance) + "," + String(DebugFlux) + "," + String(DebugTemp/100) + "," + String(MapToPWM(Distance,true)));
          StepperMotor.stepCCW();
        }
      }
      Serial.println("DataEND");
    }
  }
}