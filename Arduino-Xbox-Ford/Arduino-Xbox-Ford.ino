//Joseph Schroedl FRC Team 2059

//A delay of 1000 Microseconds is Full Reverse
//A delay of 1000 to 1460 Microseconds is Proportional Reverse
//A delay of 1460 to 1540 Microseconds is neutral
//A delay of 1540 to 2000 Microseconds is Proportional Forward
//A delay of 2000 Microseconds is Full Forward

//For Xbox controller with USB shield
#include <Servo.h>
#include <XBOXRECV.h>

//Other Includes for USB shield
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif

/*****User Configurable Variables*****/

//Motor Controller Deadzones. This zone keeps the motor controllers from fluctuation from off to +-1% speed.
const int lowNeutral = 1460;
const int highNeutral = 1540;

const int motorNeutral = 1500;

//This is deadzone threshold on the joystick because the resting position of the joystick varries. Making this value bigger will reqire the user to move the joystick further before the code starts using the joystick values
const int joystickDeadzone = 10000;

const int backLeftPin = 22;  //Back Left Motor pin
int backLeftSpeed = 1500; //Back Left Motor starting speed
Servo backLeftMotor;  //Create the backLeftMotor object for the servo library

const int frontLeftPin = 24;  //Front Left Motor pin
int frontLeftSpeed = 1500; //Front Left Motor starting speed
Servo frontLeftMotor; //Create the frontLeftMotor object for the servo library

const int backRightPin = 36;  //Back Right Motor pin
int backRightSpeed = 1500; //Back Right Motor starting speed
Servo backRightMotor; //Create the backRightMotor object for the servo library

const int frontRightPin = 34; //Front Right Motor pin
int frontRightSpeed = 1500; //Front Right Motor starting speed
Servo frontRightMotor; //Create the frontRightMotor object for the servo library

const int topRollerPin = 26;  //Dogeball Launcher Motor pin
//1460 for no running motors, no buttons held.
//1600 for low power, "X" button held.
//1800 for medium power, "A" button held.
//2000 for high power, "B" button held.
int topRollerSpeed = 1500;  //Dogeball Launcher Motor starting speed
Servo topRollerMotor; //Create the topRollerPin object for the servo library

const int bottomRollerPin = 40;  //Dogeball Launcher Motor pin
//1460 for no running motors, no buttons held.
//1600 for low power, "X" button held.
//1800 for medium power, "A" button held.
//2000 for high power, "B" button held.
int bottomRollerSpeed = 1500;  //Dogeball Launcher Motor starting speed
Servo bottomRollerMotor; //Create the bottomRollerPin object for the servo library

const int armOnePin = 28;
int armOneSpeed = 1500;
Servo armOneMotor;

const int armTwoPin = 38;
int armTwoSpeed = 1500;
Servo armTwoMotor;

//Non-PWM Outputs
const int pistonExtendRelayPin = 44;  //Relay pin to extend the shooter piston

const int pistonRetractRelayPin = 46; //Relay pin to retract the shooter piston

const int compressorRelayPin = 32;  //Relay pin to control power to the air compressor

const int airPressureSensorPin = 30;  //Input pin for the air pressure cut-off switch

const int limitSwitchSensorPin = 15; //Magnetic limit switch for the arm height

//Use the Xbox controller number 0, I am only using one controller
const int controlNum = 0;

const int filter = 15;

/*****Non-Configurable Variables*****/

short joyX = 0;             //joyX < 0 = Left, joyX > 0 = Right
short joyY = 0;             //joyY > 0 = Forward, joyY < 0 = Reverse

short armJoyY = 0;

short joyYFiltered = 0;
short joyYFilteredPrev = 0;

//Joystick turret value
short turretJoyX = 0;

int limitSwitchSensor = 0;

//Variable to keep track of whether we are driving in any direction
bool driving = false;

//Variable to keep track of whether we are shooting
bool shooting = false;

//Variable to keep track of the compressor status
bool compressor = false;

bool fire = false;

//Variable to keep track of the air pressure sensor status
bool airPressureSensor = true;

bool arm = false;

//bool limitSwitchSensor = false;

//Initialization for USB shield
USB Usb;
XBOXRECV Xbox(&Usb);

void setup()
{
  //Initialize USB shield
  Usb.Init();
  //Wait for initialization before continueing

  //Setup for the motor controller outputs pins
  //Drive train
  backLeftMotor.attach(backLeftPin);
  frontLeftMotor.attach(frontLeftPin);
  backRightMotor.attach(backRightPin);
  frontRightMotor.attach(frontRightPin);
  //Other motors
  topRollerMotor.attach(topRollerPin);
  bottomRollerMotor.attach(bottomRollerPin);

  armOneMotor.attach(armOnePin);
  armTwoMotor.attach(armTwoPin);


  //Other Input and Outputs
  pinMode(compressorRelayPin, OUTPUT);
  pinMode(airPressureSensorPin, INPUT_PULLUP);
  pinMode(pistonExtendRelayPin, OUTPUT);
  pinMode(pistonRetractRelayPin, OUTPUT);
  pinMode(limitSwitchSensorPin, INPUT);

  Serial.begin(115200);

  //Wait for USB and other setup to finish. This delay is probably not necessary.
  delay(1000);
}

void stopMotors()
{
  //Drive train
  backLeftMotor.writeMicroseconds(motorNeutral);
  frontLeftMotor.writeMicroseconds(motorNeutral);
  backRightMotor.writeMicroseconds(motorNeutral);
  frontRightMotor.writeMicroseconds(motorNeutral);
  //Other Motors
  topRollerMotor.writeMicroseconds(motorNeutral);
  bottomRollerMotor.writeMicroseconds(motorNeutral);
  //Arm Motors
  armOneMotor.writeMicroseconds(motorNeutral);
  armTwoMotor.writeMicroseconds(motorNeutral);
}

void compressorRun() {
  //Code here will run regardless of the xbox controller connection.
  //Read the air pressure sensor to check the PSI of the pnumatics
  airPressureSensor = digitalRead(airPressureSensorPin);
  if (airPressureSensor <= 0) {
    digitalWrite(compressorRelayPin, HIGH);
  }
  else if (airPressureSensor >= 1) {
    digitalWrite(compressorRelayPin, LOW);
  }
  //Serial.print("airPressureSensor: ");
  //Serial.println(airPressureSensor);
}

void limitSwitch() {
  //limitSwitchSensor = digitalRead(limitSwitchSensorPin);
  limitSwitchSensor = 0;
}

void loop()
{
  compressorRun();
  //limitSwitch();

  Usb.Task();
  if (Xbox.XboxReceiverConnected) {
    //This if statement makes sure that the motors will run only when the controller is connected. The motors will stop running when the controller is disconnected. This is the only way to disable the system other than cutting power.
    if (Xbox.Xbox360Connected[controlNum]) {

      driving = false;
      shooting = false;

      //We have to use the "Val" to seperate it from LeftHatX which is a different varible in the library
      //The "LeftHat" is the left joystick
      //"Pre" = Joystick value before maping the values
      int leftHatXValPre = 0;
      int leftHatYValPre = 0;
      int rightHatYValPre = 0;

      //Grabs the xbox controller left analog stick data once, done here once to save cpu cycles
      int xboxLeftHatXData = Xbox.getAnalogHat(LeftHatX, controlNum);
      int xboxLeftHatYData = Xbox.getAnalogHat(LeftHatY, controlNum);

      int xboxRightHatYData = Xbox.getAnalogHat(RightHatY, controlNum);

      //This is deadzone detection on the joystick because the resting position of the joystick varries
      if (xboxLeftHatXData > joystickDeadzone || xboxLeftHatXData < -joystickDeadzone) {
        leftHatXValPre = xboxLeftHatXData;
        driving = true;
      }
      if (xboxLeftHatYData > joystickDeadzone || xboxLeftHatYData < -joystickDeadzone) {
        leftHatYValPre = xboxLeftHatYData;
        driving = true;
      }

      //This is deadzone detection on the joystick because the resting position of the joystick varries
      if ((xboxRightHatYData > joystickDeadzone) || ((xboxRightHatYData < -joystickDeadzone) ^ limitSwitchSensor == 1)) {
        rightHatYValPre = xboxRightHatYData;
        arm = true;
        Serial.println(rightHatYValPre);

      }

      Serial.print("limit switch: ");
      Serial.println(limitSwitchSensor);
      //Serial.print("xboxLeftHatYData: ");
      //Serial.println(xboxLeftHatYData);

      //This section detects which buttons on the Xbox controller are being held and sets the shooter speed (shooterSpeed) based on which button pressed.
      //1460 for no running motors, no buttons held.
      //1600 for low power, "X" button held.
      //1800 for medium power, "A" button held.
      //2000 for high power, "B" button held.
      //topRollerSpeed and bottomRollerSpeed are set opposite because the shooter needs to spin rollers the opposite direction to pull the ball out

      if ((Xbox.getButtonPress(X, controlNum))) {
        topRollerSpeed = 1700;
        bottomRollerSpeed = 1700;
        shooting = true;
        //Serial.println("X button is being held");
      }
      else if ((Xbox.getButtonPress(A, controlNum))) {
        topRollerSpeed = 1800;
        bottomRollerSpeed = 1800;
        shooting = true;
        //Serial.println("A button is being held");
      }
      else if ((Xbox.getButtonPress(B, controlNum))) {
        topRollerSpeed = 2000;
        bottomRollerSpeed = 2000;
        shooting = true;
        //Serial.println("B button is being held");
      }
      else if ((Xbox.getButtonPress(Y, controlNum))) {
        topRollerSpeed = 1300;
        bottomRollerSpeed = 1300;
        shooting = true;
      }
      else {
        topRollerSpeed = 1500;
        bottomRollerSpeed = 1500;
        shooting = false;
        //Serial.println("No Shooter buttons are being held");
      }

      if ((Xbox.getButtonPress(R1, controlNum))) {
        fire = true;
      }
      else {
        fire = false;
      }

      //Serial.print("shooterSpeed: ");
      //Serial.println(shooterSpeed);

      //Convert the joystick values
      joyY = map(leftHatYValPre, -32768, 32768, 1000, 2000);
      joyX = map(leftHatXValPre, -32768, 32768, -150, 150);

      armJoyY = map(rightHatYValPre, -32768, 32768, 1200, 1800);

      joyYFiltered = (joyY + (joyYFilteredPrev * filter)) / (filter + 1);
      joyYFilteredPrev = joyYFiltered;

      //Serial.print(joyY);
      //Serial.print(" ");
      //Serial.println(joyYFiltered);


      //Serial.print("joyY: ");
      //Serial.println(joyY);

      if (driving) {
        bool drivingForward = joyY > 1650;  //Are we driving?
        bool drivingReverse = joyY < 1300;

        if (joyY <= lowNeutral || joyY >= highNeutral) {
          backLeftSpeed = frontLeftSpeed = backRightSpeed = frontRightSpeed = joyY;   //Sets the speed for all motors based on the Forward/Reverse of the joystick
        }

        int absJoyX = abs(joyX);
        if (absJoyX > 15) {    //Am I moving the joystick left or right?
          if (joyX < 0 && (!drivingForward && !drivingReverse)) {     //Zero point turn Left
            backRightSpeed = highNeutral + absJoyX;   //highNeutral for forwards movement
            frontRightSpeed = highNeutral + absJoyX;  //lowNeutral for backwords movement
            backLeftSpeed = lowNeutral + joyX;
            frontLeftSpeed = lowNeutral + joyX;
          }
          else if (joyX > 0 && (!drivingForward && !drivingReverse)) {      //Zero point turn Right
            backRightSpeed = lowNeutral - joyX;
            frontRightSpeed = lowNeutral - joyX;
            backLeftSpeed = highNeutral + joyX;
            frontLeftSpeed = highNeutral + joyX;
          }
        }
      }

      if (arm) {
        armOneSpeed = armJoyY;
        armTwoSpeed = armJoyY;
      }

      if (driving) {
        //If we moved the joystick to drive then set the motors to the speed determined by the joystick

        backLeftMotor.writeMicroseconds(backLeftSpeed);
        frontLeftMotor.writeMicroseconds(frontLeftSpeed);
        backRightMotor.writeMicroseconds(backRightSpeed);
        frontRightMotor.writeMicroseconds(frontRightSpeed);
      }
      else {
        //Else set the motors to neutral speed so they don't run
        backLeftMotor.writeMicroseconds(motorNeutral);
        frontLeftMotor.writeMicroseconds(motorNeutral);
        backRightMotor.writeMicroseconds(motorNeutral);
        frontRightMotor.writeMicroseconds(motorNeutral);

        //I use the detach command from the Servo library so the motors don't abruptly stop. This keeps taller robots from falling forward or backward when stoping.
        //DOESN'T WORK
        /*backLeftMotor.detach();
          frontLeftMotor.detach();
          backRightMotor.detach();
          frontRightMotor.detach();*/
      }

      if (shooting) {
        topRollerMotor.writeMicroseconds(topRollerSpeed);
        bottomRollerMotor.writeMicroseconds(bottomRollerSpeed);
      }
      else {
        topRollerMotor.writeMicroseconds(motorNeutral);
        bottomRollerMotor.writeMicroseconds(motorNeutral);
      }

      //Serial.println(shooting);
      if (fire) {
        //If we pressed "A" button then call the shooter piston code
        //shooterPiston();

        digitalWrite(pistonExtendRelayPin, HIGH);
        delay(10);
        digitalWrite(pistonExtendRelayPin, LOW);
      }
      else {
        digitalWrite(pistonRetractRelayPin, HIGH);
        delay(10);
        digitalWrite(pistonRetractRelayPin, LOW);
      }

      if (arm) {
        armOneMotor.writeMicroseconds(armOneSpeed);
        armTwoMotor.writeMicroseconds(armTwoSpeed);
      }
      else {
        armOneMotor.writeMicroseconds(motorNeutral);
        armTwoMotor.writeMicroseconds(motorNeutral);
      }
    }

    else {
      //******************The code after this is for safety.******************
      //If the Xbox controller is not connected set the motors to neutral. The motors will not spin without the Xbox controller connected.
      stopMotors();
    }
  }



  else {
    //If the Xbox controller is not connected set the motors to neutral. The motors will not spin without the Xbox controller connected.
    stopMotors();
  }
}
