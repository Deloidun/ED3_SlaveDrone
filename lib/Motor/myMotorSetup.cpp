#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>

#include <SlaveEsp32WifiReceiver.h>
#include <myPID_Controller.h>


////////////////////////////////////////////////
//PARAMTERS DECLARATION
////////////////////////////////////////////////
#define EscPin_RightFront 5 //ESC1
#define EscPin_RightBack 23 //ESC2
#define EscPin_LeftBack 18 //ESC3
#define EscPin_LeftFront 19 //ESC4
Servo ESC1, ESC2, ESC3, ESC4;

float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
float ReceivingThrottleInput = 0, ReceivingPitchInput = 0, ReceivingYawInput = 0, ReceivingRollInput = 0;

///////////////////////////////////////////////////////////
//FUNCTION FOR CONVERTING INPUT DATA FROM PS4 TO THROTTLE
///////////////////////////////////////////////////////////
float Convert_Received_PS4_To_Throttle(){
    ReceivingThrottleInput = PWM_value;
    return ReceivingThrottleInput;
}


///////////////////////////////////////////////////////////
//FUNCTION FOR CONVERTING INPUT DATA FROM PS4 TO ROLL
///////////////////////////////////////////////////////////
float Convert_Received_PS4_To_Roll(){
    ReceivingRollInput = JS_X_Value ;
    return ReceivingRollInput;
}


///////////////////////////////////////////////////////////
//FUNCTION FOR CONVERTING INPUT DATA FROM PS4 TO PITCH
///////////////////////////////////////////////////////////
float Convert_Received_PS4_To_Pitch(){
    ReceivingPitchInput = JS_Y_Value;
    return ReceivingPitchInput;
}


///////////////////////////////////////////////////////////
//FUNCTION FOR CONVERTING INPUT DATA FROM PS4 TO YAW
///////////////////////////////////////////////////////////
float Convert_Received_PS4_To_Yaw(){
    ReceivingYawInput = 0;
    if (leftButton){
        ReceivingYawInput = -30;
    }
    if (rightButton){
        ReceivingYawInput = 30;
    }
    return ReceivingYawInput;
}


////////////////////////////////////////////////
//FUNCTION FOR INTIALIZING ESC
////////////////////////////////////////////////
void Initialize_ESC(){
    ESC1.attach(EscPin_RightFront, 1000, 2000); //Motor 1
    ESC2.attach(EscPin_RightBack, 1000, 2000); //Motor 2
    ESC3.attach(EscPin_LeftBack, 1000, 2000); //Motor 3
    ESC4.attach(EscPin_LeftFront, 1000, 2000); //Motor 4

    ESC1.writeMicroseconds(1000);
    ESC2.writeMicroseconds(1000);
    ESC3.writeMicroseconds(1000);
    ESC4.writeMicroseconds(1000);
}


////////////////////////////////////////////////
//FUNCTION FOR CALIBRATING ESC
////////////////////////////////////////////////
void WaitForKeyStroke()
{
  while (!Serial.available());
  while (Serial.available())
    Serial.read();
}

void Calibrate_ESC(){
    delay(2000);
    ESC1.attach(EscPin_RightFront, 1000, 2000); //Motor 1
    ESC2.attach(EscPin_RightBack, 1000, 2000); //Motor 2
    ESC3.attach(EscPin_LeftBack, 1000, 2000); //Motor 3
    ESC4.attach(EscPin_LeftFront, 1000, 2000); //Motor 4

    Serial.println();
    Serial.println("Calibration step 1. Disconnect the battery.");
    Serial.println("Press any key to continue.");
    WaitForKeyStroke();
    ESC1.writeMicroseconds(2000); // Sending MAX_SIGNAL tells the ESC to enter calibration mode
    ESC2.writeMicroseconds(2000);
    ESC3.writeMicroseconds(2000);
    ESC4.writeMicroseconds(2000);
    Serial.println();
    Serial.println("Calibration step 2. Connect the battery.");
    Serial.println("Wait for two short bips.");
    Serial.println("Press any key to continue.");
    WaitForKeyStroke();

    ESC1.writeMicroseconds(1000); // Sending MIN_SIGNAL tells the ESC the calibration value
    ESC2.writeMicroseconds(1000);
    ESC3.writeMicroseconds(1000);
    ESC4.writeMicroseconds(1000);
    Serial.println();
    Serial.println("Wait for 4 short bips, and one long bip.");
    Serial.println("Press any key to finish.");
    WaitForKeyStroke();
}


///////////////////////////////////////////////////////////
//FUNCTION FOR SENDING CONVERTED THROTTLE TO ESC
//JUST FOR CALIBRATION MODE ONLY
///////////////////////////////////////////////////////////
void Send_Input_Throttle_2_ESC(){
    ESC1.write(Convert_Received_PS4_To_Throttle());
    ESC2.write(Convert_Received_PS4_To_Throttle());
    ESC3.write(Convert_Received_PS4_To_Throttle());
    ESC4.write(Convert_Received_PS4_To_Throttle()); 
}


///////////////////////////////////////////////////////////////////////////////////////
//FUNCTION TO CONTROL THROTTLE WHICH DISTRIBUTE PWM TO EVERY ESC TO BALANCE THE DRONE
///////////////////////////////////////////////////////////////////////////////////////
void Control_Throttle_To_Balance_Drone(){
    int InputPower = 120; //80% of total power
    if (InputThrottle > InputPower) InputThrottle = InputPower;

    MotorInput1 = (InputThrottle - InputPitch - InputRoll - InputYaw);
    MotorInput2 = (InputThrottle + InputPitch - InputRoll + InputYaw);
    MotorInput3 = (InputThrottle + InputPitch + InputRoll - InputYaw);
    MotorInput4 = (InputThrottle - InputPitch + InputRoll + InputYaw);

    int InputThrottleConstant = 180;
    if (MotorInput1 > InputThrottleConstant) MotorInput1 = InputThrottleConstant - 1;
    if (MotorInput2 > InputThrottleConstant) MotorInput2 = InputThrottleConstant - 1;
    if (MotorInput3 > InputThrottleConstant) MotorInput3 = InputThrottleConstant - 1;
    if (MotorInput4 > InputThrottleConstant) MotorInput4 = InputThrottleConstant - 1;

    int Throttle_Idle = 20;
    if (MotorInput1 < Throttle_Idle) MotorInput1 = Throttle_Idle;
    if (MotorInput2 < Throttle_Idle) MotorInput2 = Throttle_Idle;
    if (MotorInput3 < Throttle_Idle) MotorInput3 = Throttle_Idle;
    if (MotorInput4 < Throttle_Idle) MotorInput4 = Throttle_Idle;

    int ThrottleCutOff = 0;
    if (InputThrottle < 20){
        MotorInput1 = ThrottleCutOff;
        MotorInput2 = ThrottleCutOff;
        MotorInput3 = ThrottleCutOff;
        MotorInput4 = ThrottleCutOff;
        Reset_PID();
    }
    ESC1.write(MotorInput1);
    ESC2.write(MotorInput2);
    ESC3.write(MotorInput3);
    ESC4.write(MotorInput4);
}






