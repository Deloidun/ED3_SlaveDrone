    #include <Wire.h>
    #include <Arduino.h>
    #include <Adafruit_MPU6050.h>  
    #include <ESP32Servo.h>
    #include "../Wifi_receiver/Slave_esp_wifi.h"

    

    float RateRoll, RatePitch, RateYaw;
    float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
    int RateCalibrationNumber;
    
    float DesiredRateRoll, DesiredRatePitch,DesiredRateYaw;
    float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
    float InputRoll, InputThrottle, InputPitch, InputYaw;
    float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
    float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
    float PrevRateRoll, PrevRatePitch, PrevRateYaw;
    float CompeRoll, CompePitch;
    float PIDlimit;
    float PIDReturn[]={0, 0, 0, 0};
    float difference_Dist = (float) 180 / (float) 127;

    float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
    float AccX, AccY, AccZ;
    float AngleRoll, AnglePitch;
    float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
    float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
    float Kalman1DOutput[]={0,0};

    float DesiredAngleRoll, DesiredAnglePitch;
    float ErrorAngleRoll, ErrorAnglePitch;
    float PrevErrorAngleRoll, PrevErrorAnglePitch;
    float PrevItermAngleRoll, PrevItermAnglePitch;

    float PrevDesiredAngleRoll = 0, PrevDesiredAnglePitch = 0;

//Lubrication
    // float PRateRoll = 0.52; float PRatePitch=PRateRoll; float PRateYaw = 0.0; 
    // float IRateRoll = 1.4525; float IRatePitch=IRateRoll; float IRateYaw = 2.0;
    // float DRateRoll = 0.02855; float DRatePitch=DRateRoll; float DRateYaw = 0.0;//0.04125

    // float PRateRoll = 0.5; float PRatePitch=PRateRoll; float PRateYaw = 0.0; 
    // float IRateRoll = 1.46; float IRatePitch=IRateRoll; float IRateYaw = 2.0;
    // float DRateRoll = 0.028525; float DRatePitch=DRateRoll; float DRateYaw = 0.0;//0.04125

    // float PRateRoll = 0.5; float PRatePitch=PRateRoll; float PRateYaw = 0.006; 
    // float IRateRoll = 1.4; float IRatePitch=IRateRoll; float IRateYaw = 2.0; //I rate too high
    // float DRateRoll = 0.0285; float DRatePitch=DRateRoll; float DRateYaw = 0.0;

    // float PAngleRoll = 1.5; float PAnglePitch = PAngleRoll;
    // float IAngleRoll = 0.0; float IAnglePitch = IAngleRoll;
    // float DAngleRoll = 0.0; float DAnglePitch = DAngleRoll;

    float PRateRoll = 0.2875; float PRatePitch=PRateRoll; float PRateYaw = 1.0; 
    float IRateRoll = 0.5; float IRatePitch=IRateRoll; float IRateYaw = 0.008;
    float DRateRoll = 0.03975; float DRatePitch=DRateRoll; float DRateYaw = 0.0; //0.1575

    float PAngleRoll = 0.0; float PAnglePitch = PAngleRoll;
    float IAngleRoll = 0.0; float IAnglePitch = IAngleRoll;
    float DAngleRoll = 0.; float DAnglePitch = DAngleRoll;


// //Motor setup
    #define EscPin_RightFront 5
    #define EscPin_RightBack 23
    #define EscPin_LeftBack 18
    #define EscPin_LeftFront 19

    
    Servo ESC1, ESC2, ESC3, ESC4;

void system_setup(){
    Wire.setClock(400000);
    Wire.begin();
    // delay(250);
    Wire.beginTransmission(0x68); //address of MPU6050
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
}

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement)
{
    KalmanState = KalmanState + 0.004 * KalmanInput;
    KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
    float KalmanGain = KalmanUncertainty * 1/(1 * KalmanUncertainty + 3 * 3);
    KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
    KalmanUncertainty = (1-KalmanGain) * KalmanUncertainty;
    Kalman1DOutput[0] = KalmanState; 
    Kalman1DOutput[1] = KalmanUncertainty;
}

void gyro_signals(void) //angular speed, rad/s or degree/s
{
    Wire.beginTransmission(0x68);
    Wire.write(0x1A); 
    Wire.write(0x05);
    Wire.endTransmission();

    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();

    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(); 
    Wire.requestFrom(0x68,6);

    int16_t AccXLSB = Wire.read() << 8 | Wire.read();
    int16_t AccYLSB = Wire.read() << 8 | Wire.read();
    int16_t AccZLSB = Wire.read() << 8 | Wire.read();

    Wire.beginTransmission(0x68);
    Wire.write(0x1B); 
    Wire.write(0x8);
    Wire.endTransmission();                                                   
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(0x68,6);

    int16_t GyroX=Wire.read()<<8 | Wire.read();
    int16_t GyroY=Wire.read()<<8 | Wire.read();
    int16_t GyroZ=Wire.read()<<8 | Wire.read();

    // RateRoll=(float)GyroX/65.5;
    // RatePitch=(float)GyroY/65.5;
    RateRoll=(float)GyroY/65.5;
    RatePitch=(float)-GyroX/65.5;
    RateYaw=(float)GyroZ/65.5;
    AccX=(float)AccXLSB/4096;
    AccY=(float)AccYLSB/4096;
    AccZ=(float)AccZLSB/4096;

    // AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
    // AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
    AnglePitch = -atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180) - 2.6;
    AngleRoll = -atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180) + 0.8;
}
void calibration_measurement()
{
    for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
        gyro_signals();
        RateCalibrationRoll+=RateRoll  ;
        RateCalibrationPitch+=RatePitch ;
        RateCalibrationYaw+=RateYaw;
        delay(1);
    }
    RateCalibrationRoll /= 2000;
    RateCalibrationPitch /= 2000;
    RateCalibrationYaw /= 2000;
    
}

void gyro_compensate(){
    gyro_signals();
    CompeRoll = AngleRoll;
    CompePitch = AnglePitch;
    // CompeYaw = AngleYaw;
}


void init_ESC(){

    ESC1.attach(EscPin_RightFront, 1000, 2000); //Motor 1
    ESC2.attach(EscPin_RightBack, 1000, 2000); //Motor 2
    ESC3.attach(EscPin_LeftBack, 1000, 2000); //Motor 3
    ESC4.attach(EscPin_LeftFront, 1000, 2000); //Motor 4

    ESC1.writeMicroseconds(1000); // Sending MIN_SIGNAL tells the ESC the calibration value
    ESC2.writeMicroseconds(1000);
    ESC3.writeMicroseconds(1000);
    ESC4.writeMicroseconds(1000);
}

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
/////////////////////// Calibrate for ESC by turning on max and min throttle///////
void WaitForKeyStroke()
{
     while (!Serial.available());

     while (Serial.available())
        Serial.read();
}
void calibrate(){

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
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
}

float ReceiveThrottleInput(){
    //Left JoyStick Control - Throttle
    int MatchingThrottleInput = 0;
    MatchingThrottleInput = PWM;
    return MatchingThrottleInput;
}

void checkInputController(){
    
    ESC1.write(ReceiveThrottleInput());       
    ESC2.write(ReceiveThrottleInput());   
    ESC3.write(ReceiveThrottleInput());   
    ESC4.write(ReceiveThrottleInput());                              // Send the command to the ESC
}

float ReceivePitchInput(){
    //Right JoyStick Control (RY) - Pitch
    int MatchingPitchInput = 0;
    MatchingPitchInput = Y_value;
    return MatchingPitchInput;
}
float ReceiveRollInput(){
    //Right JoyStick Control (RX) - Roll
    int MatchingRollInput = 0;
    MatchingRollInput = X_value;
    return MatchingRollInput;
}
float ReceiveYawInput(){
    int MatchingYawInput = 0;
    
    if (leftB) {
        MatchingYawInput = 0;
        //MatchingYawInput = -30;
    }else {
        MatchingYawInput = 0;
    }
    
    if (rightB) {
        MatchingYawInput = 0;//MatchingYawInput = 30;
    }else {
        MatchingYawInput = 0;
    }
    
    return MatchingYawInput;
}



//PID equation for position (angle) and velocity (rate)
// void pid_equationR(float Error, float P , float I, float D, float PrevError, float PrevIterm, char PIDmode)
void pid_equationR(float Error, float Rate, float P , float I, float D, float PrevError, float PrevIterm, float PrevRate, char PIDmode)
{
    float Pterm=P*Error; //P controller
    float Iterm=PrevIterm-I*(Rate+PrevRate)*0.004/2; //I controller
    PIDReturn[2]=Iterm;

    if (PIDmode == 'P') {           // PID limit mode for Pitch
        Iterm = Iterm + I*(CompePitch + DesiredAnglePitch);
    } else if (PIDmode == 'R') {    // PID limit mode for Roll
        Iterm = Iterm + I*(CompeRoll + DesiredAngleRoll);
    }
    PIDlimit = 180;

    //Set the limit for I integral controller
    if (Iterm > PIDlimit) Iterm = PIDlimit;
    else if (Iterm < -PIDlimit) Iterm = -PIDlimit;

    float Dterm=D*(Error-PrevError)/0.004; //D controller
    float PIDOutput= Pterm+Iterm+Dterm; //PID output is the sum of controllers

    //Set the limit for the PID output
    if (PIDOutput > PIDlimit) PIDOutput = PIDlimit; // in motor value
    else if (PIDOutput < -PIDlimit) PIDOutput = -PIDlimit;
    // constrain(PIDOutput, -400, 400); 

    PIDReturn[0]=PIDOutput;
    PIDReturn[1]=Error;
    PIDReturn[3]=Rate;
}

void pid_equationA(float Error, float P, float D, float PrevError)
{
    float Pterm=P*Error; //P controller
    PIDlimit = 180;

    float Dterm=D*(Error-PrevError)/0.004; //D controller
    float PIDOutput= Pterm+Dterm; //PID output is the sum of controllers

    //Set the limit for the PID output
    if (PIDOutput > PIDlimit) PIDOutput = PIDlimit; // in deg/s
    else if (PIDOutput < -PIDlimit) PIDOutput = -PIDlimit;

    PIDReturn[0]=PIDOutput;
    PIDReturn[1]=Error;
    // PIDReturn[2]=Iterm;
}

void reset_pid(void) {
    //Setpoints for velocity
    PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
    PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;

    //Setpoints for position (angle)
    PrevErrorAngleRoll=0; PrevErrorAnglePitch=0;    
    PrevItermAngleRoll=0; PrevItermAnglePitch=0;

}

//////////////////////////////////Setup/////////////////////////////////////


void corrected_values()
{
    gyro_signals();
    RateRoll -= RateCalibrationRoll ;
    RatePitch -= RateCalibrationPitch ;
    RateYaw -= RateCalibrationYaw;
}

void kalman_1d_roll(){
    kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
    KalmanAngleRoll=Kalman1DOutput[0]; KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
}

void kalman_1d_pitch(){
    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
    KalmanAnglePitch=Kalman1DOutput[0]; KalmanUncertaintyAnglePitch=Kalman1DOutput[1];

}

void value_update(){ 
    DesiredAngleRoll= 0.04*(ReceiveRollInput() - 127);
    // if (DesiredAngleRoll != PrevDesiredAngleRoll){
    //     reset_pid();
    // }
        // PrevDesiredAngleRoll = DesiredAngleRoll;

    DesiredAnglePitch= 0.04*(ReceivePitchInput() - 127);
    // if (DesiredAnglePitch != PrevDesiredAnglePitch){
    //     reset_pid();
    // }
        // PrevErrorAnglePitch = DesiredAnglePitch;

    DesiredRateYaw=0.15*(ReceiveYawInput());
    InputThrottle=ReceiveThrottleInput();
    ErrorAngleRoll=DesiredAngleRoll-KalmanAngleRoll; // co gia tri
    ErrorAnglePitch=DesiredAnglePitch-KalmanAnglePitch;// co gia tri
 }



void pid_equation_angleroll(){
    pid_equationA(ErrorAngleRoll, PAngleRoll, DAngleRoll, PrevErrorAngleRoll);     
    DesiredRateRoll=PIDReturn[0]; 
    PrevErrorAngleRoll=PIDReturn[1];
    // PrevItermAngleRoll=PIDReturn[2];
    }

void pid_equation_anglepitch(){
    pid_equationA(ErrorAnglePitch, PAnglePitch, DAnglePitch, PrevErrorAnglePitch);
    DesiredRatePitch=PIDReturn[0]; 
    PrevErrorAnglePitch=PIDReturn[1];
    // PrevItermAnglePitch=PIDReturn[2];


    ErrorRateRoll=DesiredRateRoll-RateRoll;
    ErrorRatePitch=DesiredRatePitch-RatePitch;
    ErrorRateYaw=DesiredRateYaw-RateYaw; 
    }

void pid_equation_rateroll(){
    pid_equationR(ErrorRateRoll, RateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll, PrevRateRoll,'R');
    InputRoll=PIDReturn[0];
    PrevErrorRateRoll=PIDReturn[1]; 
    PrevItermRateRoll=PIDReturn[2];
    PrevRateRoll = PIDReturn[3];
}

void pid_equation_ratepitch(){
    pid_equationR(ErrorRatePitch, RatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch, PrevRatePitch,'P');
    InputPitch=PIDReturn[0]; 
    PrevErrorRatePitch=PIDReturn[1]; 
    PrevItermRatePitch=PIDReturn[2];
    PrevRatePitch = PIDReturn[3];
}

void pid_equation_rateyaw(){
    pid_equationR(ErrorRateYaw, RateYaw, PRateYaw,IRateYaw, DRateYaw, PrevErrorRateYaw,PrevItermRateYaw, PrevRateYaw,'Y');
    InputYaw=PIDReturn[0]; 
    PrevErrorRateYaw=PIDReturn[1]; 
    PrevItermRateYaw=PIDReturn[2];
    PrevRateYaw = PIDReturn[3];
}

void control_throttle(){
   
   int InputPower = 120; //80% of total power
    if (InputThrottle > InputPower) InputThrottle = InputPower;


    // MotorInput1 = (InputThrottle - InputPitch + InputRoll - InputYaw); //
    // MotorInput2 = (InputThrottle + InputPitch + InputRoll + InputYaw);
    // MotorInput3 = (InputThrottle + InputPitch - InputRoll - InputYaw);
    // MotorInput4 = (InputThrottle - InputPitch - InputRoll + InputYaw);
    MotorInput1 = (InputThrottle - InputPitch - InputRoll - InputYaw); // doi roll giong thay
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
        reset_pid();
    }
    ESC1.write(MotorInput1);
    ESC2.write(MotorInput2);
    ESC3.write(MotorInput3);
    ESC4.write(MotorInput4);
}

void SerialDataWrite() {
    Serial.printf("\n%3.0f, %3.0f, %3.0f, %3.0f, %3.0f, %3.0f", MotorInput1,MotorInput2,MotorInput3,MotorInput4, PrevItermRateRoll, PrevItermRatePitch);
    
}

    // MotorInput1 = (InputThrottle - InputPitch - InputRoll - InputYaw);
    // MotorInput2 = (InputThrottle + InputPitch - InputRoll + InputYaw);
    // MotorInput3 = (InputThrottle + InputPitch + InputRoll - InputYaw);
    // MotorInput4 = (InputThrottle - InputPitch + InputRoll + InputYaw);

    // MotorInput1 = (InputThrottle - InputPitch - InputRoll - InputYaw);
    // MotorInput2 = (InputThrottle - InputPitch + InputRoll + InputYaw);
    // MotorInput3 = (InputThrottle + InputPitch + InputRoll - InputYaw);
    // MotorInput4 = (InputThrottle + InputPitch - InputRoll + InputYaw);