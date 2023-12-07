#include <myPID_Controller.h>


///////////////////////////////////////////////////
//DECLARATION
///////////////////////////////////////////////////
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;

float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[] = {0, 0, 0};

float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
float ReceivingThrottleInput = 0, ReceivingPitchInput = 0, ReceivingYawInput = 0, ReceivingRollInput = 0;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput [] = {0, 0};

float DesiredAngleRoll, DesiredAnglePitch;
float ErrorAngleRoll, ErrorAnglePitch;
float PrevErrorAngleRoll, PrevErrorAnglePitch;
float PrevItermAngleRoll, PrevItermAnglePitch;

//PID gains for rate
float PRateRoll = 0.0; float PRatePitch=PRateRoll; float PRateYaw = 0.0; 
float IRateRoll = 0.0; float IRatePitch=IRateRoll; float IRateYaw = 0.0;
float DRateRoll = 0.0; float DRatePitch=DRateRoll; float DRateYaw = 0.0;

//PID gains for position (angle)
float PAngleRoll = 0.0; float PAnglePitch = PAngleRoll;
float IAngleRoll = 0.0; float IAnglePitch = IAngleRoll;
float DAngleRoll = 0.0; float DAnglePitch = DAngleRoll;

#define EscPin_RightFront 5 //ESC1
#define EscPin_RightBack 23 //ESC2
#define EscPin_LeftBack 18 //ESC3
#define EscPin_LeftFront 19 //ESC4
Servo ESC1, ESC2, ESC3, ESC4;


///////////////////////////////////////////////////
//FUNCTION TO SETUP THE MPU
///////////////////////////////////////////////////
void IMU_Setup(){
    Wire.setClock(400000);
    Wire.begin();

    //Wake up MPU6050
    Wire.beginTransmission(0x68); //ID address of MPU6050
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
}


///////////////////////////////////////////////////
//FUNCTION TO SETUP THE MPU
///////////////////////////////////////////////////
void Init_Serial(){
    Serial.begin(115200);
    while(!Serial);
}


////////////////////////////////////////////////
//GENERAL FUNCTION OF KALMAN FILTER TECHNIQUE
////////////////////////////////////////////////
void Kalman_1D(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement)
{
    KalmanState = KalmanState + 0.004 * KalmanInput;
    KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
    float KalmanGain = KalmanUncertainty * 1/(1 * KalmanUncertainty + 3 * 3);
    KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
    KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
    Kalman1DOutput[0] = KalmanState;
    Kalman1DOutput[1] = KalmanUncertainty;
}


///////////////////////////////////////////////////
//FUNCTION TO INITIALIZE, MEASURE ANGULAR VELOCITY
///////////////////////////////////////////////////
void GyroSignal(void){
    Wire.beginTransmission(0x68);
    Wire.write(0x1A); //Register 0x1A
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

    int16_t GyroX = Wire.read() << 8 | Wire.read();
    int16_t GyroY = Wire.read() << 8 | Wire.read();
    int16_t GyroZ = Wire.read() << 8 | Wire.read();

    RateRoll = (float)GyroX/65.5;
    RatePitch = (float)GyroY/65.5;
    RateYaw = (float)GyroZ/65.5;

    AccX = (float)AccXLSB/4096;
    AccY = (float)AccYLSB/4096;
    AccZ = (float)AccZLSB/4096;

    AngleRoll = atan(AccY/sqrt(AccX*AccX+AccZ*AccZ)) * 1/(3.142/180);
    AnglePitch =-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ)) * 1/(3.142/180);
}


///////////////////////////////////////////////////
//FUNCTION TO CALIBRATE MPU6050
///////////////////////////////////////////////////
void MPU_Calibration(){
    for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber ++){
        GyroSignal(); //Call back function
        RateCalibrationRoll += RateRoll;
        RateCalibrationPitch += RatePitch;
        RateCalibrationYaw += RateYaw;
        delay(1);
    }
    RateCalibrationRoll /= 2000;
    RateCalibrationPitch /= 2000;
    RateCalibrationYaw /= 2000;
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


///////////////////////////////////////////////////////////
//FUNCTION FOR CONVERTING INPUT DATA FROM PS4 TO THROTTLE
///////////////////////////////////////////////////////////
float Convert_Received_PS4_To_Throttle(){
    ReceivingThrottleInput = PWM_Value;
    return ReceivingThrottleInput;
}


///////////////////////////////////////////////////////////
//FUNCTION FOR CONVERTING INPUT DATA FROM PS4 TO PITCH
///////////////////////////////////////////////////////////
float Convert_Received_PS4_To_Pitch(){
    ReceivingPitchInput = JS_Y_Value;
    return ReceivingPitchInput;
}


///////////////////////////////////////////////////////////
//FUNCTION FOR CONVERTING INPUT DATA FROM PS4 TO ROLL
///////////////////////////////////////////////////////////
float Convert_Received_PS4_To_Roll(){
    ReceivingRollInput = JS_X_Value ;
    return ReceivingRollInput;
}


///////////////////////////////////////////////////////////
//FUNCTION FOR CONVERTING INPUT DATA FROM PS4 TO YAW
///////////////////////////////////////////////////////////
float Convert_Received_PS4_To_Yaw(){
    ReceivingYawInput = 0;
    if (LeftButton){
        ReceivingYawInput = -30;
    }
    if (RightButton){
        ReceivingYawInput = 30;
    }
    return ReceivingYawInput;
}


//////////////////////////////////////////////////////////
//GENERAL FUNCTION OF PID FOR POSITION AND VELOCITY
//////////////////////////////////////////////////////////
void General_PID_Equation(float Error, float P, float I, float D, float PrevError, float PrevIterm){
    float Pterm = P * Error; //P Controller
    float Iterm = PrevIterm + I * (Error + PrevError) * 0.004/2; //I Controller

    //Set the limit for I interial controller
    float PIDGainLimit = 400;
    if (Iterm > PIDGainLimit) Iterm = PIDGainLimit;
    else if (Iterm <- PIDGainLimit) Iterm =- PIDGainLimit;

    float Dterm = D * (Error - PrevError) / 0.004; //D Controller
    float PIDOutput = Pterm + Iterm + Dterm; //PID Output is the sum of controllers

    //Set the limit for the PID output
    if (PIDOutput > PIDGainLimit) PIDOutput = PIDGainLimit;
    else if (PIDOutput <- PIDGainLimit) PIDOutput =- PIDGainLimit;

    PIDReturn[0] = PIDOutput;
    PIDReturn[1] = Error;
    PIDReturn[2] = Iterm;
}


//////////////////////////////////////////////////////////
//FUNCTION TO RESET PID AFTER EVERY EXECUTION
//////////////////////////////////////////////////////////
void Reset_PID(void){
    //Setpoints for velocity or we can say, the desired velocity so that PID controller can refer back to control the current result.
    PrevErrorRateRoll = 0; PrevErrorRatePitch = 0; PrevErrorRateYaw = 0;
    PrevItermRateRoll = 0; PrevItermRatePitch = 0; PrevItermRateYaw = 0;

    //Setpoint for position (angle)
    PrevErrorAngleRoll = 0; PrevErrorAnglePitch = 0;    
    PrevItermAngleRoll = 0; PrevItermAnglePitch = 0;
}


///////////////////////////////////////////////////
//FUNCTION TO CONTINUOUSLY UPDATE
///////////////////////////////////////////////////
void MPU_Value_Update(){
    GyroSignal();
    RateRoll -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw -= RateCalibrationYaw;
}


//////////////////////////////////////////////////////////
//FUNCTION THAT IMPLEMENT TO FILTER ROLL
//////////////////////////////////////////////////////////
void Kalman_1D_Roll(){
    Kalman_1D(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
    KalmanAngleRoll = Kalman1DOutput[0];
    KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
}


//////////////////////////////////////////////////////////
//FUNCTION THAT IMPLEMENT TO FILTER PITCH
//////////////////////////////////////////////////////////
void Kalman_1D_Pitch(){
    Kalman_1D(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
    KalmanAnglePitch = Kalman1DOutput[0];
    KalmanUncertaintyAnglePitch = Kalman1DOutput[1];
}


//////////////////////////////////////////////////////////
//FUNCTION UPDATES IMPORTANT VALUES
//////////////////////////////////////////////////////////
void Values_Update(){
    DesiredAngleRoll =  0.1*(Convert_Received_PS4_To_Roll() - 127);
    DesiredAnglePitch = 0.1*(Convert_Received_PS4_To_Pitch()- 127);
    DesiredRateYaw = 0.15 * (Convert_Received_PS4_To_Yaw());
    InputThrottle = Convert_Received_PS4_To_Throttle();
    
    ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;
    ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;
}


//////////////////////////////////////////////////////////
//PID CONTROLLER FOR ROLL ANGLE
//////////////////////////////////////////////////////////
void PID_Equation_AngleRoll(){
    General_PID_Equation (ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll,PrevItermAngleRoll);     
    DesiredRateRoll = PIDReturn[0]; // not avaliable
    PrevErrorAngleRoll = PIDReturn[1];
    PrevItermAngleRoll = PIDReturn[2];
    
}


//////////////////////////////////////////////////////////
//PID CONTROLLER FOR PITCH ANGLE
//////////////////////////////////////////////////////////
void PID_Equation_AnglePitch(){
    General_PID_Equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch);
    DesiredRatePitch = PIDReturn[0];
    PrevErrorAnglePitch = PIDReturn[1];
    PrevItermAnglePitch = PIDReturn[2];
    
    ErrorRateRoll = DesiredRateRoll - RateRoll;
    ErrorRatePitch = DesiredRatePitch - RatePitch;
    ErrorRateYaw = DesiredRateYaw - RateYaw;
}


//////////////////////////////////////////////////////////
//PID CONTROLLER FOR RATE ROLL
//////////////////////////////////////////////////////////
void PID_Equation_RateRoll(){
    General_PID_Equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
    InputRoll = PIDReturn[0];
    PrevErrorRateRoll = PIDReturn[1]; 
    PrevItermRateRoll = PIDReturn[2];
}


//////////////////////////////////////////////////////////
//PID CONTROLLER FOR RATE PITCH
//////////////////////////////////////////////////////////
void PID_Equation_RatePitch(){
    General_PID_Equation (ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
    InputPitch = PIDReturn[0]; 
    PrevErrorRatePitch = PIDReturn[1]; 
    PrevItermRatePitch = PIDReturn[2];
}


//////////////////////////////////////////////////////////
//PID CONTROLLER FOR RATE YAW
//////////////////////////////////////////////////////////
void PID_Equation_RateYaw(){
    General_PID_Equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
    InputYaw = PIDReturn[0];
    PrevErrorRateYaw = PIDReturn[0];
    PrevItermRateYaw = PIDReturn[1];
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


///////////////////////////////////////////////////
//FUNCTION TO PRINT RESULTS FOR DEBUGGING
///////////////////////////////////////////////////
void Print_MPU_Angle(){
    Serial.print("\nRoll >> ");
    Serial.printf("%4.0f", AngleRoll);
    Serial.print(" degrees");
    Serial.print ("     ");
    Serial.print("Pitch >> ");
    Serial.printf("%4.0f", AnglePitch);
    Serial.print(" degrees");
}

void Print_MPU_Rate(){
    Serial.print("\nRRoll >> ");
    Serial.print(RateRoll);
    Serial.print(" deg/sec");
    Serial.print ("      ");
    Serial.print("RPitch >> ");
    Serial.print(RatePitch);
    Serial.print(" deg/sec");
    Serial.print ("      ");
    Serial.print("RYaw >> ");
    Serial.print(RateYaw);
    Serial.print(" deg/sec");
}