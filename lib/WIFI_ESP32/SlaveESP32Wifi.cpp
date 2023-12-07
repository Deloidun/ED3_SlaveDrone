#include <SlaveESP32Wifi.h>

//////////////////////////////////////////////////
//DECLARE MAC ADDRESS AND ENCRYPTING SIGNAL
//////////////////////////////////////////////////
uint8_t MasterAddress[] = {0x48, 0xE7, 0x29, 0x9E, 0x94, 0xF8};

//New MAC address of Receiver - Slave ESP32
uint8_t NewMACAddress[] = {0x48, 0xE7, 0x29, 0x96, 0x77, 0x44};

//PMK and LMK keys
static const char* PMK_KeyString = "_A_H_L_T_T_T_ED3";
static const char* LMK_KeyString = "_SON_DINH_VU_ED3";


//////////////////////////////////////////////////
//PARAMETERS, DEF STRUCT AND OBJECT DECLARATION
//////////////////////////////////////////////////
byte JS_X_Value; //X value of the joystick
byte JS_Y_Value; //Y value of the joystick
byte LeftButton; //Value of the left button
byte RightButton; //Value of the right button
int PWM_Value; //Value of the potentionmeter

//Define a WiFi Joystick message structure
typedef struct{
    byte XJS; //X Joystick
    byte YJS; //Y Joystick
    byte LB; //Left button
    byte RB; //Right button
    int P; //Pontentionmeter
} WiFiMess;

// typedef struct {
//     float Time;
//     float K_Angle_Roll;
//     float K_Angle_Pitch;
//     float K_Altitude;
//     float Temp;
//     float Pressure;
// } Wifi_Sensor_MSG;

//Create a structured objects
WiFiMess RemoteController;
// Wifi_Sensor_MSG Sensor;

//////////////////////////////////////////////////
//RECEIVED AND SENT FUNCTIONS
//////////////////////////////////////////////////
void OnDataRecv(const uint8_t * mac, const uint8_t * incomingData, int len){
    memcpy(&RemoteController, incomingData, sizeof(RemoteController));
    PWM_Value = RemoteController.P;
    JS_X_Value = RemoteController.XJS;
    JS_Y_Value = RemoteController.YJS;
    LeftButton = RemoteController.LB;
    RightButton = RemoteController.RB;
}

// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){

// }


//////////////////////////////////////////////////
//INITIALIZE FUNCTION FOR THE ESPNOW FOR SLAVE
//////////////////////////////////////////////////
void Init_ESPNOW_Slave(){
    WiFi.mode(WIFI_STA);
    esp_wifi_set_mac(WIFI_IF_STA, NewMACAddress);

    if (esp_now_init() != ESP_OK){
        Serial.println("Error in initializing ESP-NOW!");
        return;
    }
    //Create a Master peer object
    esp_now_peer_info_t MasterPeer; 

    //Set the PMK Key for Slave ESP32
    esp_now_set_pmk ((uint8_t *)PMK_KeyString); 

    //Register Master ESP32 peer
    memset(&MasterPeer, 0, sizeof(MasterPeer));
    memcpy(MasterPeer.peer_addr, MasterAddress, 6);
    MasterPeer.channel = 0;

    //Set the Master ESP32's LMK
    for (uint8_t i = 0; i < 16; i++){
        MasterPeer.lmk[i] = LMK_KeyString[i];
    }
    MasterPeer.encrypt = true; //Only Master Peer is accesible

    //Add Master Peer
    if (esp_now_add_peer(&MasterPeer) != ESP_OK){
        Serial.println ("Failed to add peer");
        return;
    }

    esp_now_register_recv_cb(OnDataRecv); //Receive
    // esp_now_register_send_cb(OnDataSent); //Send
}


// void SendingData_ThroughESPNOW(){
//     Sensor.Time = LoopTimer;
//     Sensor.K_Angle_Roll = KalmanAngleRoll;
//     Sensor.K_Angle_Pitch = KalmanAnglePitch;
//     Sensor.K_Altitude = Altitude;
//     Sensor.Temp = Temperature;
//     Sensor.Pressure = Pressure;

//     esp_err_t result = esp_now_send(MasterAddress, (uint8_t *) &Sensor, sizeof(Sensor));
// }


//////////////////////////////////////////////////
//PRINTING VALUES FOR DEBUGGING
//////////////////////////////////////////////////
void Print_PS5_Values(){
    Serial.print("\nJoystick: [");
    Serial.print(JS_X_Value);
    Serial.print("  ");
    Serial.print(JS_Y_Value);
    Serial.print("]");

    Serial.print("\t\tButton: [");
    Serial.print(LeftButton);
    Serial.print("  ");
    Serial.print(RightButton);
    Serial.print("]");

    Serial.print("\t\tPotentiometer: [");
    Serial.print(PWM_Value);
    Serial.print("]");
}