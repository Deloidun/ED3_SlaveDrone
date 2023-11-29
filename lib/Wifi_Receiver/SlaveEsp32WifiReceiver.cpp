#include <Arduino.h> //Call this libray to use the Serial function
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wifi.h>


//MAC address of the Sender - Middle ESP32
uint8_t MiddleAddress[] = {0x48, 0xE7, 0x29, 0x9F, 0xDD, 0xD4};

//New MAC address of Receiver - Slave ESP32
uint8_t NewMACAddress[] = {0xB0, 0xA7, 0x32, 0x16, 0x1E, 0x24};

//PMK and LMK keys
static const char* PMK_KeyString = "_A_H_L_T_T_T_ED3";
static const char* LMK_KeyString = "_SON_DINH_VU_ED3";

//UART Data receiverd from middle ESP32 through ESP_NOW
int8_t LStickY_ReceivedValue; //Left Joystick Y
int8_t RStickY_ReceivedValue; //Right Joystick Y
int8_t RStickX_ReceivedValue; //Right Joystick X

int8_t L1_ReceivedValue;
int8_t R1_ReceivedValue;

//Define a Wifi Joystick message structure
typedef struct {
    int8_t LStickY; //Left Joystick Y
    int8_t RStickY; //Right Joystick Y (Pitch)
    int8_t RStickX; //Right Joystick X (Roll)

    int8_t L1; //L1 button
    int8_t R1; //R1 button
} PS4_ReceivedMessage;

//Create a structured object for joystick incoming data
PS4_ReceivedMessage PS4_IncomingData;

//Create a middle peer object
esp_now_peer_info_t MiddlePeer;

//Callback function executed when data is received
void On_Data_Recv(const uint8_t * mac, const uint8_t * incomingData, int len){
    memcpy(&PS4_IncomingData, incomingData, sizeof(PS4_IncomingData));

    LStickY_ReceivedValue = PS4_IncomingData.LStickY;
    RStickY_ReceivedValue = PS4_IncomingData.RStickY;
    RStickX_ReceivedValue = PS4_IncomingData.RStickX;

    L1_ReceivedValue = PS4_IncomingData.L1;
    R1_ReceivedValue = PS4_IncomingData.R1;
}

//Create a function to initialize the ESP-NOW for Slave
void Initialize_ESPNOW_Slave(){
    WiFi.mode(WIFI_STA);
    //Assign the new MAC address to Slave ESP32
    esp_wifi_set_mac(WIFI_IF_STA, NewMACAddress);

    if (esp_now_init() != ESP_OK){
        Serial.println("Error in initializing ESP-NOW!");
        return;
    }

    //Set the PMK key for Slave ESP32
    esp_now_set_pmk ((uint8_t *)PMK_KeyString);

    //Register Middle ESP32 peer
    memcpy(MiddlePeer.peer_addr, MiddleAddress, 6);
    MiddlePeer.channel = 0;

    //Set the Middle ESP32's LMK
    for (uint8_t i = 0; i < 16; i++){
        MiddlePeer.lmk[i] = LMK_KeyString[i];
    }
    MiddlePeer.encrypt = true; //Only Middle peer is accessible

    //Add middle peer
    if(esp_now_add_peer(&MiddlePeer) != ESP_OK){
        Serial.println ("Failed to add peer");
        return;
    }
    esp_now_register_recv_cb(On_Data_Recv);
}


void Print_PS4_Values(){
    Serial.print("\nJoystick: [");
    Serial.print(LStickY_ReceivedValue); //LY joystick Y
    Serial.print("  ");
    Serial.print(RStickY_ReceivedValue); //RY joystick Y
    Serial.print("  ");
    Serial.print(RStickX_ReceivedValue); //RX joystick X
    Serial.print("]");

    Serial.print("\t\tButton L1 & R1: [");
    Serial.print(L1_ReceivedValue); //L1 button
    Serial.print("  ");
    Serial.print(R1_ReceivedValue); //R1 button
    Serial.print("]");
}