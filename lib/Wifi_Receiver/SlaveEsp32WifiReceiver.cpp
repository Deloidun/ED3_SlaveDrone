#include <Arduino.h> //Call this libray to use the Serial function
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wifi.h>


//MAC address of the Sender - Middle ESP32
uint8_t masterAddress[] = {0x48, 0xE7, 0x29, 0x9E, 0x94, 0xF8};

//PMK and LMK keys
static const char* PMK_KeyString = "_A_H_L_T_T_T_ED3";
static const char* LMK_KeyString = "_SON_DINH_VU_ED3";

//UART Data receiverd from middle ESP32 through ESP_NOW
byte JS_X_Value; // X value of the joystick
byte JS_Y_Value; // Y value of joystick
byte leftButton; // Value of left button
byte rightButton; // Value of right button
int PWM_value; // Value of potentiometer

//Define a Wifi Joystick message structure
typedef struct {
    int P;

    byte XJS; 
    byte YJS;

    byte LB;
    byte RB;
    
} wifiMess;

//Create a structured object for joystick incoming data
wifiMess controller;

//Callback function executed when data is received
void On_Data_Recv(const uint8_t * mac, const uint8_t * incomingData, int len){
    memcpy(&controller, incomingData, sizeof(controller));
    PWM_value = controller.P;
    JS_X_Value = controller.XJS;
    JS_Y_Value = controller.YJS;
    leftButton = controller.LB;
    rightButton = controller.RB;
    // Serial.printf("[ PWM: %3d, X: %3d, Y: %3d, LB: %d, RB: %d \n]", PWM_value, JS_X_Value, JS_Y_Value, leftButton, rightButton);
}

//Create a function to initialize the ESP-NOW for Slave
void Init_ESPNOW_Slave(){

    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK){
        Serial.println("Error in initializing ESP-NOW!");
        return;
    }

    //Create a middle peer object
    esp_now_peer_info_t masterPeer;

    //Set the PMK key for Slave ESP32
    esp_now_set_pmk ((uint8_t *)PMK_KeyString);

    //Register Middle ESP32 peer
    memset(&masterPeer, 0, sizeof(masterPeer));
    memcpy(masterPeer.peer_addr, masterAddress, 6);
    masterPeer.channel = 0;

        //Set the Middle ESP32's LMK
        for (uint8_t i = 0; i < 16; i++){
            masterPeer.lmk[i] = LMK_KeyString[i];
        }

    masterPeer.encrypt = true; //Only Middle peer is accessible

    //Add middle peer
    if(esp_now_add_peer(&masterPeer) != ESP_OK){
        Serial.println ("Failed to add peer");
        return;
    }
    esp_now_register_recv_cb(On_Data_Recv);
}

// Debug
void Print_PS5_Values(){
    Serial.print("\nJoystick: [");
    Serial.print(JS_X_Value);
    Serial.print("  ");
    Serial.print(JS_Y_Value);
    Serial.print("]");

    Serial.print("\t\tButton: [");
    Serial.print(leftButton);
    Serial.print("  ");
    Serial.print(rightButton);
    Serial.print("]");

    Serial.print("\t\tPotentiometer: [");
    Serial.print(PWM_value);
    Serial.print("]");
}