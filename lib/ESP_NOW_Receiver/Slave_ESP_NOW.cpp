#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <ESP32Servo.h>

#define MAX_SIGNAL 2000 // Parameter required for the ESC definition
#define MIN_SIGNAL 1000 // Parameter required for the ESC definition
#define MOTOR_PIN 5    // Pin 5 attached to the ESC signal pin
#define SIGNAL_TIMEOUT 1000  // This is signal timeout in milli seconds. We will reset the data if no signal

unsigned long lastRecvTime = 0;

// MAC address of the sender - Middle ESP32
// uint8_t masterAddress[] = {0x48, 0xE7, 0x29, 0x9F, 0xDD, 0xD4};

// New Slave MAC
uint8_t New_MAC_Address[] = {0xB0, 0xA7, 0x32, 0x16, 0x1E, 0x24};

// PMK and LMK keys
static const char* PMK_KEY_STR = "_A_H_L_T_T_T_ED3";
static const char* LMK_KEY_STR = "_SON_DINH_VU_ED3";

typedef struct {

  int CtrlPWM;

  byte JSX;
  byte JSY;
 
  byte button1;
  byte button2;

} ESPNOW_Data;

// Create a structured object
ESPNOW_Data receivedData;

Servo servo1;     
// Servo servo2;     
// Servo servo3;     
// Servo servo4;

void init_ESC()
{
  servo1.attach(MOTOR_PIN, MIN_SIGNAL, MAX_SIGNAL);
  servo1.writeMicroseconds(MIN_SIGNAL);
}

// Assign default input received values
void setInputDefaultValues()
{
  receivedData.JSX = 127;
  receivedData.JSY = 127;
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  if (len == 0)
  {
    return;
  }
  memcpy(&receivedData, incomingData, sizeof(receivedData));
}

void init_ESPNOW_Slave() 
{
  setInputDefaultValues();
 
  Serial.begin(115200);
	// Set Slave ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

	esp_wifi_set_ps(WIFI_PS_NONE);

  // Change MAC
  esp_wifi_set_mac(WIFI_IF_STA, New_MAC_Address);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
	// // Create a slave peer object
  // esp_now_peer_info_t masterPeer;

  // // Set the PMK key
  // esp_now_set_pmk((uint8_t *)PMK_KEY_STR);

  // // Register peer
  // memcpy(masterPeer.peer_addr, masterAddress, 6);
  // masterPeer.channel = 0;

	// 	// ///*** Set the middle device's LMK ***///
	// 	// for (uint8_t i = 0; i < 16; i++)
  //   // {
  //   //   masterPeer.lmk[i] = LMK_KEY_STR[i];
  //   // }

  // masterPeer.encrypt = false;
  
	// // Add peer   
  // if (esp_now_add_peer(&masterPeer) != ESP_OK){
  //   Serial.println("Failed to add peer");
  //   return;
  // }

	// Register callback function for received data
  esp_now_register_recv_cb(OnDataRecv);
}

void writePWM()
{
  servo1.write(receivedData.CtrlPWM);
}

void checkSignal()
{
  //Check Signal lost.
  unsigned long now = millis();
  if ( now - lastRecvTime > SIGNAL_TIMEOUT ) 
  {
    setInputDefaultValues(); 
  }
}