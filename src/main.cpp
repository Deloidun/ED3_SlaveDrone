#include <Arduino.h>
#include <Slave_ESP_NOW.h>

#define SIGNAL_TIMEOUT 1000  // This is signal timeout in milli seconds. We will reset the data if no signal

void setup()
{
  init_ESC();
  init_ESPNOW_Slave();
}

void loop()
{
  checkSignal();
  writePWM();
}