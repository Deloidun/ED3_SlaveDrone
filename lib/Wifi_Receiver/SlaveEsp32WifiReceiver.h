#pragma once

extern int8_t LStickY_ReceivedValue;
extern int8_t RStickY_ReceivedValue;
extern int8_t RStickX_ReceivedValue;
extern int8_t L1_ReceivedValue;
extern int8_t R1_ReceivedValue;

void Initialize_ESPNOW_Slave();
void Print_PS4_Values();