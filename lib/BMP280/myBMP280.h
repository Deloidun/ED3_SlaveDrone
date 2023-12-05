#pragma once
#include <BasicLinearAlgebra.h>

extern BLA :: Matrix <2,2> F;
extern BLA :: Matrix <2,1> G;
extern BLA :: Matrix <2,2> P;
extern BLA :: Matrix <2,2> Q;
extern BLA :: Matrix <2,1> S;
extern BLA :: Matrix <1,2> H;
extern BLA :: Matrix <2,2> I;
extern BLA :: Matrix <1,1> Acc;
extern BLA :: Matrix <2,1> K;
extern BLA :: Matrix <1,1> R;
extern BLA :: Matrix <1,1> L;
extern BLA :: Matrix <1,1> M;

extern float AccZ_Inertial;
extern float Altitude, AltitudeBarometer, AltitudeBarometerStartUp;

void BMP280_Setup();
void BMP280_Check();
void BMP280_Calibration();
void Matrix_Manipulation();
void BMP280_Update_Values();