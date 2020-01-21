#ifndef MPU_H
#define MPU_H

#include <Wire.h>
#include "kalman.h"

/* _____________________________________________________________ */
//               Descrição da classe

//              Endereços do MPU
#define WHO_AM_I                0x68
#define gravidaMax2g            0x00
#define gravidaMax4g            0x01
#define gravidaMax8g            0x10
#define gravidaMax16g           0x11

class coordenada : public Kalman{
public:
  double acc;
  double giro;
  double giroK; // Ângulo calculado usando um filtro Kalman
  double angulo;
  //const double getgiroK(void);
  coordenada(void);
  ~coordenada(void);
};

class mpuDados{
private:
  coordenada x_dado;
  coordenada y_dado;
  coordenada z_dado;
  float temperatura;
  uint8_t i2cData[14]; // Buffer do I2C dat

public:
  double gyroXangle, gyroYangle; // Cálculo de ângulo usando apenas o giroscópio
  double compAngleX, compAngleY; // Ângulo calculado usando um filtro complementar

  mpuDados(void);
  ~mpuDados();
  bool inicia_comun_i2c(void);
  bool sensorFunciona(void);
  void valoresIniciais(void);
  const float getAcc(char eixo);
  const float getGiro(char eixo);
  void getValoresSensorI2C(double dt);
  void printSemfiltro(void);
  void printComfiltro(void);
  String vetDadosparaAnalise(void);
};

#endif
