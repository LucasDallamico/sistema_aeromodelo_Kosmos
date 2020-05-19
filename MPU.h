#ifndef MPU_H
#define MPU_H
#include <Wire.h>
#include "kalman.h"

//              Endereços do MPU
#define WHO_AM_I                0x68
#define gravidaMax2g            0x00
#define gravidaMax4g            0x01
#define gravidaMax8g            0x10
#define gravidaMax16g           0x11
// Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
#define RESTRICT_PITCH // restrição a +- 90 deg

class mpu{
  public:
    mpu();
    ~mpu();
    void inicializa_sensor(void);
    void get_rotacao_x_y(double *rotacaoX, double *rotacaoY, double *a_x, double *a_y, double *a_z);
    void imprime_valores_rotacao(void);
    void imprime_valores_aceleracao(void);
    String retorna_valores_p_registro(void);
  private:
    Kalman kalmanX;
    Kalman kalmanY;
    double kalAngleX, kalAngleY;
    double accX, accY, accZ;
    float temperatura;
    uint8_t i2cData[14]; // Buffer do I2C data
    uint32_t timer;
    void sets_inicias(void);
};

#endif
