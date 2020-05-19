#include "MPU.h"

mpu::mpu()
{
  Serial.println(F("Inicializando o MPU ..."));
  kalAngleX = 0.0; kalAngleY = 0.0; temperatura = 0.0;
  accX = 0.0; accY = 0.0; accZ = 0.0;
}

mpu::~mpu(){}

void mpu::inicializa_sensor(void)
{
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = gravidaMax8g; // Seta a gravidade maxima como 8 G
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != WHO_AM_I) { // Read "WHO_AM_I" register
    Serial.println(F("Erro na comunicação com o MPU !!!"));
    while (1);
  } else {
    Serial.println(F("MPU incializado."));
  }
  delay(100); // Wait for sensor to stabilize
  sets_inicias();
}

void mpu::sets_inicias(void)
{
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  #ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else // Eq. 28 and 29
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  timer = micros();
}

void mpu::get_rotacao_x_y(double *rotacaoX, double *rotacaoY, double *a_x, double *a_y, double *a_z)
{
  double gyroX, gyroY, gyroZ;
  double gyroXangle, gyroYangle; // Angle calculate using the gyro only
  double compAngleX, compAngleY; // Calculated angle using a complementary filter
  int16_t tempRaw;

  // Obtem os dados da comunicação i2C
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  #ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else // Eq. 28 and 29
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

  #ifdef RESTRICT_PITCH
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
      kalmanX.setAngle(roll);
      compAngleX = roll;
      kalAngleX = roll;
      gyroXangle = roll;
    } else
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleX) > 90)
      gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  #else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
      kalmanY.setAngle(pitch);
      compAngleY = pitch;
      kalAngleY = pitch;
      gyroYangle = pitch;
    } else
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleY) > 90)
      gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  #endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  // Temperatura
  temperatura = (double)tempRaw / 340.0 + 36.53;
  
  // Retornar os valores de rotação x e y
  if (rotacaoX)
    *rotacaoX = kalAngleX;
  if (rotacaoY)
    *rotacaoY = kalAngleY;

  // Retornar os valores de aceleração x,y,z
  if (a_x)
    *a_x = accX;
  if (a_y)
    *a_y = accY;
  if (a_z)
    *a_z = accZ;
}
    
void mpu::imprime_valores_rotacao(void)
{
  Serial.print(F("Valores rotacao X e Y = "));
  Serial.print(kalAngleX);
  Serial.print(F(" "));
  Serial.println(kalAngleY);
}

void mpu::imprime_valores_aceleracao(void)
{
  Serial.print(F("Valores aceleração [x,y,z] = "));
  Serial.print(accX);
  Serial.print(F(" "));
  Serial.print(accY);
  Serial.print(F(" "));
  Serial.println(accZ);
}

String mpu::retorna_valores_p_registro(void)
{
  String mens = {""};
  // Escreve as rotações X e Y
  mens = (String)((float)kalAngleX) + "," + (String)((float)kalAngleY);
  // Escrever as acelerações x,y,z 
  mens += "," + (String)((float)accX) + "," + (String)((float)accY) + "," + (String)((float)accZ);
  return mens;
}
