#include "MPU.h"
/* _____________________________________________________________ */
//               IMPLEMENTAÇÃO DA CLASSE mpuDados
/* ------------------------------------------------------------- */
coordenada::coordenada(void){
  acc = 0.0; giro = 0.0; giroK = 0.0; angulo = 0.0;
}
coordenada::~coordenada(void){}
/* ------------------------------------------------------------- */
mpuDados::mpuDados(void){
  gyroXangle = 0.0; gyroYangle = 0.0;
  compAngleX = 0.0; compAngleY = 0.0;
}
mpuDados::~mpuDados(){}
/* ------------------------------------------------------------- */
bool mpuDados::inicia_comun_i2c(void)
{
  // Iniciando a comunicação I2C
  Wire.begin();
  #if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  #else
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency
  #endif
  i2cData[0] = 7;    // Defina a taxa de amostragem para 1000Hz - 8kHz / (7 + 1) = 1000Hz
  i2cData[1] = 0x00; // Desative o FSYNC e defina a filtragem de 260 Hz Acc, filtragem de 256 Hz Gyro, amostragem de 8 KHz
  i2cData[2] = 0x00; // Defina a escala de escala completa do giroscópio para ≤250deg / s
  i2cData[3] = gravidaMax2g; // Defina a escala completa do acelerômetro para ≤2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Escrever para todos os quatro registros de uma só vez
  while (i2cWrite(0x6B, 0x01, true)); // PLL com referência ao giroscópio do eixo X e desativar o modo de suspensão
  while (i2cRead(0x75, i2cData, 1));
  return 1;
}
/* ------------------------------------------------------------- */
bool mpuDados::sensorFunciona(void){
  if (i2cData[0] != WHO_AM_I) { // Read "WHO_AM_I" register
    Serial.println(F("Não foi possível inicializar o MPU !!!"));
    while(1);
    return 0;
  }
  return 1;
}

/* ------------------------------------------------------------- */
const float mpuDados::getAcc(char eixo){
  if ( eixo == 'x' || eixo == 'X')
    return x_dado.acc;
  else if ( eixo == 'y' || eixo == 'Y')
    return y_dado.acc;
  else if ( eixo == 'z' || eixo == 'Z')
    return z_dado.acc;
  else
    return 0;
}
/* ------------------------------------------------------------- */
const float mpuDados::getGiro(char eixo){
  if ( eixo == 'x' || eixo == 'X')
    return x_dado.giro;
  else if ( eixo == 'y' || eixo == 'Y')
    return y_dado.giro;
  else if ( eixo == 'z' || eixo == 'Z')
    return z_dado.giro;
  else
    return 0;
}
/* ------------------------------------------------------------- */
void mpuDados::valoresIniciais(void){
  /* Definir o ângulo inicial de kalman e giroscópio */
  while (i2cRead(0x3B, i2cData, 6));
  x_dado.acc = (int16_t)(i2cData[0] << 8) | i2cData[1];
  y_dado.acc = (int16_t)(i2cData[2] << 8) | i2cData[3];
  z_dado.acc = (int16_t)(i2cData[4] << 8) | i2cData[5];

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -p to p (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  // Restringe em 90 a variação do angulo
  // Eq. 25 and 26
  double roll  = atan2(y_dado.acc, z_dado.acc) * RAD_TO_DEG;
  double pitch = atan(-x_dado.acc / sqrt(y_dado.acc * y_dado.acc + z_dado.acc * z_dado.acc)) * RAD_TO_DEG;
  
  x_dado.setAngle(roll); // Set starting angle
  y_dado.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;
}

/* ------------------------------------------------------------- */
void mpuDados::getValoresSensorI2C(double dt){
  /* Atualize todos os valores */
  while (i2cRead(0x3B, i2cData, 14));
  x_dado.acc = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  y_dado.acc = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  z_dado.acc = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  temperatura = (int16_t)(i2cData[6] << 8) | i2cData[7];
  x_dado.giro = (int16_t)(i2cData[8] << 8) | i2cData[9];
  y_dado.giro = (int16_t)(i2cData[10] << 8) | i2cData[11];
  z_dado.giro = (int16_t)(i2cData[12] << 8) | i2cData[13];

  // Realiza a conversão para angulo
  // roll -> fi para x
  double roll  = atan2(y_dado.acc, z_dado.acc) * RAD_TO_DEG;
  double pitch = atan(-x_dado.acc / sqrt(y_dado.acc * y_dado.acc + z_dado.acc * z_dado.acc)) * RAD_TO_DEG;
  
  double gyroXrate = x_dado.giro / 131.0; // Convert to deg/s
  double gyroYrate = y_dado.giro / 131.0; // Convert to deg/s

  // Isso corrige o problema de transição quando o ângulo acelerômetro salta entre -180 e 180 graus
  if ((roll < -90 && x_dado.giroK > 90) || (roll > 90 && x_dado.giroK < -90)) {
    x_dado.setAngle(roll);
    compAngleX = roll;
    x_dado.giroK = roll;
    gyroXangle = roll;
  } else
    x_dado.giroK = x_dado.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(x_dado.giroK) > 90)
    gyroYrate = -gyroYrate; // Taxa de inversão, para que se ajuste à leitura restrita do acelerômetro
  y_dado.giroK = y_dado.getAngle(pitch, gyroYrate, dt);

  gyroXangle += gyroXrate * dt; // Calcular o ângulo do giroscópio sem nenhum filtro
  gyroYangle += gyroYrate * dt;


  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calcular o ângulo usando um filtro complementar
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Redefina o ângulo do giroscópio quando ele deriva demais
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = x_dado.giroK;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = y_dado.giroK;
}

/* ------------------------------------------------------------- */
void mpuDados::printSemfiltro(void){
  Serial.print(x_dado.acc); Serial.print("\t");
  Serial.print(y_dado.acc); Serial.print("\t");
  Serial.print(z_dado.acc); Serial.print("\t");
  Serial.print(x_dado.giro); Serial.print("\t");
  Serial.print(y_dado.giro); Serial.print("\t");
  Serial.print(z_dado.giro); Serial.print("\t");
  Serial.print("\t");
}

/* ------------------------------------------------------------- */
void mpuDados::printComfiltro(void){
  //Serial.print(x_dado.acc); Serial.print("\t");
  //Serial.print(y_dado.acc); Serial.print("\t");
  //Serial.print(z_dado.acc); Serial.print("\t");
  Serial.print(x_dado.giroK); Serial.print("\t");
  Serial.print(y_dado.giroK); Serial.print("\t");
  Serial.print("\t");
}

/* ------------------------------------------------------------- */
String mpuDados::vetDadosparaAnalise(void){
  String mens;
  mens = (String)((float)x_dado.acc) + "," + (String)((float)y_dado.acc) + "," + (String)((float)z_dado.acc) + "," + (String)((float)x_dado.giroK) + "," + (String)((float)y_dado.giroK);
  return mens;
}
