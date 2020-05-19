#include "MPU.h"
#include "micro_sd.h"

/* ------- LIMITE DA ALTURA ----------- */
// Deverão ser ajustados depois de testes
// Analises restristas a 90 deg ( pode ser alterada para 180 deg )
#define MAX_ACC_X       0.5   // m/s^2
#define MAX_ACC_Y       0.5
#define MAX_ACC_Z       0.5
#define MAX_Rotacao_X   5.0   // deg
#define MAX_Rotacao_Y   85.0

void logica_do_foguete_caindo(void);

/* ------- PORTAS USADAS -------------- */
#define ACIONADOR 3 // porta do paraquedas
#define LED_1     7
#define LED_2     6
#define Buzzer    2

/* ------------- MAIN ---------------- */

// Cria as classes dos modulos
mpu my_MPU;
micro_sd my_MicroSd("dados.csv");

// Variaveis de manipulação da main
int check_save = 0; // check point dos dados do microSD
double rotacao_X = 0.0, rotacao_Y = 0.0;
double accx = 0.0, accy = 0.0, accz = 0.0;

void setup()
{
  // Inicializando portas de comunicação
  Serial.begin(115200);
  Wire.begin();
  // Configuração da frequência para o I2C
  #if ARDUINO >= 157
    Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  #else
    TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
  #endif

  // Verifica se os modulos estão operantes
  my_MPU.inicializa_sensor();
  my_MicroSd.inicializa_sensor();

  // Prepara um cabeçalho para o arquivo
  my_MicroSd.escreveSD("rotacaoX,rotacaoY,accX,accY,accZ"); 

  // Setando as portas para zero - Usadas
  digitalWrite(ACIONADOR, LOW);
  digitalWrite(LED_1, LOW);
  digitalWrite(LED_2, LOW);
  digitalWrite(Buzzer, LOW);  
}

void loop()
{
  my_MPU.get_rotacao_x_y(&rotacao_X,&rotacao_Y,&accx,&accy,&accz);
  delay(10);
  /*
  //Imprimir os valores atravez do serial monitor
  my_MPU.imprime_valores_rotacao();
  my_MPU.imprime_valores_aceleracao();
  delay(10);
  */

  logica_do_foguete_caindo();
  
  // Registra no micro sd os dados do mpu
  my_MicroSd.escreveSD(my_MPU.retorna_valores_p_registro());
  delay(10);
  // Checkpoint para dados
  if (check_save == 20){
    Serial.println(F("Salvando dados ate o momento ..."));
    my_MicroSd.check_point();
    delay(10);
    check_save = 0;
  } else {
    check_save++;
  }
}

void logica_do_foguete_caindo(void)
{
  if ( rotacao_X < MAX_Rotacao_X && rotacao_Y < MAX_Rotacao_Y){
    if ( accz < MAX_ACC_Z || accx < MAX_ACC_X){
      // Aciona o paraquedas
      digitalWrite(ACIONADOR, HIGH);
    }
  }
}
