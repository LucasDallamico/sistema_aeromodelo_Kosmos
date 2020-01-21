
/*
Versão desenvolvida para o BOTCEM UFSC - JOINVILLE
*/
#include "MPU.h" // Classe das implementações do mpu
#include "registradados.h" // classe do cartao SD

/* _____________________________________________________________ */
//                          MAIN
// Cria as classes dos modulos, o filtro de kalman esta implementado dentro da classe mpuDados
uint32_t timer;
mpuDados MPU_1;
registraSD moduloSd("coleta.txt"); // o contrutor do objeto recebe como parametro o nome do arquivo que ira armazenar os dados
int i = 0; // itera��es de dados

#define ACIONADOR 8 // Porta digital usada para acionar o motor que ira ejetar o paraquedas
bool logicaParaquedas(void);

void setup()
{
  digitalWrite(ACIONADOR, LOW);
  Serial.begin(9600);
  //while(!Serial); // Garante que a execu��o do c�digo s� aconte�a quando for aberto o monitor s�rial, na implementa��o final tem que retirar

  /* Cada classe possui suas blindagens e verifica��es de integridade ( se o modulo esta ou n�o funcionando ), basta chamar as fun��es a seguir
   */

  MPU_1.inicia_comun_i2c(); // inicializa a comuni��o com o modulo
  MPU_1.sensorFunciona(); // Verifica se o sensor funciona
  delay(100); // Aguarde o sensor estabilizar

  moduloSd.verificaSD(); // inicializa sd
  delay(10);
  moduloSd.abrirArq('w'); // Abre o arquivo para escrita
  delay(100);

  MPU_1.valoresIniciais(); // Confirgura para radianos ou graus
  timer = micros(); // usado para fazer contas com o clock do microcontrolador

  /* Para analise de dados em python, a primeira linha do arquivo contem as configuracoes de como os dados estaram dispostos no arquivo
   */
  String aux = "accX,accY,accZ,giroXK,giroYK";
  moduloSd.escreveSD(aux); // funcao que escreve uma string no arquivo

}

void loop()
{
  double deltaTima = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  /*
    Obtem os valores do MPU usando a comunica��o I2C
   */
  MPU_1.getValoresSensorI2C(deltaTima);
  MPU_1.printComfiltro(); // Mostra no serial monitor os valores com filtro de kalman ( pode ser alterado para mostrar outros dados )
  //MPU_1.printSemfiltro();
  Serial.print("\r\n"); // tabula��o no serial monitor
  delay(50); // estabilizar
  /* Registrar no SD
   vetDadosparaAnalise -- > retorna os principais dados em uma string separado por virgula
   */
  String aux = MPU_1.vetDadosparaAnalise();
  moduloSd.escreveSD(aux);

  // Checkpoint para dados
  if (i % 10 == 0){
    Serial.print(F("Salvando dados ate o momento ..."));
    moduloSd.fechaArq();
    delay(10);
    moduloSd.abrirArq('w');
    Serial.println(F("OK"));
  }
  i++;

  if (logicaParaquedas() == 1)
    digitalWrite(ACIONADOR, HIGH);
}

/*
  Para fins de organização, aqui fica a logica para o acionamento
  do motor que ira ejetar o paraquedas
*/
bool logicaParaquedas(void)
{
  return false;
}
