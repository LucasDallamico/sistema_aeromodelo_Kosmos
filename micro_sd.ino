#include "micro_sd.h"

micro_sd::micro_sd(String Nome)
{
  this->_NomeArq = Nome;
  Serial.println(F("Inicializando biblioteca micro_sd ..."));
}

micro_sd::~micro_sd(){}

void micro_sd::inicializa_sensor(void)
{
  if (!SD.begin(4)) {
    Serial.println(F("Falha de comunicação com o modulo SD !!!"));
    while (1);
  }
  Serial.println(F("Comunicação com o Modulo SD efetuado."));
  delay(100);
  abreArq();
}

void micro_sd::abreArq(void)
{
  arq = SD.open(_NomeArq, FILE_WRITE);
  if (!arq){
    Serial.println(F("Erro ao abrir um arquivo no microSD !!!"));
    while(1);
  } else {
    Serial.println(F("Arquivo aberto !!!"));
  }
  delay(5);
}

void micro_sd::fechaArq(void){ arq.close(); }

void micro_sd::escreveSD(String mensagem){ arq.println(mensagem); }

void micro_sd::check_point(void)
{
  fechaArq();
  //Garantir que não haverá atropelo de operação
  delay(10); 
  abreArq();
}
