#include "registradados.h"
#include <stdlib.h>
/* _____________________________________________________________ */
//         IMPLEMENTAÇÃO DA CLASSE registraSD
/* ----------------------------------------------- */
registraSD::registraSD(String Nome){
  _NomeArq = Nome;
  Serial.print("Inicializando SD card ...");
  //arq = NULL; //-> blindar essa parte, não sei como
  tp_abertura = 'N';
}
/* ----------------------------------------------- */
registraSD::~registraSD(){}
/* ----------------------------------------------- */
bool registraSD::verificaSD(void)
{
  if (!SD.begin(4)) {
    Serial.println(F("Não foi possível inicializar o modulo SD !!!"));
    while (1); // trava o arduino
    return 0;
  }
  Serial.println(F("Modulo SD reconhecido"));
  return 1;
}
/* ----------------------------------------------- */
int registraSD::abrirArq(char tipo){
  // blindagem
  if (tipo != 'w' && tipo != 'r'){
    Serial.println(F("Caiu na blindagem -> abrirArq, no modulo SD"));
    while(1); // É erro para por aqui mesmo
    return 0;
  }
  if (tipo == 'w'){
    arq = SD.open(_NomeArq, FILE_WRITE);
  }
    
  else if (tipo == 'r'){
    arq = SD.open(_NomeArq);
  }

  if (arq){ // se foi possivel abrir arq
   tp_abertura = tipo;
   return 1;
  } else{
    Serial.println("Erro ao acessar o arquivo !!!");
    while(1); // para o arduino
    return 0; 
  }
}
/* ----------------------------------------------- */
void registraSD::fechaArq(void){ arq.close(); }
/* ----------------------------------------------- */
bool registraSD::escreveSD(String mensagem){
  if ( tp_abertura == 'w'){
    arq.println(mensagem); // escreve e pula linha no final
    return 1;
  }
  else{
    Serial.println(F("em fechaArq -> Abriu o arquivo com o tipo errado !!!"));
    return 0;
  }
}
/* ----------------------------------------------- */
bool registraSD::leituraSD(void){
  if (  tp_abertura == 'r'){
    //leia o arquivo até que não haja mais nada:
    while (arq.available()) {
      Serial.write(arq.read());
    }
    return 1;
  }
  else{
    Serial.println(F("rm leituraSD -> Abriu arquivo com o tipo errado !!!"));
    return 0;
  }
}
