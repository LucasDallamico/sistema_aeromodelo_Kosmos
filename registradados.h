#ifndef REGISTRADADOS_H
#define REGISTRADADOS_H

#include <SPI.h>
#include <SD.h>

class registraSD{
private:
  String _NomeArq;
  File arq;
  char tp_abertura;
public:
  registraSD(String Nome);
  ~registraSD();
  bool verificaSD(void);
  int abrirArq(char tipo);
  void fechaArq(void);
  bool escreveSD(String mensagem);
  bool leituraSD(void);
};

#endif
