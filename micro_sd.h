#ifndef MICRO_SD_H
#define MICRO_SD_H

#include <SPI.h>
#include <SD.h>
#include <stdlib.h>

class micro_sd{
  public:
    micro_sd(String Nome);
    ~micro_sd();
    void inicializa_sensor(void);
    void escreveSD(String mensagem);
    void check_point(void);

  private:
    String _NomeArq;
    File arq;
    void abreArq(void);
    void fechaArq(void);
};

#endif
