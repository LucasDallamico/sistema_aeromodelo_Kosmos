# *** sistema_aeromodelo_Kosmos ***

## Uma parceria BOTCEM e KOSMOS
Estudantes UFSC campi Joinville - SC, todos os metodos e ferramentas implementados neste projeto, tem como único objetivo a aprendizagem e ciência. Este projeto não possui fins comerciais para venda.

## Objetivo:
    Desenvolver um sistema completo para o voou de um aeromodelo

## Etapas de projeto:
    3.Finalização

## Modulos implementados:
    - MPU
    - MicroSD

## Micro-controlador usado:
    - Tensiny 4

# Documentação e implementações

## micro_sd.h e micro_sd.ino
    -> Possui metodos para registrar o conteúdo do voou.
        - micro_sd(String Nome): controi a classe do modulo;
        - inicializa_sensor(void): verifica se o modulo está funcionado e prepara para operação;
        - escreveSD(String mensagem): escreve uma string em uma linha do arquivo;
        - check_point(void): garantir que uma contidade finita de arquivos estão guardados no microSd. Está operação tem que ser feita de tempos em tempos;

## MPU.h e MPU.ino
    -> Possuim metodos para operar o contéudo do MPU. Tem como requisistos os arquivos: I2C.ino, kalman.h e kalman.cpp.
    - inicializa_sensor(void): verifica se o modulo está funcionando e prepara para operação;
    - get_rotacao_x_y(double *rotacaoX, double *rotacaoY, double *a_x, double *a_y, double *a_z): Realiza a operação para obter os dados do MPU, ainda aplica a filtro de kalman para as rotações. O retorno é obtido por referências nos paramentros;
    - imprime_valores_rotacao(void): Imprime os valores das rotações de X e Y com o filtro de kalman;
    - imprime_valores_aceleracao(void): Imprime os valores das acelerações em x,y,z;
    - retorna_valores_p_registro(void): Retorna os valores rotaçãox,rotaçãoy,aceleração em x,aceleração em y,aceleração em z tudo junto, separado por virgula. Usado para a função escritaSD para registrar esses dados no micro SD;

## I2C.ino
    -> Possui uma implementação para operar com a comunicação I2C na IDE Arduino, usando a biblioteca <Wire>

## Kalman.h e Kalman.cpp
    -> Implementação do filtro de Kalman, usando um biblioteca desenvolvida por: https://github.com/TKJElectronics/KalmanFilter. A biblioteca que tem o kalman.h, tem como objetivo tratar os dados do modulo MPU ( os ruidos ). O retorno das dos dados são o giro entorno do eixo x e y.

