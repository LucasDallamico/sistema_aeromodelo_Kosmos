# *** sistema_aeromodelo_Kosmos ***

## Uma parceria BOTCEM e KOSMOS
Estudantes UFSC campus Joinville - SC, todos os métodos e ferramentas implementados neste projeto, têm como único objetivo a aprendizagem e ciência. Este projeto não possui fins comerciais.

## Objetivo:
	Desenvolver um sistema de ejeção de paraquedas e coleta de dados para o voo de um aeromodelo

## Etapas de projeto:
	3.Finalização

## Módulos implementados:
	- MPU
	- MicroSD

## Microcontrolador usado:
	- Teensy 4
	- Arduino pro mini 3.3V

# Documentação e implementações

## micro_sd.h e micro_sd.ino
	-> Possui métodos para registrar o conteúdo do voo. Precisa da biblioteca <SD.h> e <SPI.h>

    micro_sd(String Nome);
	Contrutor que recebe como paramentro o nome do arquivo que ira registrar o voou.
 
    ~micro_sd();
	Destructor, sem efeito
 
    void inicializa_sensor(void);
	Função que faz a primeira comunicação com o módulo microSD, ele vai verificar se há comnicação atravez da porta MISO/MOSI. Caso apresente um erro, uma mensagem com a descrição aparecerá na porta Serial ( UART ).
 
    void escreveSD(String mensagem);
	Recebe como paramentro uma string, que será gravada em uma limha do arquivo  de registro. a string vem da classe MPU.retorna_valores_p_registro.
    void check_point(void);
	Abre e fecha o arquivo quanto atingir uma quantidade maxima de dados, a quantidade de arquivos de buffer, é definida na sistema_aeromodelo_Kosmos.ino

## MPU.h e MPU.ino
	-> Possuem métodos para operar o conteúdo do MPU. Tem como requisitos os arquivos: I2C.ino, kalman.h e kalman.cpp. Precisa da biblioteca <Wire.h>.
	-> No MPU.h possui uma lista com os endereços para a quantidade máxima de gravidade que o módulo irá suportar durante o voo. Por padrão está 8 gravidades ( gravidaMax8g ), isso pode ser alterado no arquivo mpu.ino, linha 17: i2cData[3] = gravidaMax8g; // Seta a gravidade máxima como 8 G.
	-> Por padrão, o valor máximo do angulo de rotação é 90 deg, especificado através da linha 13: #define RESTRICT_PITCH // restrição a +- 90 deg, para obter analises com valores máximos até 180 deg, basta comentar essa linha.
	

    mpu(void);
	Construtor da classe MPU, apenas inicializa as variáveis.
    ~mpu(void);
	Destrutor da classe MPU, não tem efeito no código.
    void inicializa_sensor(void);
	Função que faz a primeira comunicação com o módulo MPU, ele vai verificar se há comnicação atravez da porta I2C. Caso apresente um erro, uma mensagem com a descrição aparecerá na porta Serial ( UART )
    
void get_rotacao_x_y(double *rotacaoX, double *rotacaoY, double *a_x, double *a_y, double *a_z);
	Função que obtém os dados do MPU (temperatura, aceleração, ângulos), recebe como parâmetro opcional, variáveis para retorno. Essa função trata todos os dados seguindo dois tipos de análises, para 90 deg e 180 deg. Por padrão, as rotações vão até 90 deg ( atenção ao sinal ).
Rotação em torno do eixo X ( rotacaoX );
Rotação em torno do eixo Y ( rotacaoY );
** Ambas possuem tratamento através do filtro de Kalman.
As acelerações em x,y,z são sem tratamento;
 
    void imprime_valores_rotacao(void);
	Imprime os valores de rotação nos eixos x,y. Os valores são com filtro de kalman.
	
    void imprime_valores_aceleracao(void);
	Imprime os valores das acelerações nos eixos x,y,z. Os valores não tem filtro
 
    String retorna_valores_p_registro(void);
	Função que retorna uma string contendo os dados armazenados da classe MPU, (rotação em x, rotação em y, aceleração em X, aceleração em y, aceleração em z). Foi criada para facilitar o registro no classe micro_sd.


## I2C.ino
	-> Possui uma implementação para operar com a comunicação I2C na IDE Arduino, usando a biblioteca <Wire>

## Kalman.h e Kalman.cpp
	-> Implementação do filtro de Kalman, usando um biblioteca desenvolvida por: https://github.com/TKJElectronics/KalmanFilter. A biblioteca que tem o kalman.h, tem como objetivo tratar os dados do módulo MPU (os ruídos). O retorno das dos dados são o giro em torno do eixo x e y.

## sistema_aeromodelo_Kosmos.ino

	-> Main do projeto, contém a void setup e void loop;
	-> Possui variáveis para a lógica do acionamento do paraquedas:

#define MAX_ACC_X       0.5   // m/s^2
#define MAX_ACC_Y       0.5
#define MAX_ACC_Z       0.5
#define MAX_Rotacao_X   5.0   // deg
#define MAX_Rotacao_Y   85.0	

	Esses são os valores minimos para acionamento do parquedas ( requer teste ). eles equivalem ao ponto mais alto da curva do foguete, simulando um instante t, onde impoteticamente seria a menor velocidade que o foquete alcansaria, e posição no espaço que estaria neste momento, impoteticamente horizontal ( ou quase ).
	As velocidades são dadas em m/s^2, e os angulos em deg ( ~0 até ~90 ).
