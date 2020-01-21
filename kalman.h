/* Essa versão foi tradudiza pelo alunos de graduação UFSC, usado para fins educaionais para a equipe de competição BOTCEM, Campus Joinville-SC
 * -> Editor Lucas Barros Dallamico

 codigo fonte: https://github.com/TKJElectronics/KalmanFilter
 */

#ifndef KALMAN_H
#define KALMAN_H

class Kalman {
private:
    // Variáveis de filtro Kalman
    float Q_angle;   // Variação do ruído do processo para o acelerômetro
    float Q_bias;    // Variação do ruído do processo para a polarização do giroscópio
    float R_measure; // Variação do ruído de medição - na verdade, é a variação do ruído de medição

    float angle;  // O ângulo calculado pelo filtro Kalman - parte do vetor de estado 2x1
    float bias;   // A polarização do giroscópio calculada pelo filtro Kalman - parte do vetor de estado 2x1
    float rate;   // Taxa imparcial calculada a partir da taxa e do viés calculado - é necessário chamar getAngle para atualizar a taxa

    float matrizCov[2][2]; // Matriz de covariância de erro - esta é uma matriz 2x2
public:
    // Contrutor
    Kalman();

    // O ângulo deve estar em graus e a taxa deve estar em graus por segundo e o tempo delta em segundos
    float getAngle(float newAngle, float newRate, float dt);

    void setAngle(float angle); // Usado para definir o ângulo, deve ser definido como o ângulo inicial
    float getRate(); // Retornar a taxa imparcial

    // Eles são usados para ajustar o filtro Kalman
    void setQangle(float Q_angle);
    /*
    setQbias(float Q_bias)
    O valor padrão (0,003f) está em Kalman.cpp
    Aumente isso para acompanhar mais de perto as informações,
    abaixe isso para suavizar o resultado do filtro kalman.
     */
    void setQbias(float Q_bias);

    void setRmeasure(float R_measure);

    float getQangle();
    float getQbias();
    float getRmeasure();

};

#endif
