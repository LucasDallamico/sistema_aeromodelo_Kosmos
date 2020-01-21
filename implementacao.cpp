/* Versão traduzida
 */

#include "Kalman.h"
/* ____________________________________________________________________ */
Kalman::Kalman() {
    // Vamos definir as variáveis assim, estas também podem ser ajustadas pelo usuário
    Q_angle = 0.001f;
    Q_bias = 0.003f;
    R_measure = 0.03f;

    angle = 0.0f; // Redefinir o ângulo
    bias = 0.0f; // Redefinir bias

    /* Como assumimos que o viés é 0 e sabemos o ângulo inicial (use setAngle),
    a matriz de covariância de erro é definida da seguinte forma
    consulte: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical */
    matrizCov[0][0] = 0.0f;
    matrizCov[0][1] = 0.0f;
    matrizCov[1][0] = 0.0f;
    matrizCov[1][1] = 0.0f;
};

// O ângulo deve estar em graus e a taxa deve estar em graus por segundo e o tempo delta em segundos
float Kalman::getAngle(float newAngle, float newRate, float dt) {
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Equações discretas de atualização do tempo do filtro de Kalman - Atualização do tempo ("Previsão")
    // Update xhat - projete o estado à frente
    /* Degrau 1 */
    rate = newRate - bias;
    angle += dt * rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    matrizCov[0][0] += dt * (dt*matrizCov[1][1] - matrizCov[0][1] - matrizCov[1][0] + Q_angle);
    matrizCov[0][1] -= dt * matrizCov[1][1];
    matrizCov[1][0] -= dt * matrizCov[1][1];
    matrizCov[1][1] += Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = matrizCov[0][0] + R_measure; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = matrizCov[0][0] / S;
    K[1] = matrizCov[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = newAngle - angle; // Angle difference
    /* Step 6 */
    angle += K[0] * y;
    bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = matrizCov[0][0];
    float P01_temp = matrizCov[0][1];

    matrizCov[0][0] -= K[0] * P00_temp;
    matrizCov[0][1] -= K[0] * P01_temp;
    matrizCov[1][0] -= K[1] * P00_temp;
    matrizCov[1][1] -= K[1] * P01_temp;

    return angle;
};

void Kalman::setAngle(float angle) { this->angle = angle; }; // Used to set angle, this should be set as the starting angle
float Kalman::getRate() { return this->rate; }; // Return the unbiased rate

/* These are used to tune the Kalman filter */
void Kalman::setQangle(float Q_angle) { this->Q_angle = Q_angle; };
void Kalman::setQbias(float Q_bias) { this->Q_bias = Q_bias; };
void Kalman::setRmeasure(float R_measure) { this->R_measure = R_measure; };

float Kalman::getQangle() { return this->Q_angle; };
float Kalman::getQbias() { return this->Q_bias; };
float Kalman::getRmeasure() { return this->R_measure; };
