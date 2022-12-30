#ifndef kalman_h
#define kalman_h
#include "stm32f4xx.h"
#include <stdio.h>
typedef struct 
{
  float _err_measure;
  float _err_estimate;
  float _q;
  float _current_estimate;
  float _last_estimate;
  float _kalman_gain;

}KalmanFilter;

void SimpleKalmanFilterInit(KalmanFilter* kalman, float mea_e, float est_e, float q);
float updateEstimate(KalmanFilter* kalman, float mea);
void setMeasurementError(KalmanFilter* kalman, float mea_e);
void setEstimateError(KalmanFilter* kalman, float est_e);
void setProcessNoise(KalmanFilter* kalman, float q);
float getKalmanGain(KalmanFilter* kalman);
float getEstimateError(KalmanFilter* kalman);


#endif
