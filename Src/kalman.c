
#include "kalman.h"
#include <math.h>

void SimpleKalmanFilterInit(KalmanFilter* kalman, float mea_e, float est_e, float q)
{
  kalman->_err_measure=mea_e;
  kalman->_err_estimate=est_e;
  kalman->_q = q;
}

float updateEstimate(KalmanFilter* kalman, float mea)
{
  kalman->_kalman_gain = kalman->_err_estimate/(kalman->_err_estimate + kalman->_err_measure);
  kalman->_current_estimate = kalman->_last_estimate + kalman->_kalman_gain * (mea - kalman->_last_estimate);
  kalman->_err_estimate =  (1.0 - kalman->_kalman_gain)*kalman->_err_estimate + fabs(kalman->_last_estimate-kalman->_current_estimate)*kalman->_q;
  kalman->_last_estimate=kalman->_current_estimate;

  return kalman->_current_estimate;
}

void setMeasurementError(KalmanFilter* kalman, float mea_e)
{
  kalman->_err_measure=mea_e;
}

void setEstimateError(KalmanFilter* kalman, float est_e)
{
  kalman->_err_estimate=est_e;
}

void setProcessNoise(KalmanFilter* kalman, float q)
{
  kalman->_q=q;
}

float getKalmanGain(KalmanFilter* kalman) {
  return kalman->_kalman_gain;
}

float getEstimateError(KalmanFilter* kalman) {
  return kalman->_err_estimate;
}