#include "KalmanFilter.h"
#include "math.h"

KalmanFilter::KalmanFilter(double inputError, double estimatedError, double processNoise) {
	_inputError = inputError;
	_estimatedError = estimatedError;
	_processNoise = processNoise;
}

void KalmanFilter::in(double input) { _input = input; }

double KalmanFilter::out() {
	_kalmanGain = _estimatedError / (_estimatedError + _inputError);
	_currentEstimate = _lastEstimate + _kalmanGain * (_input - _lastEstimate);
	_estimatedError = (1.0 - _kalmanGain) * _inputError + fabs(_lastEstimate - _currentEstimate)* _processNoise;
	_lastEstimate = _currentEstimate;

	return _currentEstimate;
}

void KalmanFilter::setInputError(double inputError) {_inputError = inputError; }

void KalmanFilter::setEstimateError(double estimatedError) { _estimatedError = estimatedError; }

void KalmanFilter::setProcessNoise(double processNoise) { _processNoise = processNoise; }

double KalmanFilter::getKalmanGain() { return _kalmanGain; }

double KalmanFilter::getEstimateError() { return _estimatedError; }
