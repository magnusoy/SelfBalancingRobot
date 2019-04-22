
#include "Arduino.h"
#include "ComplementaryFilter.h"

ComplementaryFilter::ComplementaryFilter(const double HIGHPASS_COEFFICIENT=0.8, const double LOWPASS_COEFFICIENT=0.2) {}

void ComplementaryFilter::in(double gyro, double accel, int dt) {
	_gyro = gyro;
	_accel = accel;
	_dt = dt;
}

double ComplementaryFilter::out() {
	_filteredOutput = 0.8 * (_filteredOutput + _gyro * _dt / 1000000) + 0.2 * _accel;
	return _filteredOutput;
}