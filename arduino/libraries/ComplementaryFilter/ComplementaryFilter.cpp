
#include "Arduino.h"
#include "ComplementaryFilter.h"

ComplementaryFilter::ComplementaryFilter(double filterAngle, double timeConstant, double dt) {
	_filterAngle = filterAngle;
	_dt = dt;
}

ComplementaryFilter::in(double angle, double rate) {
	_angle = angle;
	_rate = rate;
}

ComplementaryFilter::out() {
	filterTerm0 = (_angle - _filterAngle) * _timeConstant * _timeConstant;
	filterTerm2 += filterTerm0 * _dt;
	filterTerm1 = filterTerm2 + ((_angle - _filterAngle) * 2 * _timeConstant) + rate;
	filterAngle = (filterTerm1 * _dt) + _filterAngle;
	_output = _filterAngle;
	lastCompTime = millis();
	return _output;
}

ComplementaryFilter::setFilterAngle(double filterAngle) {
	_filterAngle = filterAngle;
}

ComplementaryFilter::setDt(double dt) {
	_dt = dt;
}

ComplementaryFilter::setTimeConstant(double timeConstant) {
	_timeConstant = timeConstant;
}