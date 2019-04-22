#ifndef _COMPLEMENTARYFILTER_H_
#define _COMPLEMENTARYFILTER_H_

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

class ComplementaryFilter {
public:
	ComplementaryFilter(const double HIGHPASS_COEFFICIENT=0.8, const double LOWPASS_COEFFICIENT=0.2);
	void in(double gyro, double accel, int dt);
	double out(void);
	void setDt(int dt);

private:
	const double _HIGHPASS_COEFFICIENT;
	const double LOWPASS_COEFFICIENT;
	int _dt;
	double _filteredOutput:
	double _gyro;
	double _accel;

	

};

#endif // _COMPLEMENTARYFILTER_H_



