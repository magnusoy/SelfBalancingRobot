#ifndef KalmanFilter_h
#define KalmanFilter_h

class KalmanFilter {
public:
	KalmanFilter(double inputError, double estimatedError, double processNoise);
	void in(double input);
	double out();
	void setInputError(double inputError);
	void setEstimateError(double estimatedError);
	void setProcessNoise(double processNoise);
	double getKalmanGain();
	double getEstimateError();

private:
	double _input;
	double _inputError;
	double _estimatedError;
	double _processNoise;
	double _currentEstimate;
	double _lastEstimate;
	double _kalmanGain;
	double _output;
};

#endif



