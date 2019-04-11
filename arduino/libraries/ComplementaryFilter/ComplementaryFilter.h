#ifndef ComplementaryFilter
#define ComplementaryFilter

class ComplementaryFilter {
public:
	ComplementaryFilter(double filterAngle, double timeConstant, double dt);
	void in(double angle, double rate);
	double out();
	void setFilterAngle(double filterAngle);
	void setDt(double dt);
	void setTimeConstant(double timeConstant);

private:
	double filterTerm0;
	double filterTerm1;
	double filterTerm2;
	double _timeConstant;
	double _angle;
	double _rate;
	double _lastCompTime = 0;
	double _filterAngle;
	double _dt;
	double _output;
};

#endif



