// LowPassFilter class example from Github
// https://github.com/jimmyberg/LowPassFilter
// GPL-2.0 License

#include <cmath>

class LowPassFilter{
public:
	//constructors
	LowPassFilter();
	LowPassFilter(double iCutOffFrequency, double iDeltaTime);
	//functions
	double update(double input);
	double update(double input, double deltaTime, double cutoffFrequency);
	//get and configure funtions
	double getOutput() const{return output;}
	void reconfigureFilter(double deltaTime, double cutoffFrequency);
private:
	double output;
	double ePow;
};