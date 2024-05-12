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

    //intit filter 
    double init_filter(double input);

	//get and configure funtions
	double getOutput() const{return output_;}
	void reconfigureFilter(double deltaTime, double cutoffFrequency);
private:
	double output_;
	double ePow_;
};