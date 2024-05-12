#include "LowPassFilter.h"
#include <iostream>

LowPassFilter::LowPassFilter():
	output(0),
	ePow(0){}

LowPassFilter::LowPassFilter(double iCutOffFrequency, double iDeltaTime):
	output(0),
	ePow(1-exp(-iDeltaTime * 2 * M_PI * iCutOffFrequency))
{
	if (iDeltaTime <= 0){
		std::cout << "Warning: A LowPassFilter instance has been configured with 0 s as delta time.";
		ePow = 0;
	}
	if(iCutOffFrequency <= 0){
		std::cout << "Warning: A LowPassFilter instance has been configured with 0 Hz as cut-off frequency.";
		ePow = 0;
	}
}

double LowPassFilter::update(double input){
	return output += (input - output) * ePow;
}

double LowPassFilter::update(double input, double deltaTime, double cutoffFrequency){
	reconfigureFilter(deltaTime, cutoffFrequency); //Changes ePow accordingly.
	return output += (input - output) * ePow;
}

void LowPassFilter::reconfigureFilter(double deltaTime, double cutoffFrequency){
	if (deltaTime <= 0){
		std::cout << "Warning: A LowPassFilter instance has been configured with 0 s as delta time.";
		ePow = 0;
	}
	if(cutoffFrequency <= 0){
		std::cout << "Warning: A LowPassFilter instance has been configured with 0 Hz as cut-off frequency.";
		ePow = 0;
	}
	ePow = 1-exp(-deltaTime * 2 * M_PI * cutoffFrequency);
}