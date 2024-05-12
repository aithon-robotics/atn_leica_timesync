#include "LowPassFilter.h"
#include <iostream>

LowPassFilter::LowPassFilter():
	output_(0),
	ePow_(0){}

LowPassFilter::LowPassFilter(double iCutOffFrequency, double iDeltaTime):
	output_(0),
	ePow_(1-exp(-iDeltaTime * 2 * M_PI * iCutOffFrequency))
{
	if (iDeltaTime <= 0){
		std::cout << "Warning: A LowPassFilter instance has been configured with 0 s as delta time.";
		ePow_ = 0;
	}
	if(iCutOffFrequency <= 0){
		std::cout << "Warning: A LowPassFilter instance has been configured with 0 Hz as cut-off frequency.";
		ePow_ = 0;
	}
}

double LowPassFilter::update(double input){
	return output_ += (input - output_) * ePow_;
}

//TM: init Filter
double LowPassFilter::init_filter(double input){
	return output_ = input;
}

double LowPassFilter::update(double input, double deltaTime, double cutoffFrequency){
	reconfigureFilter(deltaTime, cutoffFrequency); //Changes ePow_accordingly.
	return output_ += (input - output_) * ePow_;
}

void LowPassFilter::reconfigureFilter(double deltaTime, double cutoffFrequency){
	if (deltaTime <= 0){
		std::cout << "Warning: A LowPassFilter instance has been configured with 0 s as delta time.";
		ePow_ = 0;
	}
	if(cutoffFrequency <= 0){
		std::cout << "Warning: A LowPassFilter instance has been configured with 0 Hz as cut-off frequency.";
		ePow_ = 0;
	}
	ePow_ = 1-exp(-deltaTime * 2 * M_PI * cutoffFrequency);
}