/**
 * fir_filter.cpp
 * 
 * @brief 
 * 		This class is used for applying a FIR filter to arbitrary data
 * 
 * @author
 * 		Shawn Hanna
 * 		Tom Nason
 * 		Joel Griffith
 * 
 * @date
 * 		created - 2/2/2012
 * 		modified - 2/25/2012
 **/

#include "fir_filter.h"

#include <cstdlib>
#include <fstream>

FIRFilter::FIRFilter(std::string fileName) 
: _order(0), _coefficients(), _samples(), _nextSample(0) {
    // read in the taps from the specified .ffc file and set the order
    int numTaps = _readTaps(fileName);
    _order = numTaps-1;
}

/* Returns the order of the filter (taps-1) */
int FIRFilter::getOrder() {
    return _order;
}
/* Returns the current value of the filter */
float FIRFilter::getValue() {
    return _value;
}

/* Seeds the samples array with values from a file */
void FIRFilter::seedFromFile(std::string fileName) {
    std::vector<float> samples(getOrder()+1);
    std::ifstream f(fileName.c_str());
    std::string line;
    while (std::getline(f, line)) {
        samples.push_back(atof(line.c_str()));
    }
    seed(&samples);
}

/* Seeds the samples array with the given values */
void FIRFilter::seed(std::vector<float> *samples) {
    int numSamples = getOrder()+1;
    if (samples->size() < numSamples) {
        numSamples = samples->size();
    }

    for (int i = 0; i < numSamples; i++) {
        _samples[i] = (*samples)[i];
    }
    _nextSample = 0;
}

void FIRFilter::seed( float value ){
	for(int i=0; i<_order; i++){
		_samples[i] = value;
	}
}

float FIRFilter::filter(float val) {
    float sum = 0;
    int i, j;

    // add this value as the next sample
    _samples[_nextSample] = val;
    for (i = 0, j = _nextSample; i < getOrder()+1; i++) {
        sum += _coefficients[i] * _samples[j++];
        // if j has passed our filter size, reset it to 0
        if (j == getOrder()+1) {
            j = 0;
        }
    }

    // the next sample should be put at 0 if we've reached our size
    if (++_nextSample == getOrder()+1) {
        _nextSample = 0;
    }
	_value=sum;
    return sum;
}

/* Reads in the taps (coefficients) from a file, line by line */
int FIRFilter::_readTaps(std::string fileName) {
    std::ifstream f(fileName.c_str());
    std::string line;
    int numTaps = 0;
    while (std::getline(f, line)) {
        _coefficients.push_back(atof(line.c_str()));
        numTaps++;
    }
    _coefficients.resize(numTaps);
    _samples.resize(numTaps);
    return numTaps;
}
