#ifndef CS1567_FIRFILTER_H
#define CS1567_FIRFILTER_H

#define NUM_TAPS 8

const float WE_COEFFICIENTS[NUM_TAPS] = {-.040798, .11034, .21151, .29248, .29248, .21151, .11034, -.040798};
const float NS_X_COEFFICIENTS[NUM_TAPS] = {-.040798, .11034, .21151, .29248, .29248, .21151, .11034, -.040798};
const float NS_Y_COEFFICIENTS[NUM_TAPS] = {-.040798, .11034, .21151, .29248, .29248, .21151, .11034, -.040798};
const float NS_THETA_COEFFICIENTS[NUM_TAPS] = {-.040798, .11034, .21151, .29248, .29248, .21151, .11034, -.040798};

enum FilterType { 
    WE, 
    NS_X,
    NS_Y,
    NS_THETA 
};

class FIRFilter {
public:
    FIRFilter(int ftype);
    float filter(float val);
private:
    float _coefficients[NUM_TAPS];
    float _samples[NUM_TAPS];
    unsigned int _nextSample;
};

#endif
