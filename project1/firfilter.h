#ifndef CS1567_FIRFILTER_H
#define CS1567_FIRFILTER_H

#define NUM_TAPS 8

const float WE_COEFFICIENTS[NUM_TAPS] = {-.040798, .11034, .21151, .29248, .29248, .21151, .11034, -.040798};
const float NS_X_COEFFICIENTS[NUM_TAPS] = {.11086,.1371,.17947,.19554,.17947,.1371,.11086};
const float NS_Y_COEFFICIENTS[NUM_TAPS] = {.11086,.1371,.17947,.19554,.17947,.1371,.11086};
//const float NS_Y_COEFFICIENTS[NUM_TAPS] = {-.040798, .11034, .21151, .29248, .29248, .21151, .11034, -.040798};
const float NS_THETA_COEFFICIENTS[NUM_TAPS] = {-.040798, .11034, .21151, .29248, .29248, .21151, .11034, -.040798};

// TODO: better samples?
const float WE_SAMPLES[NUM_TAPS] = {14, 37, 14, 15, 33, 14, 15, 15};
const float NS_X_SAMPLES[NUM_TAPS] = {-8748, -8130, -7787, -7683, -7097, -6486, -6064, -5733};
const float NS_Y_SAMPLES[NUM_TAPS] = {-3938, -4382, -4301, -4069, -4941, -4860, -5092, -5310};
const float NS_THETA_SAMPLES[NUM_TAPS] = {1, 1, 1, 1, 1, 1, 1, 1};

enum FilterType { 
    WE, 
    NS_X,
    NS_Y,
    NS_THETA 
};

class FIRFilter {
public:
    FIRFilter(int ftype);
    void seed(const float samples[]);
    float filter(float val);
private:
	int _type;
    float _coefficients[NUM_TAPS];
    float _samples[NUM_TAPS];
    unsigned int _nextSample;
};

#endif
