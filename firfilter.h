#ifndef CS1567_FIRFILTER_H
#define CS1567_FIRFILTER_H

#define NUM_TAPS 5

// TODO: use actual coefficients
const float WE_COEFFICIENTS[NUM_TAPS] = {0.5, 0.5, 0.5, 0.5, 0.5};
const float NS_COEFFICIENTS[NUM_TAPS] = {0.5, 0.5, 0.5, 0.5, 0.5};

enum FilterType { 
    WE, 
    NS 
};

class FIRFilter {
public:
    FIRFilter(int ftype);
    float Filter(float val);
private:
    float _coefficients[NUM_TAPS];
    float _samples[NUM_TAPS];
    unsigned int _nextSample;
};

#endif