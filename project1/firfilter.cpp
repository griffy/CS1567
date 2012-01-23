#include "firfilter.h"

#include <cstring>

FIRFilter::FIRFilter(int ftype) {
    _type = ftype;
    _nextSample = 0;

    // FIXME: is the copying necessary when the coeff arrays are const?
    switch (_type) {
    case WE:
        memcpy(_coefficients, WE_COEFFICIENTS, sizeof(_coefficients));
        seed(WE_SAMPLES);
        break;
    case NS_X:
        memcpy(_coefficients, NS_X_COEFFICIENTS, sizeof(_coefficients));
        seed(NS_X_SAMPLES);
        break;
    case NS_Y:
        memcpy(_coefficients, NS_Y_COEFFICIENTS, sizeof(_coefficients));
        seed(NS_Y_SAMPLES);
        break;
    case NS_THETA:
        memcpy(_coefficients, NS_THETA_COEFFICIENTS, sizeof(_coefficients));
        seed(NS_THETA_SAMPLES);
        break;
    default:
        memcpy(_coefficients, WE_COEFFICIENTS, sizeof(_coefficients));
        seed(WE_SAMPLES);    
    }
}

void FIRFilter::seed(const float samples[]) {
    for (int i = 0; i < NUM_TAPS; i++) {
        _samples[i] = samples[i];
    }
    _nextSample = 0;
}

float FIRFilter::filter(float val) {
    float sum = 0;
    int i, j;

    _samples[_nextSample] = val;
    for (i = 0, j = _nextSample; i < NUM_TAPS; i++) {
        sum += _coefficients[i] * _samples[j++];
        if (j == NUM_TAPS) {
            j = 0;
        }
    }

    if (++_nextSample == NUM_TAPS) {
        _nextSample = 0;
    }

    return sum;
}
