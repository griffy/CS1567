#include "firfilter.h"

#include <cstring>

FIRFilter::FIRFilter(int ftype) {
    _nextSample = 0;

    // FIXME: is the copying necessary when the coeff arrays are const?
    switch (ftype) {
    case WE:
        memcpy(_coefficients, WE_COEFFICIENTS, sizeof(_coefficients));
        break;
    case NS_X:
        memcpy(_coefficients, NS_X_COEFFICIENTS, sizeof(_coefficients));
        break;
    case NS_Y:
        memcpy(_coefficients, NS_Y_COEFFICIENTS, sizeof(_coefficients));
        break;
    case NS_THETA:
        memcpy(_coefficients, NS_THETA_COEFFICIENTS, sizeof(_coefficients));
        break;
    default:
        memcpy(_coefficients, WE_COEFFICIENTS, sizeof(_coefficients));
    }

    for (int i = 0; i < NUM_TAPS; i++) {
        _samples[i] = 0;
    }
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
