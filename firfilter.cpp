#include "firfilter.h"

#include <cstring>

FIRFilter::FIRFilter(int ftype) {
    _nextSample = 0;

    switch (ftype) {
    case WE:
        memcpy(_coefficients, WE_COEFFICIENTS, sizeof(_coefficients));
        break;
    case NS:
        memcpy(_coefficients, NS_COEFFICIENTS, sizeof(_coefficients));
        break;
    }

    for (int i = 0; i < NUM_TAPS; i++) {
        _samples[i] = 0;
    }
}

float FIRFilter::Filter(float val) {
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