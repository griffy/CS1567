#ifndef CS1567_FIRFILTER_H
#define CS1567_FIRFILTER_H

#include <string>
#include <vector>

class FIRFilter {
public:
    FIRFilter(std::string fileName);
    int getOrder();
    void seedFromFile(std::string fileName);
    void seed(std::vector<float> *samples);
    float filter(float val);
private:
    int _order;
    std::vector<float> _coefficients;
    std::vector<float> _samples;
    unsigned int _nextSample;
    
    int _readTaps(std::string fileName);
};

#endif
