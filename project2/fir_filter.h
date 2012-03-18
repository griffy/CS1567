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
    void seed(float value);
    float filter(float val);
	float getValue();
private:
    int _order;
	float _value;
    std::vector<float> _coefficients;
    std::vector<float> _samples;
    unsigned int _nextSample;
	
    
    int _readTaps(std::string fileName);
};

#endif
