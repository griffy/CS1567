// Note: This file purposely does not abide by the style guidelines
#include "fakerobotinterface.h"
#include <robot_if++.h>
#include <fstream>

#define DEFAULT_ROOM 2

FakeRobotInterface::FakeRobotInterface(std::string address, int id) {
    _weSamples = _getWESamples();
    _nsSamples = _getNSSamples();
    _curSample = 0;
}

int FakeRobotInterface::update(void) {
    // move to the next sample point
    _curSample++;
    if (_curSample > _weSamples.size()-1 ||
        _curSample > _nsSamples.size()-1) {
        return RI_RESP_FAILURE;
    }
    return RI_RESP_SUCCESS;
}

void FakeRobotInterface::reset_state(void) {
}

int FakeRobotInterface::Move(int movement, int speed) {
    return RI_RESP_SUCCESS;
}

int FakeRobotInterface::getWheelEncoder(int wheel) {
    std::string sample = _weSamples[_curSample];
    std::stringstream sampleStream(sample);
    std::string point;
    // let control drop through to call right
    // amount of getlines for each value
    switch (wheel) {
    case RI_WHEEL_REAR:
        std::getline(sampleStream, point, ',');
    case RI_WHEEL_RIGHT:
        std::getline(sampleStream, point, ',');
    case RI_WHEEL_LEFT:
        std::getline(sampleStream, point, ',');
        break;
    default:
        std::getline(sampleStream, point, ',');
    }
    return atoi(point.c_str());
}

bool FakeRobotInterface::IR_Detected(void) {
    return false;
}

int FakeRobotInterface::X(void) {
    std::string sample = _nsSamples[_curSample];
    std::stringstream sampleStream(sample);
    std::string point;
    std::getline(sampleStream, point, ',');
    return atoi(point.c_str());
}

int FakeRobotInterface::Y(void) {
    std::string sample = _nsSamples[_curSample];
    std::stringstream sampleStream(sample);
    std::string point;
    std::getline(sampleStream, point, ',');
    std::getline(sampleStream, point, ',');
    return atoi(point.c_str());
}

float FakeRobotInterface::Theta(void) {
    std::string sample = _nsSamples[_curSample];
    std::stringstream sampleStream(sample);
    std::string point;
    std::getline(sampleStream, point, ',');
    std::getline(sampleStream, point, ',');
    std::getline(sampleStream, point, ',');
    return atof(point.c_str());
}

int FakeRobotInterface::RoomID(void) {
    return DEFAULT_ROOM;
}

std::vector<std::string> FakeRobotInterface::_getWESamples() {
    std::ifstream f("we_samples.dat");
    std::vector<std::string> samples;
    std::string line;
    while (std::getline(f, line)) {
        samples.push_back(line);
    }
    return samples;
}
        
std::vector<std::string> FakeRobotInterface::_getNSSamples() {
    std::ifstream f("ns_samples.dat");
    std::vector<std::string> samples;
    std::string line;
    while (std::getline(f, line)) {
        samples.push_back(line);
    }
    return samples;
}

