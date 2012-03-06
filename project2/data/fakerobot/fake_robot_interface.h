// Note: This file purposely does not abide by the style guidelines
#ifndef CS1567_FAKEROBOTINTERFACE_H
#define CS1567_FAKEROBOTINTERFACE_H

#include <string>
#include <vector>

class FakeRobotInterface {
public:
    FakeRobotInterface(std::string address, int id);
    void reset(); // custom method to start over
    int update(void);
    void reset_state(void);
    int Move(int movement, int speed);
    int getWheelEncoder(int wheel);
    bool IR_Detected(void);
    int X(void);
    int Y(void);
    float Theta(void);
    int RoomID(void);
private:
    int _curSample;
    std::vector<std::string> _weSamples;
    std::vector<std::string> _nsSamples;

    std::vector<std::string> _getWESamples();
    std::vector<std::string> _getNSSamples();
};

#endif
