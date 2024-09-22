#ifndef KEYBOARDCONTROL_H
#define KEYBOARDCONTROL_H

#include <array>
#include <ncurses.h>
#include "InterfaceSIM.h"
#include "PIDControl.h"
#include <cmath>
#include <chrono>
#include <thread>

class KeyboardControl {
private:
    std::array<double, 2> m_desiredSpeed;  // 0 rightSpeed, 1 leftSpeed [-0.5,0.5](m/s)
    std::array<double, 2> m_actualSpeed;   // 0 rightSpeed, 1 leftSpeed [-0.5,0.5](m/s)
    std::array<int, 2> m_iMicros;
    
    InterfaceSIM m_simInterface;
    
    std::array<PIDControl, 2> m_pidControl;

public:
	static KeyboardControl *transferPointer;

    KeyboardControl();

    void SpeedProtect(double &speed, double adjustspeed);

    void SpeedProtect();
    void Communicate();
    void Step();

    static void transferFunction();

};




#endif // KEYBOARDCONTROL_H