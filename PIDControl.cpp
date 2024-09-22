#include "PIDControl.h"

PIDControl::PIDControl(double Kp, double Ki, double Kd, double Ta)
        :dKp(Kp), dKi(dKi), dKd(Kd), dTa(Ta), dEsum(0.0), de_old(0.0), dU(0.0)
        {}

void PIDControl::calculateU(double dW, double dY) {
    double de_new = dW - dY;    //current error
    dEsum += de_new;            //accumulative error   
    double dEdiff = (de_new - de_old) / dTa;    //Differential error
    //update
    de_old = de_new;
    dU = dKp * de_new + dKi * dTa * dEsum + dKd * dEdiff;  
}

double PIDControl::getU() const {
    return dU;
}






