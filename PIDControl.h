#ifndef PIDCONTROL_H
#define PIDCONTROL_H

class PIDControl {

private:
    double dKp;    // Proportional gain
    double dKi;    // Integral gain
    double dKd;    // Differential gain
	double dTa;    // Sampling time
	double dEsum;  // Sum of errors
	double de_old;  // Previous error
	double dU;     // Control deviation
public:
	PIDControl(double Kp, double Ki, double Kd, double Ta);

	void calculateU(double dW, double dY);

	double getU() const;
};

#endif /* PIDCONTROL_H_ */