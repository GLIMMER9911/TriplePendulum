#ifndef PID_HPP
#define PID_HPP

#include <iostream>
#include <vector>
#include "export.h"

/*
	calculate PID 

*/
class PIDController {
public:
	void setTarget(double targetValue);
	double calculateP(double value);
	double calculatePD(double value, double dt);
	double calculatePI(double value, double dt);
	double calculatePID(double value, double dt);


	PIDController(double p,double i,double d);
	~PIDController();

private:
	double p_;
	double i_;
	double d_;
	double target;
	double preError;
	double integral;

};


#endif 