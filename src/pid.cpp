
#include "pid.hpp"
/*
PID Controller
	1. calculate P, PI, PD, PID controller;
	2. P = Kp*error;
	3. PI = Kp*error + Ki*integral;
	4. PD = Kp*error + Kd*preError;
	5. PID = Kp*error + Ki*integral + Kd*preError;
*/

void PIDController::setTarget(double targetValue) {
	target = targetValue;
}


double PIDController::calculateP(double value) {
	double error = target - value;
	double pTerm = error * p_;
	return pTerm;
}

double PIDController::calculatePI(double value, double dt) {
    
	// calculate P
	double error = target - value;
	double pTerm = error * p_;

	// calculate I
	integral += error * dt;
	double iTerm = integral * i_;

	double pi_Term = iTerm + pTerm;


	return pi_Term;
}

double PIDController::calculatePD(double value, double dt) {

	// calculate P
	double error = target - value;
	double pTerm = error * p_;

	// calculate D
	double dTerm = d_ * (error - preError) / dt;
	preError = error;

	double pd_Term = dTerm + pTerm;

	return pd_Term;
}


double PIDController::calculatePID(double value, double dt) {


	// calculate P
	double error = target - value;
	double pTerm = error * p_;

	// calculate I
	integral += error * dt;
	double iTerm = integral * i_;

	// calculate D
	double dTerm = d_ * (error - preError) / dt;
	preError = error;

	double pid_Term = dTerm + pTerm + iTerm;

	return pid_Term;
}


PIDController::PIDController(double p, double i, double d) {
	p_ = p;
	i_ = i;
	d_ = d;
	target = 0;
	preError = 0;
	integral = 0;
}

PIDController:: ~PIDController() = default;
