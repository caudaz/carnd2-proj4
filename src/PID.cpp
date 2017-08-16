#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	PID::Kp = Kp;
	PID::Ki = Ki;
	PID::Kd = Kd;
	
	p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
}

void PID::UpdateError(double cte) {
    d_error = cte - p_error; // assumes dt=1
	i_error = i_error + cte; // assumes dt=1
	p_error = cte;
}

double PID::TotalError() {
    return p_error * Kp + d_error * Kd + i_error * Ki;
}

