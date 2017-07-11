#include "PID.h"


PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
}

void PID::UpdateError(double cte) {
    d_error_  = cte - p_error_;
    p_error_  = cte;
    i_error_ += cte;
}

double PID::TotalError() {
    return (-Kp_ * p_error_) + (-Kd_ * d_error_) + (-Ki_ * i_error_);
}

