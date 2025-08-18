#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <cmath>
#include <ros/ros.h>

#include "following_controller/pid_controller.h"

namespace FOLLOWING
{
    PID_controller::PID_controller(double kp, double ki, double kd, double deadband,
                                   double min_pid_out, double max_pid_out,
                                   double min_err_int, double max_err_int,
                                   double dt, std::string controller_name)
        : kp_(kp), ki_(ki), kd_(kd), deadband_(deadband), dt_(dt),
          min_pid_out_(min_pid_out), max_pid_out_(max_pid_out),
          min_err_int_(min_err_int), max_err_int_(max_err_int),
          err_cur_(0.0), err_last_(0.0), err_int_(0.0),
          err_der_(0.0), pid_ref_(0.0), pid_out_(0.0)
    {
        if (dt <= 0.0)
        {
            throw std::invalid_argument("dt < 0");
        }

        if (min_pid_out >= max_pid_out)
        {
            throw std::invalid_argument("min_pid_out > max_pid_out");
        }

        if (min_err_int >= max_err_int)
        {
            throw std::invalid_argument("min_err_int > max_err_int");
        }

        ROS_INFO_STREAM("PID controller: "<< controller_name << " is ready.");
    }

    PID_controller::~PID_controller()
    {
    }

    double PID_controller::calc_output(double input, double dt)
    {
        // Ensure time step is positive
        if (dt <= 0.0) {
            ROS_WARN_STREAM("Invalid time step: " << dt << ", using previous value: " << dt_);
            dt = dt_;
        } else {
            dt_ = dt;
        }

        // Calculate current error
        err_cur_ = pid_ref_ - input;

        // Deadband handling: error is considered 0 within deadband
        const double deadband_half = deadband_ / 2.0;
        const double bounded_error = (std::abs(err_cur_) <= deadband_half) ? 0.0 : err_cur_;

        // Improve derivative calculation with low-pass filter to reduce noise
        const double derivative = (bounded_error - err_last_) / dt_;
        // Update err_der_ member variable
        err_der_ = 0.8 * err_der_ + 0.2 * derivative; // First-order low-pass filter

        // Calculate integral term with integral windup protection
        if (bounded_error != 0.0) { // Only accumulate integral when error is outside deadband
            err_int_ += bounded_error * dt_;
            err_int_ = std::clamp(err_int_, min_err_int_, max_err_int_);
        }

        // Calculate PID terms
        const double p_term = kp_ * bounded_error;
        const double i_term = ki_ * err_int_;
        const double d_term = kd_ * err_der_;
        double output = p_term + i_term + d_term;

        // Limit output range
        output = std::clamp(output, min_pid_out_, max_pid_out_);

        // Store current error for next calculation
        err_last_ = bounded_error;
        pid_out_ = output;

        // Optional: Output PID parameters and terms for debugging
        // ROS_INFO_STREAM("kP: " << kp_ << " kI: " << ki_ << " kD: " << kd_);
        // ROS_INFO_STREAM("P_term: " << p_term << " I_term: " << i_term << " D_term: " << d_term);

        return output;
    }
}