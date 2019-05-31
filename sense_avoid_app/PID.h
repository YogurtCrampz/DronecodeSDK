/*******************************************************************************************
 * Author: Julian Torres
 * File Name: PID.h
 * Purpose:
 *      To implement a PID controller that controls distance to object
 *      in stopping sequence by sending velocity commands.
 * Resources: https://gist.github.com/bradley219/5373998 
 *****************************************************************************************/

#ifndef PID_H
#define PID_H

class PID {
    public:
        PID(double dt, double max, double min, double K, double Kp, double Ki, double Kd) :
            _dt(dt),
            _max(max),
            _min(min),
            _K(K),
            _Kp(Kp),
            _Ki(Ki),
            _Kd(Kd),
            _pre_error(0),
            _integral(0)
        {
        }

        // Returns the manipulated variable given a setpoint and current process value
        double Calculate(double setpoint, double pv) {
            // Calculate error
            double error = setpoint - pv;

            // Proportional term
            double Pout = _Kp * error;

            // Integral term
            _integral += error * _dt;
            double Iout = _Ki * _integral;

            // Derivative term
            double derivative = (error - _pre_error) / _dt;
            double Dout = _Kd * derivative;

            // Calculate total output
            double output = (-1) *_K * (Pout + Iout + Dout);

            // Restrict to max/min
            if( output > _max )
                output = _max;
            else if( output < _min )
                output = _min;

            // Save error to previous error
            _pre_error = error;

            return output;
        }  

    private:
        double _dt;
        double _max;
        double _min;
        double _K;
        double _Kp;
        double _Ki;
        double _Kd;
        double _pre_error;
        double _integral;
};


#endif
