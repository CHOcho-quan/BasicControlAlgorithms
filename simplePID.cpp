//
//  simplePID.cpp
//
//
//  Created by 铨 on 2019/3/12.
//
//

#include <iostream>
#include <set>
#include <vector>
#include <string>
#include <stack>
#include <iomanip>
#include <cmath>
#include <queue>

using namespace std;

int sgn(double a)
{
    if (a >= 0) return 1;
    else return -1;
}

class LowPassFilter
{
/***
 A simple Low pass filter with cut-off frequency
 if cut-off frequency < 0 we consider the system not filtered
 ePow : Fourier transformation of a inertial part
 
 ***/
private:
    double cutOffFrequency;
    double ePow;
    double dt;
    double output;
public:
    LowPassFilter()
    {
        output = 0;
        dt = 0;
        ePow = 0;
        cutOffFrequency = 0;
    }
    
    LowPassFilter(double cof, double delta)
    {
        cutOffFrequency = cof;
        dt = delta;
        ePow = 1 - exp(-dt * cutOffFrequency);
    }
    
    double update(double input)
    {
        if (cutOffFrequency < 0) return input;
        return output += (input - output) * ePow;
    }
    
    double setCutOffFrequency(double cof)
    {
        cutOffFrequency = cof;
    }
    
    double getOutput()
    {
        return output;
    }
    
    double getCutOffFrequency()
    {
        return cutOffFrequency;
    }
};

class PID
{
/***
 This is a simple PID controller with clamp method & low pass filter
 Kp Ki Kd : The PID parameters
 max min : the actuator of the real system
 
 ***/
private:
    double Kp;
    double Ki;
    double Kd;
    double dt;
    double integral;
    double p_error;
    double max;
    double min;
    bool clamp;
    
    LowPassFilter lpf;
public:
    PID(double p, double i, double d, double t, double ma, double mi, double cof)
    {
        lpf = LowPassFilter(cof, t);
        Kp = p;
        Ki = i;
        Kd = d;
        dt = t;
        integral = 0;
        p_error = 0;
        max = ma;
        min = mi;
        clamp = false;
    }
    
    void setKp(double newKp)
    {
        Kp = newKp;
    }
    
    void setKi(double newKi)
    {
        Ki = newKi;
    }
    
    void setKd(double newKd)
    {
        Kd = newKd;
    }
    
    double getKp()
    {
        return Kp;
    }
    
    double getKi()
    {
        return Ki;
    }
    
    double getKd()
    {
        return Kd;
    }
    
    double actuator(double value)
    {
        if (value >= max) return max;
        if (value <= min) return min;
        return value;
    }
    
    double calculate(double setpoint, double processvalue)
    {
        double err = setpoint - processvalue;
        
        // P term
        double P = Kp * err;
        
        // I term
        if (!clamp) integral += err * dt;
        double I = Ki * integral;
        
        // D term
        lpf.update(err);
        err = lpf.getOutput();
        double error = (err - p_error) / dt;
        double D = Kd * error;
        p_error = err;
        
        double output = P + I + D;
        double actuated = actuator(output);
        
        clamp = ((actuated!=output) && (sgn(error) == sgn(output)));
        
        return actuated;
    }
    
    void reset()
    {
        p_error = 0;
        clamp = false;
        integral = 0;
    }
};

int main() {
    
    PID pid = PID(0.1, 0.01, 0.5, 0.1, 100, -100, -1);
    
    double val = 20;
    for (int i = 0; i < 100; i++) {
        double inc = pid.calculate(0, val);
        printf("val:% 7.3f inc:% 7.3f\n", val, inc);
        val += inc;
    }
    
    return 0;
}
