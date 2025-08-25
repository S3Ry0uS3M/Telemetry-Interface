#pragma once

#include <limits>
#include <cmath>

class PIDController
{
public:
	PIDController(double kp, double ki, double kd, double outputMin = std::numeric_limits<double>::min(), double outputMax = std::numeric_limits<double>::max()) {
		this->Kp = kp;
		this->Ki = ki;
		this->Kd = kd;
		this->OutputMax = outputMax;
		this->OutputMin = outputMin;
	};
	~PIDController() {};

public:
	double getValue() const { return output; };

protected:
	double Kp, Ki, Kd;
	double Integral = 0;
	double LastError = 0;
	double OutputMin = std::numeric_limits<double>::min();
	double OutputMax = std::numeric_limits<double>::max();

private:
	double output = 0;

public:
	double update(double setpoint, double measured, double dt)
	{
		double error = setpoint - measured;
		Integral += error * dt;
		double derivative = (error - LastError) / dt;
		LastError = error;

		double control = Kp * error + Ki * Integral + Kd * derivative;
		output = std::fmax(OutputMin, std::fmin(OutputMax, control));

		return output;
	}
};