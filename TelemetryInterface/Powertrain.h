#pragma once

#include "Motor.h"
#include "Battery.h"
#include "Inverter.h"
#include "PIDController.h"

class Powertrain
{
public:
	Powertrain() {
		motor = new Motor();
		battery = new Battery();
		inverter = new Inverter(motor->getMaxCurrent(motor->getMaxTorque(), battery->getVdcMax()/std::sqrt(3)));
		pidDriver = new PIDController(100, 5, 0, -4000, 4000);

		double VdcMax = battery->getVdcMax();
		pidId = new PIDController(1, 10, 0, -VdcMax, VdcMax);
		pidIq = new PIDController(1, 0.1, 0, -VdcMax, VdcMax);
	};
	~Powertrain() {};

public:
	Motor* motor;
	Battery* battery;
	Inverter* inverter;
	PIDController* pidDriver;
	PIDController* pidId;
	PIDController* pidIq;
};