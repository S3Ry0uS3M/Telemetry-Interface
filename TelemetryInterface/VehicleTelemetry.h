#pragma once

#include <map>
#include "Vehicle.h"

class VehicleTelemetry : public Vehicle
{
public:
	VehicleTelemetry(double fsw) : Vehicle(fsw)
	{
		telemetry["CarSpeed"] = [this]() { return this->getSpeed(); };
		telemetry["CarOmega"] = [this]() { return this->getOmega(); };
		telemetry["CarLoadTorque"] = [this]() { return this->getLoadTorque(); };
		telemetry["CarFrolling"] = [this]() { return this->getRollingResistance(); };
		telemetry["CarFaero"] = [this]() { return this->getAeroResistance(); };
		telemetry["CarFtractive"] = [this]() { return this->getTractiveForce(); };
		telemetry["CarFbrake"] = [this]() { return this->getBrakingForce(); };
		telemetry["BatteryVoltage"] = [this]() { return this->powertrain->battery->getVbat(); };
		telemetry["BatterySoC"] = [this]() { return this->powertrain->battery->getSoC(); };
		telemetry["MotorIq"] = [this]() { return this->powertrain->motor->getIq(); };
		telemetry["MotorId"] = [this]() { return this->powertrain->motor->getId(); };
		telemetry["MotorMut.InductanceIq"] = [this]() { return this->powertrain->motor->getMutualInductaceIq(); };
		telemetry["MotorMut.InductanceId"] = [this]() { return this->powertrain->motor->getMutualInductaceId(); };
		telemetry["MotorMechPower"] = [this]() { return this->powertrain->motor->getMechanicalPowerKW(); };
		telemetry["MotorElec.Angle"] = [this]() { return this->powertrain->motor->getElectricalAngle(); };
		telemetry["MotorTorque"] = [this]() { return this->powertrain->motor->getTorque(); };
		telemetry["MotorSpeedRPM"] = [this]() { return this->powertrain->motor->getSpeedRPM(); };
		telemetry["InverterVd"] = [this]() { return this->powertrain->inverter->getVd(); };
		telemetry["InverterVq"] = [this]() { return this->powertrain->inverter->getVq(); };
	};
	~VehicleTelemetry() {};

public:
	QStringList getChannels()
	{
		QStringList keys;
		for (const auto& pair : telemetry) {
			keys.push_back(pair.first);
		}
		return keys;
	}

	std::map<QString, std::function<double()>>* getMap() { return &telemetry; };

protected:
	std::map<QString, std::function<double()>> telemetry;
};