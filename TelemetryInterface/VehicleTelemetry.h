#pragma once

#include <map>
#include "Vehicle.h"

class VehicleTelemetry : public Vehicle
{
public:
	VehicleTelemetry() : Vehicle()
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

	void simulateStep(double throttlePercent, double brakePercent, double simTime, double dt)
	{
		Vehicle* veh = this;
		Powertrain* powertrain = veh->powertrain;
		//double tau_star = powertrain->pidDriver->update(speed_ref, veh->getSpeed(), dt) * veh->getTractiveRatio();

		Motor* motor = powertrain->motor;
		double tau_star = (throttlePercent / 100) * motor->getMaxTorque();
		if (throttlePercent < 1e-3 && veh->getSpeed() > 1.0) {
			tau_star = -motor->getCoastRegenTorque();  // small negative
		}
		if ((veh->getSpeed() != 0 && throttlePercent < 1e-3) || throttlePercent > 1e-3)
		{
			double vd_star = powertrain->pidId->update(id_target, motor->getId(), dt) - motor->getMutualInductaceIq();
			double vq_star = powertrain->pidIq->update(motor->getIqTarget(tau_star), motor->getIq(), dt) + motor->getMutualInductaceId();

			Battery* battery = powertrain->battery;
			battery->update((motor->getMechanicalPowerKW() * 1000) / battery->getVbat(), dt);

			Inverter* inverter = powertrain->inverter;
			inverter->update(battery->getVbat(), vd_star, vq_star, motor->getElectricalAngle(), inverter->carrier->getCarrierValue(simTime), (int)(1.0 / (inverter->carrier->getFrequency() * dt)));

			motor->update(veh->getOmega(), inverter->getVd(), inverter->getVq(), battery->getSoC(), dt);  // vd_star, vq_star, dt); // inverter->getVd(), inverter->getVq(), dt);
			veh->update(motor->getTorque(), 0, brakePercent / 100, dt);
		}
		else if (throttlePercent < 1e-3)
			veh->update(0, 0, brakePercent / 100, dt);
	}

protected:
	double speed_ref = 100.0;
	double id_target = 0.0;
	double tau_ref = 200;
	std::map<QString, std::function<double()>> telemetry;
};