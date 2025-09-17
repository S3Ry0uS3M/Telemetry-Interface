#pragma once

#include <QStringList>
#include <functional>
#include <map>
#include "XdofVehicle.h"

class XdofVehicleTelemetry : public XdofVehicle
{
public:
	XdofVehicleTelemetry() : XdofVehicle()
	{
		telemetry["CarSpeedX"] = [this]() { return this->getVx(); };
		telemetry["CarSpeedY"] = [this]() { return this->getVy(); };
		telemetry["CarOmega"] = [this]() { return this->getOmega(); };
		telemetry["CarX"] = [this]() { return this->getX(); };
		telemetry["CarY"] = [this]() { return this->getY(); };
		telemetry["CarYaw"] = [this]() { return this->getYaw(); };
		telemetry["Gear"] = [this]() { return this->getGear(); };
		telemetry["EngineTorque"] = [this]() { return this->engine->getEngineTorque(); };
		telemetry["EngineRPM"] = [this]() { return this->engine->getEngineRPM(); };
		telemetry["Brake1_Force"] = [this]() { return this->flTyre->getBrakeForce(); };
		telemetry["Brake2_Force"] = [this]() { return this->rlTyre->getBrakeForce(); };
		telemetry["Brake3_Force"] = [this]() { return this->rrTyre->getBrakeForce(); };
		telemetry["Brake4_Force"] = [this]() { return this->frTyre->getBrakeForce(); };
		telemetry["Wheel1_Load"] = [this]() { return this->getWheelLoads()[0]; };
		telemetry["Wheel2_Load"] = [this]() { return this->getWheelLoads()[1]; };
		telemetry["Wheel3_Load"] = [this]() { return this->getWheelLoads()[2]; };
		telemetry["Wheel4_Load"] = [this]() { return this->getWheelLoads()[3]; };
		telemetry["Wheel1_SpeedX"] = [this]() { return this->flTyre->getTyreVx(); };
		telemetry["Wheel2_SpeedX"] = [this]() { return this->rlTyre->getTyreVx(); };
		telemetry["Wheel3_SpeedX"] = [this]() { return this->rrTyre->getTyreVx(); };
		telemetry["Wheel4_SpeedX"] = [this]() { return this->frTyre->getTyreVx(); };
		telemetry["Wheel1_ForceX"] = [this]() { return this->flTyre->getFx(); };
		telemetry["Wheel2_ForceX"] = [this]() { return this->rlTyre->getFx(); };
		telemetry["Wheel3_ForceX"] = [this]() { return this->rrTyre->getFx(); };
		telemetry["Wheel4_ForceX"] = [this]() { return this->frTyre->getFx(); };
		telemetry["Wheel1_ForceY"] = [this]() { return this->flTyre->getFy(); };
		telemetry["Wheel2_ForceY"] = [this]() { return this->rlTyre->getFy(); };
		telemetry["Wheel3_ForceY"] = [this]() { return this->rrTyre->getFy(); };
		telemetry["Wheel4_ForceY"] = [this]() { return this->frTyre->getFy(); };
		telemetry["Wheel1_SpeedY"] = [this]() { return this->flTyre->getTyreVy(); };
		telemetry["Wheel2_SpeedY"] = [this]() { return this->rlTyre->getTyreVy(); };
		telemetry["Wheel3_SpeedY"] = [this]() { return this->rrTyre->getTyreVy(); };
		telemetry["Wheel4_SpeedY"] = [this]() { return this->frTyre->getTyreVy(); };
		telemetry["Wheel1_Beta"] = [this]() { return this->flTyre->getLatSlip(); };
		telemetry["Wheel2_Beta"] = [this]() { return this->rlTyre->getLatSlip(); };
		telemetry["Wheel3_Beta"] = [this]() { return this->rrTyre->getLatSlip(); };
		telemetry["Wheel4_Beta"] = [this]() { return this->frTyre->getLatSlip(); };
		telemetry["Wheel1_Lambda"] = [this]() { return this->flTyre->getLongSlip(); };
		telemetry["Wheel2_Lambda"] = [this]() { return this->rlTyre->getLongSlip(); };
		telemetry["Wheel3_Lambda"] = [this]() { return this->rrTyre->getLongSlip(); };
		telemetry["Wheel4_Lambda"] = [this]() { return this->frTyre->getLongSlip(); };
		telemetry["Steering_w1"] = [this]() { return this->getSteer_w1(); };
		telemetry["Steering_w4"] = [this]() { return this->getSteer_w4(); };
	};
	~XdofVehicleTelemetry() {};

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

	void simulateStep(double throttlePercent, double brakePercent, int gear, double steer, double dt)
	{
		update(throttlePercent/100, brakePercent/100, steer, gear, dt);
	}

protected:
	std::map<QString, std::function<double()>> telemetry;
};