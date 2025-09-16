#pragma once

#include <QStringList>
#include <functional>
#include <map>
#include "XdofVehicle.h"

class XdofVehicleTelemetry : public XdofVehicle
{
public:
	XdofVehicleTelemetry() : XdofVehicle(0)
	{
		telemetry["CarSpeedX"] = [this]() { return this->getVx(); };
		telemetry["CarSpeedY"] = [this]() { return this->getVy(); };
		telemetry["CarOmega"] = [this]() { return this->getOmega(); };
		telemetry["CarX"] = [this]() { return this->getX(); };
		telemetry["CarY"] = [this]() { return this->getY(); };
		telemetry["CarYaw"] = [this]() { return this->getYaw(); };
		telemetry["Gear"] = [this]() { return this->getGear(); };
		telemetry["EngineTorque"] = [this]() { return this->getT_m(); };
		telemetry["EngineRPM"] = [this]() { return this->getRpm(); };
		telemetry["Brake1_Force"] = [this]() { return this->getBrakeForces()[0]; };
		telemetry["Brake2_Force"] = [this]() { return this->getBrakeForces()[1]; };
		telemetry["Brake3_Force"] = [this]() { return this->getBrakeForces()[2]; };
		telemetry["Brake4_Force"] = [this]() { return this->getBrakeForces()[3]; };
		telemetry["Wheel1_Load"] = [this]() { return this->getWheelLoads()[0]; };
		telemetry["Wheel2_Load"] = [this]() { return this->getWheelLoads()[1]; };
		telemetry["Wheel3_Load"] = [this]() { return this->getWheelLoads()[2]; };
		telemetry["Wheel4_Load"] = [this]() { return this->getWheelLoads()[3]; };
		telemetry["Wheel1_SpeedX"] = [this]() { return this->getWheelSpeedsX()[0]; };
		telemetry["Wheel2_SpeedX"] = [this]() { return this->getWheelSpeedsX()[1]; };
		telemetry["Wheel3_SpeedX"] = [this]() { return this->getWheelSpeedsX()[2]; };
		telemetry["Wheel4_SpeedX"] = [this]() { return this->getWheelSpeedsX()[3]; };
		telemetry["Wheel1_ForceX"] = [this]() { return this->getWheelForcesX()[0]; };
		telemetry["Wheel2_ForceX"] = [this]() { return this->getWheelForcesX()[1]; };
		telemetry["Wheel3_ForceX"] = [this]() { return this->getWheelForcesX()[2]; };
		telemetry["Wheel4_ForceX"] = [this]() { return this->getWheelForcesX()[3]; };
		telemetry["Wheel1_ForceY"] = [this]() { return this->getWheelForcesY()[0]; };
		telemetry["Wheel2_ForceY"] = [this]() { return this->getWheelForcesY()[1]; };
		telemetry["Wheel3_ForceY"] = [this]() { return this->getWheelForcesY()[2]; };
		telemetry["Wheel4_ForceY"] = [this]() { return this->getWheelForcesY()[3]; };
		telemetry["Wheel1_SpeedY"] = [this]() { return this->getWheelSpeedsY()[0]; };
		telemetry["Wheel2_SpeedY"] = [this]() { return this->getWheelSpeedsY()[1]; };
		telemetry["Wheel3_SpeedY"] = [this]() { return this->getWheelSpeedsY()[2]; };
		telemetry["Wheel4_SpeedY"] = [this]() { return this->getWheelSpeedsY()[3]; };
		telemetry["Wheel1_Lambda"] = [this]() { return this->getWheelLambdas()[0]; };
		telemetry["Wheel2_Lambda"] = [this]() { return this->getWheelLambdas()[1]; };
		telemetry["Wheel3_Lambda"] = [this]() { return this->getWheelLambdas()[2]; };
		telemetry["Wheel4_Lambda"] = [this]() { return this->getWheelLambdas()[3]; };
		telemetry["Wheel1_Beta"] = [this]() { return this->getWheelBetas()[0]; };
		telemetry["Wheel2_Beta"] = [this]() { return this->getWheelBetas()[1]; };
		telemetry["Wheel3_Beta"] = [this]() { return this->getWheelBetas()[2]; };
		telemetry["Wheel4_Beta"] = [this]() { return this->getWheelBetas()[3]; };
		telemetry["Wheel1_Torque"] = [this]() { return this->getWheelTorques()[0]; };
		telemetry["Wheel2_Torque"] = [this]() { return this->getWheelTorques()[1]; };
		telemetry["Wheel3_Torque"] = [this]() { return this->getWheelTorques()[2]; };
		telemetry["Wheel4_Torque"] = [this]() { return this->getWheelTorques()[3]; };
		telemetry["Steering_w1"] = [this]() { return this->getSteer_w1(); };
		telemetry["Steering_w4"] = [this]() { return this->getSteer_w4(); };
		telemetry["Wheel1_LatForce"] = [this]() { return this->getWheelFLat()[0]; };
		telemetry["Wheel2_LatForce"] = [this]() { return this->getWheelFLat()[1]; };
		telemetry["Wheel3_LatForce"] = [this]() { return this->getWheelFLat()[2]; };
		telemetry["Wheel4_LatForce"] = [this]() { return this->getWheelFLat()[3]; };
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