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

	void simulateStep(double dt)
	{

	}

protected:
	std::map<QString, std::function<double()>> telemetry;
};