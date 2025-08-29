#pragma once

#include <cmath>
#include <numbers>
#include "Powertrain.h"

const double RHO = 1.2041;

class Vehicle
{
public:
    Vehicle() {
        powertrain = new Powertrain();
    };
    ~Vehicle() {};

public:
    double getSpeed() const { return Vx * 3.6; };
    double getTractiveRatio() const { return RearWheelRadius / GearRatio; };
    double getOmega() const { return Vx * GearRatio / RearWheelRadius; };
    double getLoadTorque() const { return Tau_load; };
    double getRollingResistance() const { return F_rolling; };
    double getAeroResistance() const { return F_aero; };
    double getTractiveForce() const { return F_tractive; };
    double getBrakingForce() const { return F_brake; };

public:
    Powertrain* powertrain;

protected:
    // Parameters
    double Mcar = 800;              // [kg]
    double Af = 2;                  // [m^2]
    double Cx = 0.7;                // [-]
    double Crr = 0.01;              // [-]
    double RearWheelRadius = 0.38;  // [m]
    double GearRatio = 3;           // [-]
    double BrakeMax = 20000;        // [N] max braking force

private:
    // States
    double Vx = 0;
    double Tau_load = 0;

    // Outputs
    double F_tractive = 0;
    double F_rolling = 0;
    double F_aero = 0;
    double F_brake = 0;

public:
    void update(double tau_motor, double aSlope, double brake, double dt)
    {
        double beta = std::atan(aSlope * std::numbers::pi / 180);

        F_tractive = tau_motor * GearRatio / RearWheelRadius;
        F_rolling = Crr * Mcar * 9.80665 * std::cos(beta);
        F_aero = 0.5 * Af * Cx * RHO * std::pow(Vx, 2);
        F_brake = brake * BrakeMax; // linear brake model

        Tau_load = (F_aero + F_rolling + F_brake) * RearWheelRadius / GearRatio;

        double Fx = 0;
        double F_load = F_rolling + F_aero + F_brake;

        if (Vx > 1e-6) {
            Fx = F_tractive - F_load;
        }
        else if (Vx < -1e-6) {
            Fx = F_tractive + F_load; // resistance opposes backward motion
        }
        else {
            // car at rest
            if (F_tractive > F_load)
                Fx = F_tractive - F_load;
            else if (F_tractive < -F_load)
                Fx = F_tractive + F_load;
            else
                Fx = 0; // stay at rest
        }

        double dvx_dt = Fx / Mcar;
        Vx += dvx_dt * dt;

        // prevent tiny numerical reverse velocities
        if (std::abs(Vx) < 1e-6) Vx = 0;
    }
};