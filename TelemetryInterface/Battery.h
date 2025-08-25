#pragma once

#include <cmath>

class Battery
{
public:
	Battery() {};
	~Battery() {};

public:
    double getVdcMax() const { return this->VdcMax; };
    double getVbat() const { return Vbat; };
    double getSoC() const { return SoC * 100; };

protected:
    // Parameters
    double VdcMax = 1000;   // Max battery voltage

    double Qbat = 1.11;  // Charge of a cell [Ah]
    int Ns = 260;        // Number of series cells [-]
    int Np = 12;         // Number of parallel cells [-]

    double Voc_a = -0.7;
    double Voc_b = 1.6;
    double Voc_c = 3.0;

    double R0_a = 0.001;
    double R0_b = -0.001;
    double R0_c = 0.0005;

    double R1_a = 0.001;
    double R1_b = -0.001;
    double R1_c = 0.0005;

    double C1_a = 200.0;
    double C1_b = -50.0;
    double C1_c = 100.0;

private:
    // States
    double SoC = 0.8;
    double Vrc = 0;

    // Outputs
    double Voc = 0;
    double R0 = 0;
    double Vbat = 1000;

public:
    void update(double i_bat, double dt)
    {
        // Open circuit voltage
        Voc = Voc_a * std::pow(SoC, 2) + Voc_b * SoC + Voc_c;

        // Series resistance
        R0 = R0_a * std::pow(SoC, 2) + R0_b * SoC + R0_c;

        // Transient resistance
        double R1 = R1_a * std::pow(SoC, 2) + R1_b * SoC + R1_c;

        // Transient capacitance
        double C1 = C1_a * std::pow(SoC, 2) + C1_b * SoC + C1_c;

        // Total charge of the battery
        double Qtot = Qbat * Np;

        // Battery voltage
        Vbat = Ns * (Voc - R0 * i_bat - Vrc);
        Vbat = std::fmax(Vbat, 0);

        // Dynamics of the system
        double dSoC_dt = -i_bat / (Qtot * 3600);
        double dVrc_dt = -(Vrc / (R1 * C1)) + (i_bat / C1);

        SoC += dSoC_dt * dt;
        Vrc += dVrc_dt * dt;

        SoC = std::fmin(100, std::fmax(SoC, 0));
    }
};