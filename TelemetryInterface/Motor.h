#pragma once

#include <numbers>

class Motor
{
public:
    Motor() {};
    ~Motor() {};

    double getIq() const { return iq; };
    double getId() const { return id; };
    double getIqTarget(double tau_tgt) const { return (2.0 / 3.0) * (tau_tgt / (PolePairs * LambdaM)); };
    double getMutualInductaceIq() const { return Lq* getIq()* omega* PolePairs; };
    double getMutualInductaceId() const { return (Ld * getId() + LambdaM) * omega* PolePairs; };
    double getMechanicalPowerKW() const { return getTorque() * omega / 1000.0; };
    double getElectricalAngle() const { return theta_e; };
    double getTorque() const { return 1.5 * PolePairs * (LambdaM * iq + (Ld - Lq) * id * iq); };
    double getSpeedRPM() const { return omega * 60.0 / (2 * std::numbers::pi); };
    double getMaxTorque() const { return 500; };
    double getCoastRegenTorque() const { return 150; };

protected:
    // Parameters
    double Rs = 0.05;      // Resistenza [Ohm]
    double Ld = 2E-4;      // Induttanza d [H]
    double Lq = 2E-4;      // Induttanza q [H]
    double LambdaM = 0.1;  // Flusso magnetico [Wb]
    int PolePairs = 4;     // Coppie polari
    double J = 0.01;       // Inerzia [kg*m^2]
    double B = 0.00001;    // Attrito viscoso [Nm*s]
    
private:
    // Stati
    double id = 0, iq = 0;   // [A]
    double omega = 0;        // [rad/s]
    double theta = 0;        // [rad]
    double theta_e = 0;      // [rad]

public:
    void update(double carOmega, double vd, double vq, double dt)
    {
        double omega_e = carOmega * PolePairs;
        theta_e += omega_e * dt;

        // Derivate delle correnti (modello d-q)
        double did_dt = (vd - Rs * id + omega_e * Lq * iq) / Ld;
        double diq_dt = (vq - Rs * iq - omega_e * (Ld * id + LambdaM)) / Lq;

        id += did_dt * dt;
        iq += diq_dt * dt;

        // Dinamica meccanica
        omega = carOmega;
        theta += omega * dt;

        double torque = 1.5 * PolePairs * (LambdaM * iq + (Ld - Lq) * id * iq);
    }

    void updateTorque(double vd, double vq, double tau_L, double dt)
    {
        double omega_e = omega * PolePairs;
        theta_e += omega_e * dt;

        // Derivate delle correnti (modello d-q)
        double did_dt = (vd - Rs * id + omega_e * Lq * iq) / Ld;
        double diq_dt = (vq - Rs * iq - omega_e * (Ld * id + LambdaM)) / Lq;

        id += did_dt * dt;
        iq += diq_dt * dt;

        // Coppia elettromagnetica
        double torque = 1.5 * PolePairs * (LambdaM * iq + (Ld - Lq) * id * iq);

        // Dinamica meccanica
        double domega_dt = (torque - tau_L - B * omega) / J;
        omega += domega_dt * dt;
        theta += omega * dt;
    }
};