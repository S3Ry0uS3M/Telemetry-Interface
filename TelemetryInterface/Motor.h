#pragma once

#include <numbers>

class Motor
{
public:
    Motor() {};
    ~Motor() {};

    double getIq() const { return iq; };
    double getId() const { return id; };
    double getIdTarget() const { return 0.0; };
    double getIqTarget(double tau_tgt) const { return (2.0 / 3.0) * (tau_tgt / (PolePairs * LambdaM)); };
    double getMutualInductaceIq() const { return Lq* getIq()* omega * PolePairs; };
    double getMutualInductaceId() const { return (Ld * getId() + LambdaM) * omega* PolePairs; };
    double getMechanicalPowerKW() const { return getTorque() * omega / 1000.0; };
    double getElectricalAngle() const { return theta_e; };
    double getTorque() const { return Torque; };
    double getSpeedRPM() const { return omega * 60.0 / (2 * std::numbers::pi); };
    double getMaxTorque() const { return 500; };
    double getMaxCurrent(double Tau_max, double Vmax) const {
        // For Ld != Lq:
        // T = 1.5 * Pp * (LambdaM * iq + (Ld - Lq) * id * iq)
        // B = Ld - Lq
        // T = 1.5 * Pp * (LambdaM * iq + B * id * iq)
        // Id = (T/(1.5 * Pp) - LambdaM * iq) / (B * iq)
        // Id = ((2 * T)/(3 * Pp))/(B * Iq) - LambdaM / B
        // Then:
        // Vd = Rs * Id - omega_e * Lq * Iq
        // Vq = Rs * Iq + omega_e * (Ld * Id + LambdaM)
        if (Ld != Lq)
        {
            auto F = [&](double Iq, double omega) {
                double B = Ld - Lq;
                double omega_e = omega * PolePairs;
                // Id from torque eq
                double Id = ((2.0 * Tau_max) / (3.0 * PolePairs)) / (B * Iq) - LambdaM / B;
                // Vd, Vq from (5)
                double Vd = Rs * Id - omega_e * Lq * Iq;
                double Vq = Rs * Iq + omega_e * (Ld * Id + LambdaM);
                return Vd * Vd + Vq * Vq - Vmax * Vmax;
            };

            double Imax = 0.0;
            for (double rpm = 0; rpm <= 12000; rpm += 200)
            {
                double omega = rpm * 2 * M_PI / 60;
                
                double Iq_min = 1.0e-6;
                double Iq_max = 2000.0;
                double fmin = F(Iq_min, omega);
                double fmax = F(Iq_max, omega);
                if (fmin * fmax > 0)
                    return Imax; // no solutions available at omega_e given -> limit of voltage reached to give max torque -> return last value found
                double Iq_star = 0.0;
                for (int iter = 0; iter < 100; ++iter)
                {
                    double m = 0.5 * (Iq_min + Iq_max);
                    double fm = F(m, omega);
                    if (std::abs(fm) < 1e-6)
                    {
                        Iq_star = m;
                        break;
                    }

                    if (fmin * fm <= 0) {
                        Iq_max = m; fmax = fm;
                    }
                    else {
                        Iq_min = m; fmin = fm;
                    }
                }
                if (Iq_star == 0.0)
                    Iq_star = 0.5 * (Iq_min + Iq_max);
                double Id_star = ((2.0 * Tau_max) / (3.0 * PolePairs)) / ((Ld - Lq) * Iq_star) - LambdaM / (Ld - Lq);
                Imax = std::sqrt(Id_star * Id_star + Iq_star * Iq_star);
            }
            return Imax;
        }
        // if Ld == Lq:
        return getMaxTorque() / (1.5 * PolePairs * LambdaM);
    };
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

protected:
    // Outputs
    double Torque = 0;

public:
    void update(double carOmega, double vd, double vq, double SoC, double dt)
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

        double clampTorque = 1.0;
        if (SoC < 20)
        {
            double scale = SoC / 20.0;  // 20% -> scale=1, 0% -> scale=0
            clampTorque = std::clamp(scale, 0.0, 1.0);
        }
        Torque = 1.5 * PolePairs * (LambdaM * iq + (Ld - Lq) * id * iq) * clampTorque;
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