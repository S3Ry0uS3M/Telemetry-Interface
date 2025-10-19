#include <iostream>
#include <fstream>
#include <qmath.h>

#include "Powertrain.h"

const double dt = 1E-05;
const double simTime = 5.0; // seconds

using namespace std;

int main()
{
    std::ofstream file("c:\\users\\super\\desktop\\motor_characterization.txt");
    if (!file.is_open()) {
        std::cerr << "Error: cannot open output file." << std::endl;
        return 1;
    }

    file << "RPM,TauTarget[Nm],Torque[Nm],Iq[A],Id[A],Vd[V],Vq[V],Power[W]\n";

    Powertrain* pt = new Powertrain();
    double tau_step = 25.0; // Nm
    double tau_max = pt->motor->getMaxTorque();
    double Vmax = pt->battery->getVdcMax() / std::sqrt(3.0);
    
    for (double rpm = 0; rpm <= 12000; rpm += 200)
    {
        double omega = rpm * 2 * M_PI / 60;

        pt = new Powertrain();
        double Vmag = 0.0;
        double time = 0.0;
        while (time < simTime)
        {
            double vd_star = pt->pidId->update(0.0, pt->motor->getId(), dt) - pt->motor->getMutualInductaceIq();
            double vq_star = pt->pidIq->update(pt->motor->getIqTarget(tau_max), pt->motor->getIq(), dt) + pt->motor->getMutualInductaceId();

            Inverter* inverter = pt->inverter;
            inverter->update(pt->battery->getVbat(), vd_star, vq_star, pt->motor->getElectricalAngle(), inverter->carrier->getCarrierValue(time), (int)(1.0 / (inverter->carrier->getFrequency() * dt)));

            pt->motor->update(omega, inverter->getVd(), inverter->getVq(), pt->battery->getSoC(), dt);

            double Vd = pt->inverter->getVd();
            double Vq = pt->inverter->getVq();
            Vmag = std::sqrt(Vd * Vd + Vq * Vq);

            time += dt;
        }

        std::cout << rpm << ","
            << tau_max << ": "
            << pt->motor->getTorque() << ","
            << pt->motor->getIq() << ","
            << pt->motor->getId() << ","
            << (Vmag > Vmax) << ","
            << pt->inverter->getVd() << ","
            << pt->inverter->getVq() << ","
            << pt->motor->getMechanicalPowerKW() * 1000
            << std::endl;

        file << rpm << ","
            << tau_max << ","
            << pt->motor->getTorque() << ","
            << pt->motor->getIq() << ","
            << pt->motor->getId() << ","
            << pt->inverter->getVd() << ","
            << pt->inverter->getVq() << ","
            << pt->motor->getMechanicalPowerKW() * 1000
            << "\n";
    }

    file.close();

    /*for (double rpm = 0; rpm <= 12000; rpm += 200)
    {
		double omega = rpm * 2 * M_PI / 60;

        pt = new Powertrain();
        for (double tau_tgt = 0.0; tau_tgt <= tau_max; tau_tgt += tau_step)
        {
            double time = 0.0;
            bool voltage_saturated = false;
            while (time < simTime)
            {
                double vd_star = pt->pidId->update(0.0, pt->motor->getId(), dt) - pt->motor->getMutualInductaceIq();
                double vq_star = pt->pidIq->update(pt->motor->getIqTarget(tau_tgt), pt->motor->getIq(), dt) + pt->motor->getMutualInductaceId();

                Inverter* inverter = pt->inverter;
                inverter->update(pt->battery->getVbat(), vd_star, vq_star, pt->motor->getElectricalAngle(), inverter->carrier->getCarrierValue(time), (int)(1.0 / (inverter->carrier->getFrequency() * dt)));

                pt->motor->update(omega, inverter->getVd(), inverter->getVq(), pt->battery->getSoC(), dt);
            
                double Vd = vd_star;
                double Vq = vq_star;
                double Vmag = std::sqrt(Vd * Vd + Vq * Vq);
                double Vmax = pt->battery->getVbat() / std::sqrt(3.0);

                if (Vmag > Vmax)
                {
					voltage_saturated = true;
                    break;
                }

                time += dt;
            }

            if (!voltage_saturated)
            {
                file << rpm << ","
                     << tau_tgt << ","
                     << pt->motor->getTorque() << ","
                     << pt->motor->getIq() << ","
                     << pt->motor->getId() << ","
                     << pt->inverter->getVd() << ","
                     << pt->inverter->getVq() << ","
                     << pt->motor->getMechanicalPowerKW() * 1000
                     << "\n";
            }
            else
                break;
        }
    }*/

    /*double time = 0.0;
    double rpm = 1000.0;
    double tau_tgt = 300.0;
    while (time < simTime)
    {
        double omega = rpm * 2 * M_PI / 60;

        double vd_star = pt->pidId->update(0.0, pt->motor->getId(), dt) - pt->motor->getMutualInductaceIq();
        double vq_star = pt->pidIq->update(pt->motor->getIqTarget(tau_tgt), pt->motor->getIq(), dt) + pt->motor->getMutualInductaceId();

        Inverter* inverter = pt->inverter;
        inverter->update(pt->battery->getVbat(), vd_star, vq_star, pt->motor->getElectricalAngle(), inverter->carrier->getCarrierValue(time), (int)(1.0 / (inverter->carrier->getFrequency() * dt)));

        pt->motor->update(omega, inverter->getVd(), inverter->getVq(), pt->battery->getSoC(), dt);
        
		double Vd = vd_star;
		double Vq = vq_star;
        double Vmag = std::sqrt(Vd * Vd + Vq * Vq);
        double Vmax = pt->battery->getVbat() / std::sqrt(3.0);

        std::cout << rpm << ","
            << tau_tgt << ": "
            << pt->motor->getTorque() << ","
            << pt->motor->getIq() << ","
            << pt->motor->getId() << ","
			<< (Vmag > Vmax) << ","
            //<< pt->inverter->getVd() << ","
            //<< pt->inverter->getVq() << ","
            //<< pt->motor->getMechanicalPowerKW() * 1000
            << std::endl;
        
        time += dt;
    }*/
}