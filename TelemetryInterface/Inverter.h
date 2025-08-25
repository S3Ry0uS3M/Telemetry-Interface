#pragma once

#include <QMutex>
#include "Carrier.h"

class Inverter
{
public:
	Inverter(double fsw) {
        carrier = new Carrier(fsw);
    };
	~Inverter() {};

public:
    mutable QMutex bufferMutex;

public:
    double getVd() const { return Vd; };
    double getVq() const { return Vq; };
    double getVAN() const { return Van; };
    double getVBN() const { return Vbn; };
    double getVCN() const { return Vcn; };
    double getVa() const { return vanSum / vanBuffer.size(); };
    double getVb() const { return vbnSum / vbnBuffer.size(); };
    double getVc() const { return vcnSum / vcnBuffer.size(); };

public:
    Carrier* carrier;

protected:
	// Parametri
	QVector<double> vanBuffer = {};
    QVector<double> vbnBuffer = {};
    QVector<double> vcnBuffer = {};
	double vanSum = 0;
	double vbnSum = 0;
	double vcnSum = 0;

private:
	// Stati
	double Van = 0;
	double Vbn = 0;
	double Vcn = 0;
	double Vd = 0;
	double Vq = 0;

public:
	void update(double Vdc, double vd_star, double vq_star, double theta_e, double carrier, int windowSize)
    {
        QMutexLocker locker(&bufferMutex);  // locks mutex for the scope

        // RMS voltage of battery
        double Vdc_rms = Vdc / std::sqrt(3);

        // Voltage vector magnitude and angle
        double Vm = std::sqrt(std::pow(vd_star, 2) + std::pow(vq_star, 2));
        double Am = std::atan2(vq_star, vd_star);

        // Voltage limit
        double Vlim = std::max(std::min(Vdc_rms, Vm), 0.0);

        // d-axis and q-axis voltage limited
        double vd_lim = Vlim * std::cos(Am);
        double vq_lim = Vlim * std::sin(Am);

        // Inverse T matrix
        double vd_s = vd_lim * std::cos(theta_e) - vq_lim * std::sin(theta_e);
        double vq_s = vd_lim * std::sin(theta_e) + vq_lim * std::cos(theta_e);

        // Inverse D matrix
        double Va = vd_s;
        double Vb = -0.5 * vd_s + std::sqrt(3) / 2 * vq_s;
        double Vc = -vd_s + 0.5 * vd_s - std::sqrt(3) / 2 * vq_s;

        // Scaled over rms voltage
        double Va_rms = Va / Vdc_rms;
        double Vb_rms = Vb / Vdc_rms;
        double Vc_rms = Vc / Vdc_rms;

        // Symmetrical sinusoid
        double Va_s = Va_rms * 2 / std::sqrt(3);
        double Vb_s = Vb_rms * 2 / std::sqrt(3);
        double Vc_s = Vc_rms * 2 / std::sqrt(3);

        double Va_m = Va_s - 0.5 * (std::max(std::max(Va_s, Vb_s), Vc_s) + std::min(std::min(Va_s, Vb_s), Vc_s));
        double Vb_m = Vb_s - 0.5 * (std::max(std::max(Va_s, Vb_s), Vc_s) + std::min(std::min(Va_s, Vb_s), Vc_s));
        double Vc_m = Vc_s - 0.5 * (std::max(std::max(Va_s, Vb_s), Vc_s) + std::min(std::min(Va_s, Vb_s), Vc_s));

        double Sa = Va_m >= carrier ? Vdc : 0;
        double Sb = Vb_m >= carrier ? Vdc : 0;
        double Sc = Vc_m >= carrier ? Vdc : 0;

        Van = (Sa * 2 - Sb - Sc) / 3;
        Vbn = (Sb * 2 - Sa - Sc) / 3;
        Vcn = (Sc * 2 - Sa - Sb) / 3;

        vanBuffer.push_back(Van);
        vbnBuffer.push_back(Vbn);
        vcnBuffer.push_back(Vcn);
        vanSum += Van;
        vbnSum += Vbn;
        vcnSum += Vcn;

        auto getSum = [windowSize](QVector<double>* buffer, double sum) -> double {
            if (buffer->size() > windowSize) {
                sum -= buffer->front();
                buffer->removeFirst();
            }
            return sum;
        };
        vanSum = getSum(&vanBuffer, vanSum);
        vbnSum = getSum(&vbnBuffer, vbnSum);
        vcnSum = getSum(&vcnBuffer, vcnSum);

        double Va_sin = vanSum / vanBuffer.size();
        double Vb_sin = vbnSum / vbnBuffer.size();

        // D Matrix
        double ds = Va_sin;
        double qs = 1 / std::sqrt(3) * Va_sin + 2 / std::sqrt(3) * Vb_sin;

        Vd = ds * std::cos(theta_e) + qs * std::sin(theta_e);
        Vq = -ds * std::sin(theta_e) + qs * std::cos(theta_e);
    }
};