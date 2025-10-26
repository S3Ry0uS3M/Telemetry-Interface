#pragma once

#include <QVector>
#include <numbers>

class Engine
{
public:
	Engine()
	{
		for (int i = 0; i < gearRatios.size(); i++)
			rt.push_back(gearRatios[i] * r_final);
	}
	~Engine() {};

private:
	// [Nm] Engine torque curve
	const QVector<double> T_m = { 558.687348016378, 572.805288620221, 583.255710898092, 593.261434355629, 599.923795131873, 609.180956729229, 615.258432626951,
		622.474732624635, 632.114828041388, 638.740769635113, 646.421953778950, 652.546822483130, 659.707141963830, 667.848414811759, 674.498006166769,
		678.541340835388, 683.620480696612, 687.381698252657, 691.021586210120, 694.545922168933, 696.862701052572, 698.027656264507, 698.093143426179,
		696.060109153369, 695.119737372311, 692.170826196689, 688.302812756178, 683.554532357745, 677.962662791304, 673.486118156730, 667.232318488132,
		661.145286810696, 654.294295691963, 644.884824567477, 634.816173460368, 625.002424912934, 614.556082818016, 577.487620679787, 531.044977487345, 486.567638719944 };

	// [rpm] Engine speed curve
	const QVector<double> n_m = { 4400, 4500, 4600, 4700, 4800, 4900, 5000, 5100, 5200, 5300, 5400, 5500, 5600, 5700, 5800, 5900, 6000, 6100, 6200, 6300, 6400, 6500, 6600, 6700,
		6800, 6900, 7000, 7100, 7200, 7300, 7400, 7500, 7600, 7700, 7800, 7900, 8000, 8100, 8200, 8300 };

	const double rpmRiseRate = 3000.0;  // [rpm/s] how fast engine can rev up
	const double rpmFallRate = 2000.0;  // [rpm/s] how fast it drops

	QVector<double> gearRatios = { 35.0 / 13.0, 33.0 / 16.0, 27.0 / 16.0, 27.0 / 19.0, 23.0 / 19.0, 20.0 / 19.0, 23.0 / 24.0 }; // rt: gear ratios(out / in = n_in / n_out)
	double r_final = 46.0 / 14.0; // final gear to differential
	QVector<double> rt = {}; // total gear ratios (gear * final)

	// Differential
	const double K_diff = 500.0; // [Nm/(rad/s)] differential stiffness
	const double d_acc = 0.6; // acceleration differential action
	const double d_brk = 0.3; // braking differential action
	const double d_preload = 80.0; // [Nm] differential preload torque

protected:
	//Measurements
	double totTorque = 0.0; // [Nm] total engine torque
	double torque2 = 0.0; // [Nm] engine torque seen in rl tyre
	double torque3 = 0.0; // [Nm] engine torque seen in rr tyre
	double RPM = 0.0; // [rpm]

public:
	double getEngineTorque() const { return totTorque; };
	double getEngineTorque2() const { return torque2; };
	double getEngineTorque3() const { return torque3; };
	double getEngineRPM() const { return RPM; };

public:
	void update(double vx_car, double omega_w2, double omega_w3, double throttle, int gear, double dt)
	{
		// Engine torque
		double n_ice2 = RPM;
		double n_ice3 = RPM;
		torque2 = 0.0;
		torque3 = 0.0;
		if (gear > 0)
		{
			// Engine speed
			double n_2 = omega_w2 * 30 / std::numbers::pi; // [rpm] speed of wheel 2
			double n_3 = omega_w3 * 30 / std::numbers::pi; // [rpm] speed of wheel 3

			double n_ice2_target = n_2 * rt[gear - 1]; // [rpm] speed of wheel 2 seen in ICE
			double n_ice3_target = n_3 * rt[gear - 1]; // [rpm] speed of wheel 3 seen in ICE
			double rpm_tgt = 0.5 * (n_ice2_target + n_ice3_target);

			// Approach target gradually
			double diff = rpm_tgt - RPM;
			double maxStep = (diff > 0 ? rpmRiseRate : -rpmFallRate * (gear == 1 ? 1.0 : 10.0)) * dt;
			if (std::abs(diff) > std::abs(maxStep))
				RPM += maxStep;
			RPM = std::min(n_m.last(), std::max(n_m.first(), rpm_tgt));

			double T_engine = 0.0;
			if (vx_car >= 0.1 && throttle < 0.05)
				T_engine = -0.1 * 0.5 * linearInterp(n_m, T_m, RPM); // Engine braking
			else if ((n_ice2 < n_m.last() || vx_car < 0.1) && throttle >= 0.05)
				T_engine = 0.5 * linearInterp(n_m, T_m, RPM);	// Engine traction

			double T_drive = T_engine * rt[gear - 1];
			double dOmega = omega_w2 - omega_w3;
			if (std::abs(dOmega) < 1E-3) // avoid numerical issues at very low differences
				dOmega = 0.0;
			if (dOmega != 0.0)
				dOmega = dOmega;

			double bias = (T_drive >= 0.0) ? d_acc : d_brk;
			double T_lock_max = bias * std::abs(T_drive);
			T_lock_max = std::max(T_lock_max, d_preload); // ensure minimum preload
			double T_lock_dyn = std::min(K_diff * std::abs(dOmega), T_lock_max);
			double T_clutch = std::copysign(T_lock_dyn, dOmega);

			double T_L = 0.5 * (T_drive - T_clutch);
			double T_R = 0.5 * (T_drive + T_clutch);

			torque2 = T_L;
			torque3 = T_R;
			totTorque = T_L + T_R; // = T_drive
		}
		else
		{
			// Target RPM based on throttle
			double targetRPM = n_m.first() + throttle * (n_m.last() - n_m.first());

			// Approach target gradually
			double diff = targetRPM - RPM;
			double maxStep = (diff > 0 ? rpmRiseRate : -rpmFallRate) * dt;
			if (std::abs(diff) > std::abs(maxStep))
				RPM += maxStep;
			else
				RPM = targetRPM;
			totTorque = torque2 + torque3;
		}
	}

private:
	double linearInterp(QVector<double> x_vec, QVector<double> y_vec, double x)
	{
		// Find the interval [x0, x1] where x is located
		if (x <= x_vec.first()) return y_vec.first();
		if (x >= x_vec[x_vec.size() - 1]) return y_vec.last();
		int i = 0;
		while (i < x_vec.size() - 1 && x_vec[i + 1] < x) {
			i++;
		}
		double x0 = x_vec[i];
		double y0 = y_vec[i];
		double x1 = x_vec[i + 1];
		double y1 = y_vec[i + 1];

		// Linear interpolation formula
		return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
	}
};