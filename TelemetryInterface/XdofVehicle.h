#pragma once

#include <Dense>

using namespace Eigen;

class XdofVehicle
{
public:
	XdofVehicle(int index) {
		Iz = 3.176 * Mcar - 1754.164;
		if (index > Theta.size())
			index = Theta.size() - 1;
		Cf_l = { Theta[0][index], Theta[1][index], Theta[2][index] };
		Cf_t = { Theta[0][index], Theta[1][index], Theta[2][index] };
		cd = (0.8 * CxSmax) / af;
		cl = CzSmax / af;

		I_wf = 0.5 * mw * std::pow(rw_f, 2.0);
		I_wr = 0.5 * mw * std::pow(rw_r, 2.0);

		for (int i = 0; i < gearRatios.size(); i++)
			rt.push_back(gearRatios[i] * r_final);
	};
	~XdofVehicle() {};

public:
	mutable QMutex bufferMutex;

protected:
	// Parameters
	double Mcar = 1030;	// [kg]
	double Iz = 0.001;
	double Crr = 0.006; // [-] Rolling resistance coefficient
	QVector<double> Cf_l = {};
	QVector<double> Cf_t = {};
	/* Road types:
	1) Dry asphalt
	2) wet asphalt
	3) Snow
	4) Ice
	5) Dry Cobblestone
	6) wet cobblestone*/
	const QVector<QVector<double>> Theta = { 
		{ 1.2891, 0.86, 0.19, 0.05, 1.37, 0.4 },
		{ 23.99, 33.82, 94.13, 306.39, 6.46, 33.71 },
		{ 0.52, 0.35, 0.05, 0, 0.67, 0.12 }
	};
	double Wb = 3.148;	// [m] Wheel base
	double Wdis = 45;	// [%] Weight distribution
	double a = (1 - Wdis * 0.01) * Wb; // [m] Front tyres to CoG distance
	double b = Wdis * 0.01 * Wb;  // [m] Rear tyres to CoG distance
	double cF = 1.650; // [m] Front track from CoG
	double cR = 1.614; // [m] Rear track from CoG
	double h = 0.00012 * Mcar + 0.14338; // [m] CoG height

	double rw_f = 0.349; // [m] Front tyre radius
	double rw_r = 0.347; // [m] Rear tyre radius

	double CzSmax = 5.2; // [max lift x frontal area]
	double CxSmax = 1;   // [max drag x frontal area]
	double L_Dmax = 4;   // [max lift / max drag]

	double af = 1.8; // [m^2] frontal section
	double cd = 0; // [-] drag coefficient
	double cl = 0; // [-] lift coefficient
	double rho = 1.2041; // [kg / m^3] Standard air density
	double Vwind = 0; // [m/s] wind speed along the vehicle longitudinal axis

	double bb = 0.55; // Brake bias (front)
	double mu_tyre = 1.7; // Dry compound [-] Coefficient of friction

	double mw = 10; // [kg] Wheel mass
	double I_wf = 0.0;	// [kg m^2] Front wheel inertia
	double I_wr = 0.0;	// [kg m^2] Rear wheel inertia

	QVector<double> gearRatios = { 35.0/13.0, 33.0/16.0, 27.0/16.0, 27.0/19.0, 23.0/19.0, 20.0/19.0, 23.0/24.0 }; // rt: gear ratios(out / in = n_in / n_out)
	double r_final = 46.0/14.0; // final gear to differential
	QVector<double> rt = {}; // total gear ratios (gear * final)

	// [Nm] Engine torque curve
	QVector<double> T_m = { 558.687348016378, 572.805288620221, 583.255710898092, 593.261434355629, 599.923795131873, 609.180956729229, 615.258432626951, 
		622.474732624635, 632.114828041388, 638.740769635113, 646.421953778950, 652.546822483130, 659.707141963830, 667.848414811759, 674.498006166769, 
		678.541340835388, 683.620480696612, 687.381698252657, 691.021586210120, 694.545922168933, 696.862701052572, 698.027656264507, 698.093143426179, 
		696.060109153369, 695.119737372311, 692.170826196689, 688.302812756178, 683.554532357745, 677.962662791304, 673.486118156730, 667.232318488132, 
		661.145286810696, 654.294295691963, 644.884824567477, 634.816173460368, 625.002424912934, 614.556082818016, 577.487620679787, 531.044977487345, 486.567638719944 };

	// [rpm] Engine speed curve
	QVector<double> n_m = { 4400, 4500, 4600, 4700, 4800, 4900, 5000, 5100, 5200, 5300, 5400, 5500, 5600, 5700, 5800, 5900, 6000, 6100, 6200, 6300, 6400, 6500, 6600, 6700, 
		6800, 6900, 7000, 7100, 7200, 7300, 7400, 7500, 7600, 7700, 7800, 7900, 8000, 8100, 8200, 8300 };

	const double k_lat_damp = 1000.0; // [N/(m/s)]
	const double k_yaw_damp = 1000.0; // [Nm/rad]

private:
	// States
	double Vx = 10.0;		// [m/s] Longitudinal Speed
	double Vy = 0;		// [m/s] Lateral Speed
	double Omega = 0;	// [rad/s] Yaw rotational Speed
	double Yaw = 0;	// [rad] Yaw angle
	double X = 0;	// [m] Global X position
	double Y = 0;	// [m] Global Y position

	double Vx_world = 0;// [m/s] Global X speed
	double Vy_world = 0;// [m/s] Global Y speed

	double Omega1 = 0;	// [rad/s] FLTyre rotational speed
	double Omega2 = 0;	// [rad/s] RLTyre rotational speed
	double Omega3 = 0;	// [rad/s] RRTyre rotational speed
	double Omega4 = 0;	// [rad/s] FRTyre rotational speed

protected:
	// Outputs
	double steer_w1 = 0;	// [rad]
	double steer_w4 = 0;	// [rad]
	double engineTorque = 0; // [Nm]
	double RPM = 0; // [rpm]
	int gearNum = 0; // [-]
	QVector<double> tyreLoads = { 0.0, 0.0, 0.0, 0.0 };
	QVector<double> tyreLambdas = { 0.0, 0.0, 0.0, 0.0 };
	QVector<double> tyreBetas = { 0.0, 0.0, 0.0, 0.0 };
	QVector<double> tyreSpeedsX = { 0.0, 0.0, 0.0, 0.0 };
	QVector<double> tyreSpeedsY = { 0.0, 0.0, 0.0, 0.0 };
	QVector<double> tyreFLong = { 0.0, 0.0, 0.0, 0.0 };
	QVector<double> tyreFLat = { 0.0, 0.0, 0.0, 0.0 };
	QVector<double> tyreForcesX = { 0.0, 0.0, 0.0, 0.0 };
	QVector<double> tyreForcesY = { 0.0, 0.0, 0.0, 0.0 };
	QVector<double> tyreTorques = { 0.0, 0.0, 0.0, 0.0 };
	QVector<double> brakeForces = { 0.0, 0.0, 0.0, 0.0 };

public:
	double getVx() const { return Vx * 3.6; };
	double getVy() const { return Vy * 3.6; };
	double getOmega() const { return Omega; };
	double getGear() const { return gearNum; };
	double getT_m() const { return engineTorque; };
	double getRpm() const { return RPM; };
	double getSteer_w1() const { return steer_w1 * 180 / std::numbers::pi; };
	double getSteer_w4() const { return steer_w4 * 180 / std::numbers::pi; };
	double getX() const { return X; };
	double getY() const { return Y; };
	double getYaw() const { return Yaw * 180 / std::numbers::pi; };
	QVector<double> getBrakeForces() const { QMutexLocker locker(&bufferMutex); return brakeForces; };
	QVector<double> getWheelLoads() const { QMutexLocker locker(&bufferMutex); return tyreLoads; };
	QVector<double> getWheelLambdas() const { QMutexLocker locker(&bufferMutex); return tyreLambdas; };
	QVector<double> getWheelBetas() const { QMutexLocker locker(&bufferMutex); return tyreBetas; };
	QVector<double> getWheelSpeedsX() const { QMutexLocker locker(&bufferMutex); return tyreSpeedsX; };
	QVector<double> getWheelSpeedsY() const { QMutexLocker locker(&bufferMutex); return tyreSpeedsY; };
	QVector<double> getWheelForcesX() const { QMutexLocker locker(&bufferMutex); return tyreForcesX; };
	QVector<double> getWheelForcesY() const { QMutexLocker locker(&bufferMutex); return tyreForcesY; };
	QVector<double> getWheelTorques() const { QMutexLocker locker(&bufferMutex); return tyreTorques; };
	QVector<double> getWheelFLat() const { QMutexLocker locker(&bufferMutex); return tyreFLat; };

public:
	void update(double throttle, double brake, double steer, int gear, double dt)
	{
		QMutexLocker locker(&bufferMutex);  // locks mutex for the scope

		gearNum = gear;

		// Engine torque request
		double T_req = throttle * 700; // Max Request Torque: 700 Nm
		
		// Brake torque request
		double Tb_req = brake * 10000;

		// Front Steering System
		std::tuple<double, double> delta = steering(steer);
		double delt1 = get<0>(delta);
		double delt4 = get<1>(delta);

		steer_w1 = std::get<0>(delta);
		steer_w4 = std::get<1>(delta);

		// Velocity in the Wheel Reference Frame
		double v1_x = std::cos(delt1) * (Vx - cF / 2 * Omega) + std::sin(delt1) * (Vy + a * Omega);
		double v2_x = std::cos(0.000) * (Vx - cR / 2 * Omega) + std::sin(0.000) * (Vy - b * Omega);
		double v3_x = std::cos(0.000) * (Vx + cR / 2 * Omega) + std::sin(0.000) * (Vy - b * Omega);
		double v4_x = std::cos(delt4) * (Vx + cF / 2 * Omega) + std::sin(delt4) * (Vy + a * Omega);

		double v1_y = std::cos(delt1) * (Vy + a * Omega) - std::sin(delt1) * (Vx - cF / 2 * Omega);
		double v2_y = std::cos(0.000) * (Vy - b * Omega) - std::sin(0.000) * (Vx - cR / 2 * Omega);
		double v3_y = std::cos(0.000) * (Vy - b * Omega) - std::sin(0.000) * (Vx + cR / 2 * Omega);
		double v4_y = std::cos(delt4) * (Vy + a * Omega) - std::sin(delt4) * (Vx + cF / 2 * Omega);

		tyreSpeedsX = { v1_x, v2_x, v3_x, v4_x };
		tyreSpeedsY = { v1_y, v2_y, v3_y, v4_y };

		tyreLambdas[0] = lambda(v1_x, Omega1 * rw_f);
		tyreLambdas[1] = lambda(v2_x, Omega2 * rw_r);
		tyreLambdas[2] = lambda(v3_x, Omega3 * rw_r);
		tyreLambdas[3] = lambda(v4_x, Omega4 * rw_f);

		double f1_L = mu_l(tyreLambdas[0]) - rollingResistance(v1_x);
		double f2_L = mu_l(tyreLambdas[1]) - rollingResistance(v2_x);
		double f3_L = mu_l(tyreLambdas[2]) - rollingResistance(v3_x);
		double f4_L = mu_l(tyreLambdas[3]) - rollingResistance(v4_x);

		tyreFLong = { f1_L, f2_L, f3_L, f4_L };

		tyreBetas[0] = beta(v1_x, v1_y);
		tyreBetas[1] = beta(v2_x, v2_y);
		tyreBetas[2] = beta(v3_x, v3_y);
		tyreBetas[3] = beta(v4_x, v4_y);

		// Transverse Forces per Unit of Vertical Load
		double f1_T = mu_t(tyreBetas[0]);
		double f2_T = mu_t(tyreBetas[1]);
		double f3_T = -mu_t(tyreBetas[2]);
		double f4_T = -mu_t(tyreBetas[3]);

		tyreFLat = { f1_T, f2_T, f3_T, f4_T };

		// Front Tyre Forces per Unit of Vertical Load in the Body Reference Frame
		double f1_x = f1_L * std::cos(delt1) - f1_T * std::sin(delt1);
		double f2_x = f2_L * std::cos(0.000) - f2_T * std::sin(0.000);
		double f3_x = f3_L * std::cos(0.000) - f3_T * std::sin(0.000);
		double f4_x = f4_L * std::cos(delt4) - f4_T * std::sin(delt4);

		double f1_y = f1_L * std::sin(delt1) + f1_T * std::cos(delt1);
		double f2_y = f2_L * std::sin(0.000) + f2_T * std::cos(0.000);
		double f3_y = f3_L * std::sin(0.000) + f3_T * std::cos(0.000);
		double f4_y = f4_L * std::sin(delt4) + f4_T * std::cos(delt4);

		// Downforce
		double Dz = 0.5 * cl * rho * af * std::pow(Vx, 2);

		// Vertical Wheel Load
		QVector<QVector<double>> A_qvec = {
			{ -f1_x * h - a,     -f2_x * h + b,     -f3_x * h + b,      -f4_x * h - a },
			{  f1_y * h + (cF/2), f2_y * h + (cR/2), f3_y * h + (-cR/2), f4_y * h + (-cF/2) },
			{ 1, 1, 1, 1 }
		};
		QVector<double> b_qvec = { 0, 0, Mcar * 9.81 + Dz };

		// Convert in Eigen format
		MatrixXd A(3, 4);
		for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 4; ++j)
				A(i, j) = A_qvec[i][j];

		VectorXd b_vec(3);
		for (int i = 0; i < 3; ++i)
			b_vec(i) = b_qvec[i];

		// Calculate the pseudo-inverse of As
		VectorXd N = pinv(A) * b_vec;
		tyreLoads = toQVector(N);

		// Tyre Forces in the Body Reference Frame
		double F1_x = tyreLoads[0] * f1_x;
		double F2_x = tyreLoads[1] * f2_x;
		double F3_x = tyreLoads[2] * f3_x;
		double F4_x = tyreLoads[3] * f4_x;

		tyreForcesX = { F1_x, F2_x, F3_x, F4_x };

		// Tyre Forces along the Body y axis
		double F1_y = tyreLoads[0] * f1_y - k_lat_damp * v1_y;
		double F2_y = tyreLoads[1] * f2_y - k_lat_damp * v2_y;
		double F3_y = tyreLoads[2] * f3_y - k_lat_damp * v3_y;
		double F4_y = tyreLoads[3] * f4_y - k_lat_damp * v4_y;

		tyreForcesY = { F1_y, F2_y, F3_y, F4_y };

		// Torques generated by the Tyre Forces on the Vehicle along the Body Z axis (tau = x*F_y - y*F_x)
		double Tau1 =  a * F1_y - ( cF / 2) * F1_x;
		double Tau2 = -b * F2_y - ( cR / 2) * F2_x;
		double Tau3 = -b * F3_y - (-cR / 2) * F3_x;
		double Tau4 =  a * F4_y - (-cF / 2) * F4_x;

		tyreTorques = { Tau1, Tau2, Tau3, Tau4 };

		// Areodynamic Drag
		double v = sqrt(Vx * Vx + Vy * Vy);
		double Fdrag = 0.5 * rho * af * cd * std::pow(v, 2.0);
		double Fx_drag = Fdrag * (Vx / v);
		double Fy_drag = Fdrag * (Vy / v);

		// Engine torque
		double n_ice2 = n_m.first();
		double n_ice3 = n_m.first();
		double t_d2 = 0.0;
		double t_d3 = 0.0;
		if (gear > 0)
		{
			// Engine speed
			double n_2 = Omega2 * 30 / std::numbers::pi; // [rpm] speed of wheel 2
			double n_3 = Omega3 * 30 / std::numbers::pi; // [rpm] speed of wheel 3

			n_ice2 = n_2 * rt[gear - 1]; // [rpm] speed of wheel 2 seen in ICE
			n_ice2 = std::min(n_m.last(), std::max(n_m.first(), n_ice2));
			n_ice3 = n_3 * rt[gear - 1]; // [rpm] speed of wheel 3 seen in ICE
			n_ice3 = std::min(n_m.last(), std::max(n_m.first(), n_ice3));

			double Tice_2 = 0.0;
			if (Vx >= 0.1 && throttle < 0.05)
				Tice_2 = -0.1 * 0.5 * linearInterp(n_m, T_m, n_ice2); // Engine braking
			else if ((n_ice2 < n_m.last() || Vx < 0.1) && throttle >= 0.05)
				Tice_2 = 0.5 * linearInterp(n_m, T_m, n_ice2);	// Engine traction
	
			double Tice_3 = 0.0;
			if (Vx >= 0.1 && throttle < 0.05)
				Tice_3 = -0.1 * 0.5 * linearInterp(n_m, T_m, n_ice3);
			else if ((n_ice3 < n_m.last() || Vx < 0.1) && throttle >= 0.05)
				Tice_3 = 0.5 * linearInterp(n_m, T_m, n_ice3);

			t_d2 = Tice_2 * rt[gear - 1];
			t_d3 = Tice_3 * rt[gear - 1];
		}
		engineTorque = t_d2 + t_d3;
		RPM = (n_ice2 + n_ice3) / 2;

		// Brakes
		double F_req_f = Tb_req / rw_f;
		double F_req_r = Tb_req / rw_r;

		double F_bias_f = bb * F_req_f;
		double F_bias_r = (1 - bb) * F_req_r;

		// Brake forces
		double Fbr_1 = std::max(0.0, std::min(N[0], F_bias_f * 0.5));
		double Fbr_2 = std::max(0.0, std::min(N[1], F_bias_r * 0.5));
		double Fbr_3 = std::max(0.0, std::min(N[2], F_bias_r * 0.5));
		double Fbr_4 = std::max(0.0, std::min(N[3], F_bias_f * 0.5));

		brakeForces = { Fbr_1, Fbr_2, Fbr_3, Fbr_4 };

		double t_br1 = 0.0;
		double t_br2 = 0.0;
		double t_br3 = 0.0;
		double t_br4 = 0.0;
		if (Vx > 0.1 && brake > 0.01)
		{
			// Brake torques
			t_br1 = Fbr_1 * rw_f;
			t_br2 = Fbr_2 * rw_r;
			t_br3 = Fbr_3 * rw_r;
			t_br4 = Fbr_4 * rw_f;
		}
		
		// Total forces and torques
		double Fx_total = F1_x + F2_x + F3_x + F4_x - Fx_drag;
		double Fy_total = F1_y + F2_y + F3_y + F4_y - Fy_drag;
		double Tau_total = Tau1 + Tau2 + Tau3 + Tau4 - k_yaw_damp * Omega;

		// Dynamics
		double dvx_dt = Fx_total / Mcar - Vy * Omega;
		double dvy_dt = Fy_total / Mcar + Vx * Omega;
		double dw_dt = Tau_total / Iz;

		if (isnan(dvx_dt))
			return;
		Vx += dvx_dt* dt;
		if (Vx < 0.001 && brake > 0.01)
			Vx = 0.001;
		Vy += dvy_dt * dt;
		Omega += dw_dt * dt;
		X += (Vx * cos(Yaw) - Vy * sin(Yaw)) * dt;
		Y += (Vx * sin(Yaw) + Vy * cos(Yaw)) * dt;
		Yaw += Omega * dt;

		// Transform body-frame forces back into wheel coordinates
		double Fx_w1 = cos(delt1) * F1_x + sin(delt1) * F1_y;
		double Fx_w2 = cos(0.000) * F2_x + sin(0.000) * F2_y;
		double Fx_w3 = cos(0.000) * F3_x + sin(0.000) * F3_y;
		double Fx_w4 = cos(delt4) * F4_x + sin(delt4) * F4_y;

		double dw1_dt = (- t_br1 - Fx_w1 * rw_f) / I_wf;
		double dw2_dt = (t_d2 - t_br2 - Fx_w2 * rw_r) / I_wr;
		double dw3_dt = (t_d3 - t_br3 - Fx_w3 * rw_r) / I_wr;
		double dw4_dt = (- t_br4 - Fx_w4 * rw_f) / I_wf;
		Omega1 += dw1_dt * dt;
		Omega2 += dw2_dt * dt;
		Omega3 += dw3_dt * dt;
		Omega4 += dw4_dt * dt;
	}

private:
	// Burckhardt Tyre Force Model (longitudinal)
	double mu_l(double s)
	{
		double sign = 1;
		if (s < 0)
			sign = -1;
		return sign * Cf_l[0] * (1 - std::exp(-std::abs(s) * Cf_l[1])) - s * Cf_l[2];
	}

	// Burckhardt Tyre Force Model (lateral)
	double mu_t(double s)
	{
		double S_h = -0.005;
		s = s - S_h;
		double sign = 1;
		if (s < 0)
			sign = -1;
		return sign * Cf_t[0] * (1 - std::exp(-std::abs(s) * Cf_t[1])) - s * Cf_t[2];
	}

	// Longitudinal Slip Ratio
	double lambda(double vx, double vrad)
	{
		const double eps = 1e-4;
		if (std::abs(vx) < eps) return 0.0; // avoid division by zero

		return (vrad - vx) / (std::max(std::max(std::abs(vrad), std::abs(vx)), std::abs(vrad - vx)));
		//return (vrad - vx) / (std::max(std::max(std::abs(vrad), std::abs(vx)), std::abs(vrad - vx)) + 1e-9);
	}

	// Lateral Slip Ratio
	double beta(double vx, double vy, double vrad = 0)
	{
		const double eps = 1e-4;
		if (std::abs(vx) < eps) return 0.0; // avoid division by zero

		return std::atan(vy/std::abs(vx));//(vy + vrad) / (vx));

		//return -std::atan(vy / (vx + (1.0 / std::cosh(vx)) / 1e9)) / (std::numbers::pi / 2);
	}

	double rollingResistance(double vx)
	{
		double sign = 1.0;
		if (vx < 0)
			sign = -1.0;
		return (std::abs(Vx) > 1E-04 ? sign * Crr : 0.0);
	}

	// Steering system
	std::tuple<double, double> steering(double deltaS)
	{
		// Steering sensitivity
		double k = 0.1;
		// Istantaneous turning radius
		double invR = k * deltaS;

		double delta1 = -std::atan((a + b) * invR / (1 - cF/2 * invR));
		double delta2 = -std::atan((a + b) * invR / (1 + cF/2 * invR));
		return { delta1, delta2 };
	}

	// Funzione per pseudoinversa (via SVD)
	MatrixXd pinv(const MatrixXd& A, double tol = -1.0)
	{
		JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV); // thin = dimensioni ridotte
		VectorXd S = svd.singularValues(); // r x 1
		MatrixXd U = svd.matrixU();        // m x r
		MatrixXd V = svd.matrixV();        // n x r

		double eps = std::numeric_limits<double>::epsilon();
		if (tol <= 0)
			tol = std::max(A.rows(), A.cols()) * S.maxCoeff() * eps;

		// pseudoinversa dei valori singolari
		VectorXd Sinv = S;
		for (int i = 0; i < S.size(); ++i)
			Sinv(i) = (S(i) > tol) ? (1.0 / S(i)) : 0.0;

		// V (n x r) * S^+ (r x r) * U^T (r x m) => n x m 
		return V * Sinv.asDiagonal() * U.transpose();
	}

	QVector<double> toQVector(const VectorXd& vec)
	{
		QVector<double> result(vec.size());
		for (int i = 0; i < vec.size(); ++i) {
			double val = vec(i);
			val = std::floor(val * 10000.0) / 10000.0;  // truncate
			result[i] = val;
		}
		return result;
	}

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

	double getVectorSign(double value)
	{
		return (value >= 0) ? 1.0 : -1.0;
	}
};