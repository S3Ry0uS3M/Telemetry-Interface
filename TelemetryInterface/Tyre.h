#pragma once

#include <QVector>
#include <cmath>
#include <algorithm>

enum eCorner
{
	FL,
	FR,
	RL,
	RR
};

struct TyreModel
{
	double c1 = 0.0;
	double c2 = 0.0;
	double c3 = 0.0;
};

/* Road types:
1) Dry asphalt
2) wet asphalt
3) Snow
4) Ice
5) Dry Cobblestone
6) wet cobblestone*/
const QVector<QVector<double>> Theta_l = {
	{ 1.2891, 0.86, 0.19, 0.05, 1.37, 0.4 },
	{ 23.99, 33.82, 94.13, 306.39, 6.46, 33.71 },
	{ 0.52, 0.35, 0.05, 0, 0.67, 0.12 }
};
const QVector<QVector<double>> Theta_t = {
	{ 1.2891, 0.86, 0.19, 0.05, 1.37, 0.4 },
	{ 23.99, 33.82, 94.13, 306.39, 6.46, 33.71 },
	{ 0.52, 0.35, 0.05, 0, 0.67, 0.12 }
};

class Tyre
{
public:
	Tyre(eCorner corner, double x, double y, double radius, int index)
	{
		this->corner = corner;
		this->xPos = x;
		this->yPos = y;
		this->radius = radius;
		this->Iw = 0.5 * mw * std::pow(radius, 2.0);	// [kg m^2] tyre inertia
		this->Cf_l = { Theta_l[0][index], Theta_l[1][index], Theta_l[2][index] };
		this->Cf_t = { Theta_t[0][index], Theta_t[1][index], Theta_t[2][index] };
	}
	~Tyre() {}

private:
	// States
	double Omega = 0.0; // [rad/s] tyre rotational speed

protected:
	// Parameters
	eCorner corner;
	double xPos = 0.0; // [m] pos from CoG of car
	double yPos = 0.0; // [m] pos from CoG of car
	double radius = 0.0; // [m] tyre radius
	double mw = 10; // [kg] tyre mass
	double Iw = 0.0;	// [kg m^2] tyre inertia
	const double Crr = 0.006;	// [-] rolling resistance coefficient
	TyreModel Cf_l; // Longitudinal tyre model coefficients
	TyreModel Cf_t; // Transversal tyre model coefficients
	double bb = 0.55; // Brake bias (front)
	const double k_lat_damp = 1000.0; // [N/(m/s)] lateral tyre damping

protected:
	// Measurements
	double Vx = 0.0;	// [m/s] tyre longitudinal speed in tyre frame
	double Vy = 0.0;	// [m/s] tyre lateral speed in tyre frame
	double Fx = 0.0;	// [N] longitudinal force per unit load
	double Fy = 0.0;	// [N] lateral force per unit load
	double Fb_x = 0.0;	// [N] longitudinal force in body frame
	double Fb_y = 0.0;	// [N] lateral force in body frame
	double longSlip = 0.0; // [-] longitudinal slip ratio
	double latSlip = 0.0; // [rad] lateral slip angle
	double t_brake = 0.0; // [Nm] brake torque
	double f_brake = 0.0; // [N] brake force

public:
	double getTyreOmega() const { return Omega; };
	double getXpos() const { return xPos; };
	double getYpos() const { return yPos; };
	double getTyreVx() const { return Vx; };
	double getTyreVy() const { return Vy; };
	double getFx() const { return Fx; };
	double getFy() const { return Fy; };
	double getFb_x() const { return Fb_x; };
	double getFb_y() const { return Fb_y; };
	double getLongSlip() const { return longSlip; };
	double getLatSlip() const { return latSlip; };
	double getBrakeForce() const { return f_brake; };
	double getBrakeTorque() const { return t_brake; };

public:
	void calculateForcePerUnitLoad(double vx_car, double vy_car, double omega_car, double deltaS)
	{
		// Speed in wheel reference
		double vw_x = vx_car - omega_car * yPos;
		double vw_y = vy_car + omega_car * xPos;
		this->Vx =  std::cos(deltaS) * vw_x + std::sin(deltaS) * vw_y;
		this->Vy = -std::sin(deltaS) * vw_x + std::cos(deltaS) * vw_y;

		// Slip ratio
		this->longSlip = lambda(this->Vx, this->Omega * radius);
		// Slip angle
		this->latSlip = beta(this->Vx, this->Vy);

		// Longitudinal Force per Unit of Vertical Load
		double fw_L = mu_l(longSlip) - rollingResistance(vx_car, this->Vx);

		// Transverse Force per Unit of Vertical Load
		double fw_T = ((corner == eCorner::FR || corner == eCorner::RR) ? -1.0 : 1.0) * mu_t(latSlip);

		// Tyre Force per Unit of Vertical Load in the Body Reference Frame
		Fx = fw_L * std::cos(deltaS) - fw_T * std::sin(deltaS);
		Fy = fw_L * std::sin(deltaS) + fw_T * std::cos(deltaS);
	}

	void calculateForce(double load)
	{
		Fb_x = load * Fx;
		Fb_y = load * Fy - k_lat_damp * Vy;
	}

	void calculateBrakeTorque(double vx_car, double load, double Tb_req, double brake)
	{
		double F_req = Tb_req / radius;
		double F_bias = 0.0;
		if (corner == eCorner::FL ||  corner == eCorner::FR)
			F_bias = bb * F_req;
		else
			F_bias = (1 - bb) * F_req;
		f_brake = std::max(0.0, std::min(load, F_bias * 0.5));

		t_brake = 0.0;
		if (vx_car > 0.1 && brake > 0.01)
			t_brake = f_brake * radius;
	}

	void update(double t_tractive, double t_brake, double deltaS, double dt)
	{
		double Fw_x = cos(deltaS) * Fb_x + sin(deltaS) * Fb_y;
		double dw_dt = (t_tractive - t_brake - Fw_x * radius) / Iw;
		Omega += dw_dt * dt;
	}

private:
	// Burckhardt Tyre Force Model (longitudinal)
	double mu_l(double s)
	{
		double sign = 1;
		if (s < 0)
			sign = -1;
		return sign * Cf_l.c1 * (1 - std::exp(-std::abs(s) * Cf_l.c2)) - s * Cf_l.c3;
	}

	// Burckhardt Tyre Force Model (lateral)
	double mu_t(double s)
	{
		double S_h = -0.005;
		s = s - S_h;
		double sign = 1;
		if (s < 0)
			sign = -1;
		return sign * Cf_t.c1 * (1 - std::exp(-std::abs(s) * Cf_t.c2)) - s * Cf_t.c3;
	}

	// Longitudinal Slip Ratio
	double lambda(double vx, double vrad)
	{
		const double eps = 1e-4;
		if (std::abs(vx) < eps) return 0.0; // avoid division by zero
		
		// Pure rolling condition: vrad = vx -> lambda = 0
		// Driving condition: vrad > vx -> lambda > 0
		// Braking condition: vrad < vx -> lambda < 0
		return (vrad - vx) / (std::max(std::max(std::abs(vrad), std::abs(vx)), std::abs(vrad - vx)));
	}

	// Lateral Slip Ratio
	double beta(double vx, double vy)
	{
		const double eps = 1e-4;
		if (std::abs(vx) < eps) return 0.0; // avoid division by zero

		return std::atan(vy / std::abs(vx));
	}

	// Rolling resistance force per unit load
	double rollingResistance(double vx_car, double vx)
	{
		double sign = 1.0;
		if (vx < 0)
			sign = -1.0;
		return (std::abs(vx_car) > 1E-04 ? sign * Crr : 0.0);
	}
};