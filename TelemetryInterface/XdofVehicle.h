#pragma once

#include <QMutex>
#include <QList>
#include <Dense>

#include "Engine.h"
#include "Tyre.h"

using namespace Eigen;

const double CzSmax = 5.2; // [max lift x frontal area]
const double CxSmax = 1;   // [max drag x frontal area]
const double rho = 1.2041; // [kg / m^3] Standard air density

class XdofVehicle
{
public:
	XdofVehicle()
	{
		this->engine = new Engine();
		this->flTyre = new Tyre(eCorner::FL,  a,  cF / 2, 0.349, 0);
		this->rlTyre = new Tyre(eCorner::RL, -b,  cR / 2, 0.347, 0);
		this->rrTyre = new Tyre(eCorner::RR, -b, -cR / 2, 0.347, 0);
		this->frTyre = new Tyre(eCorner::FR,  a, -cF / 2, 0.349, 0);
	}
	~XdofVehicle() {};

public:
	mutable QMutex bufferMutex;
	Tyre* flTyre;
	Tyre* frTyre;
	Tyre* rlTyre;
	Tyre* rrTyre;
	Engine* engine;

private:
	// States
	double Vx = 10;
	double Vy = 0.0;
	double Omega = 0.0;

	double X = 0;	// [m] Global X position
	double Y = 0;	// [m] Global Y position
	double Yaw = 0;	// [rad] Yaw angle

protected:
	// Car Parameters
	double Mcar = 1030;	// [kg] car mass (no driver)
	double Iz = 3.176 * Mcar - 1754.164; // [kg * m^2] car inertia
	double Wb = 3.148;	// [m] Wheel base
	double Wdis = 45;	// [%] Weight distribution
	double a = (1 - Wdis * 0.01) * Wb; // [m] Front tyres to CoG distance
	double b = Wdis * 0.01 * Wb;  // [m] Rear tyres to CoG distance
	double cF = 1.650; // [m] Front track length
	double cR = 1.614; // [m] Rear track length
	double h = 0.00012 * Mcar + 0.14338; // [m] CoG height
	double af = 1.8; // [m^2] frontal section area
	double cd = (0.8 * CxSmax) / af; // [-] Drag coefficient
	double cl = CzSmax / af; // [-] Lift coefficient
	const double k_yaw_damp = 1000.0; // [Nm/rad]

protected:
	// Measurements
	double steer_w1 = 0;	// [rad]
	double steer_w4 = 0;	// [rad]
	int gearNum = 0; // [-]
	QVector<double> tyreLoads = { 0.0, 0.0, 0.0, 0.0 };

public:
	double getVx() const { return Vx * 3.6; };
	double getVy() const { return Vy * 3.6; };
	double getOmega() const { return Omega; };
	double getGear() const { return gearNum; };
	double getX() const { return X; };
	double getY() const { return Y; };
	double getYaw() const { return Yaw * 180 / std::numbers::pi; };
	double getSteer_w1() const { return steer_w1 * 180 / std::numbers::pi; };
	double getSteer_w4() const { return steer_w4 * 180 / std::numbers::pi; };
	QVector<double> getWheelLoads() const { QMutexLocker locker(&bufferMutex); return tyreLoads; };

public:
	void update(double throttle, double brake, double steer, int gear, double dt)
	{
		QMutexLocker locker(&bufferMutex);  // locks mutex for the scope

		gearNum = gear;

		// Engine torque request
		double T_req = throttle * 700;

		// Brake torque request
		double Tb_req = brake * 10000;

		// Front Steering System
		std::tuple<double, double> delta = steering(steer);
		steer_w1 = std::get<0>(delta);
		steer_w4 = std::get<1>(delta);

		QList<Tyre*> tyres = { this->flTyre, this->rlTyre, this->rrTyre, this->frTyre };
		QList<double> steering = { steer_w1, 0.0, 0.0, steer_w4 };
		for (int i = 0; i < tyres.size(); i++)
		{
			// Calculate tyre forces per unit of load
			tyres[i]->calculateForcePerUnitLoad(Vx, Vy, Omega, steering[i]);
		}
		
		// Areodynamic Downforce
		double Dz = 0.5 * cl * rho * af * std::pow(Vx, 2);

		// Vertical Wheels Load
		QVector<QVector<double>> A_qvec = {
			{ -this->flTyre->getFx() * h - a,       -this->rlTyre->getFx() * h - (-b),    -this->rrTyre->getFx() * h - (-b),     -this->frTyre->getFx() * h - a },
			{  this->flTyre->getFy() * h + (cF / 2), this->rlTyre->getFy() * h + (cR / 2), this->rrTyre->getFy() * h + (-cR / 2), this->frTyre->getFy() * h + (-cF / 2) },
			{ 1, 1, 1, 1 }
		};
		QVector<double> b_qvec = { 0, 0, Mcar * 9.81 + Dz };
		QVector<double> N = calculateLoad(A_qvec, b_qvec);
		tyreLoads = N;
		
		double Fw_x = 0.0;
		double Fw_y = 0.0;
		double Tau_w = 0.0;
		for (int i = 0; i < tyres.size(); i++)
		{
			tyres[i]->calculateForce(N[i]);

			// Car forces
			Fw_x += tyres[i]->getFb_x();
			Fw_y += tyres[i]->getFb_y();

			// Car Torque (tau = x * F_y - y * F_x)
			Tau_w += (tyres[i]->getXpos() * tyres[i]->getFb_y() - tyres[i]->getYpos() * tyres[i]->getFb_x());
		}

		// Areodynamic Drag
		double v = sqrt(Vx * Vx + Vy * Vy);
		double Fdrag = 0.5 * rho * af * cd * std::pow(v, 2.0);
		double Fx_drag = 0.0;
		double Fy_drag = 0.0;
		if (std::abs(v) > 1E-04)
		{
			Fx_drag = Fdrag * (Vx / v);
			Fy_drag = Fdrag * (Vy / v);
		}

		// Engine torque
		this->engine->update(Vx, this->rlTyre->getTyreOmega(), this->rrTyre->getTyreOmega(), throttle, gear);

		QList<double> engineTorques = { 0.0, this->engine->getEngineTorque2(), this->engine->getEngineTorque3(), 0.0 };
		for (int i = 0; i < tyres.size(); i++)
		{
			// Mechanical brakes
			tyres[i]->calculateBrakeTorque(Vx, N[i], Tb_req, brake);
			// Update tyre rotational speed
			tyres[i]->update(engineTorques[i], tyres[i]->getBrakeTorque(), steering[i], dt);
		}

		// Total forces and torques
		double Fx_total = Fw_x - Fx_drag;
		double Fy_total = Fw_y - Fy_drag;
		double Tau_total = Tau_w - k_yaw_damp * Omega;

		// Dynamics
		double dvx_dt = Fx_total / Mcar - Vy * Omega;
		double dvy_dt = Fy_total / Mcar + Vx * Omega;
		double dw_dt = Tau_total / Iz;

		Vx += dvx_dt * dt;
		if (Vx < 0.001 && brake > 0.01)
			Vx = 0.001;
		Vy += dvy_dt * dt;
		Omega += dw_dt * dt;

		X += (Vx * cos(Yaw) - Vy * sin(Yaw)) * dt;
		Y += (Vx * sin(Yaw) + Vy * cos(Yaw)) * dt;
		Yaw += Omega * dt;
	}

private:
	// Steering system
	std::tuple<double, double> steering(double deltaS)
	{
		// Steering sensitivity
		double k = 0.1;
		// Istantaneous turning radius
		double invR = k * deltaS;

		double delta1 = -std::atan((a + b) * invR / (1 - cF / 2 * invR));
		double delta2 = -std::atan((a + b) * invR / (1 + cF / 2 * invR));
		return { delta1, delta2 };
	}

	// Tyre Load calc
	QVector<double> calculateLoad(QVector<QVector<double>> A_qvec, QVector<double> b_qvec)
	{
		// Convert in Eigen format
		MatrixXd A(3, 4);
		for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 4; ++j)
				A(i, j) = A_qvec[i][j];

		VectorXd b_vec(3);
		for (int i = 0; i < 3; ++i)
			b_vec(i) = b_qvec[i];
		
		// Pseudoinverse function (via SVD)
		JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV); // thin = dimensioni ridotte
		VectorXd S = svd.singularValues(); // r x 1
		MatrixXd U = svd.matrixU();        // m x r
		MatrixXd V = svd.matrixV();        // n x r

		double eps = std::numeric_limits<double>::epsilon();
		double tol = std::max(A.rows(), A.cols()) * S.maxCoeff() * eps;

		// pseudoinversa dei valori singolari
		VectorXd Sinv = S;
		for (int i = 0; i < S.size(); ++i)
			Sinv(i) = (S(i) > tol) ? (1.0 / S(i)) : 0.0;

		// V (n x r) * S^+ (r x r) * U^T (r x m) => n x m 
		MatrixXd A_pinv = V * Sinv.asDiagonal() * U.transpose();
		VectorXd N = A_pinv * b_vec;

		QVector<double> result(N.size());
		for (int i = 0; i < N.size(); ++i) {
			double val = N(i);
			val = std::floor(val * 10000.0) / 10000.0;  // truncate
			result[i] = val;
		}
		return result;
	}
};