#pragma once

#include <QVector>
#include <cmath>

class Carrier
{
public:
	Carrier() {
		this->T = 1.0 / Fsw;
	};
	~Carrier() {};

public:
	double getFrequency() const { return Fsw; };

protected:
	double Fsw = 5000; // Frequenza di commutazione
	double T;		   // Periodo di commutazione

public:
	double getCarrierValue(double t)
	{
		// Modulo t rispetto al periodo per ottenere il tempo nel ciclo
		double t_mod = std::fmod(t, T);

		// Punti di tempo e valori per l'interpolazione
		QVector<double> time_points = { 0, T / 2, T };
		QVector<double> values = { -1, 1, -1 };

		// Interpolazione lineare per ottenere il valore del carrier
		return Interpolate(time_points, values, t_mod);
	}

private:
	// Metodo per l'interpolazione lineare
	double Interpolate(QVector<double> time_points, QVector<double> values, double t_mod)
	{
		// Troviamo i due punti più vicini in tempo
		if (t_mod <= time_points[1])
		{
			// Interpoliamo tra 0 e T/2
			return LinearInterpolation(time_points[0], values[0], time_points[1], values[1], t_mod);
		}
		else
		{
			// Interpoliamo tra T/2 e T
			return LinearInterpolation(time_points[1], values[1], time_points[2], values[2], t_mod);
		}
	}

	// Funzione di interpolazione lineare
	double LinearInterpolation(double t0, double v0, double t1, double v1, double t)
	{
		return v0 + (v1 - v0) * (t - t0) / (t1 - t0);
	}
};

	