#pragma once

#include <QtWidgets>
#include <QTimer>
#include <QtMath>
#include <QGamepad>
#include "qcustomplot.h"

#include "VehicleTelemetry.h"
#include "PlotWidget.h";
#include "ui_MainWindow.h"

using namespace std;

const double dt = 1E-05;
const double millis = 1.0 / 33.0;

const double fsw = 5000;
const double speed_ref = 100.0;
const double id_target = 0.0;
const double tau_ref = 200;

class MainWindow : public QMainWindow, Ui::MainWindow
{
	Q_OBJECT
public:
	MainWindow(QWidget* parent = nullptr);
	~MainWindow() {
		running = false;
		if (simThread.joinable())
			simThread.join();
	};

public:
	VehicleTelemetry* vehTelem;

protected:
	QTimer* timer;
	QSet<int> keysHeld;     // track pressed keys
	QSet<int> keysReleased; // track released keys

	QList<PlotWidget*> plotWidgets;

	double simTime;
	QElapsedTimer* realClock;   // wall-clock timer
	
	QGamepad* gamepad;
	double throttlePercent;
	double brakePercent;

protected:
	void keyPressEvent(QKeyEvent* event) override;
	void keyReleaseEvent(QKeyEvent* event) override;

private:
	std::thread simThread;
	bool running = true;
	void simulateLoop();
	void simulateStep();

private slots:
	void on_actionAdd_Plot_triggered();
	void updateData(double realTime);
};