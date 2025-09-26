#pragma once

#include <QtWidgets>
#include <QTimer>
#include <QtMath>
#include <QGamepad>
#include <variant>
#include <dinput.h>
#include "qcustomplot.h"

#include "VehicleTelemetry.h"
#include "XdofVehicleTelemetry.h"

#include "PlotWidget.h"
#include "XyPlotWidget.h"
#include "ui_MainWindow.h"

using namespace std;
using Objects = std::variant<std::shared_ptr<VehicleTelemetry>, std::shared_ptr<XdofVehicleTelemetry>>;

const double dt = 1E-03; //1E-05;
const double millis = 33.0; // 30 FPS

class MainWindow : public QMainWindow, Ui::MainWindow
{
	Q_OBJECT
public:
	MainWindow(QWidget* parent = nullptr);
	~MainWindow() {
		running = false;
		if (simThread.joinable())
			simThread.join();

		if (g_pJoystick) {
			g_pJoystick->Unacquire();
			g_pJoystick->Release();
		}
		if (g_pDI) g_pDI->Release();
	};

protected:
	QString selectedObject;
	Objects* selectedSimulation = nullptr;
	std::map<QString, Objects> objectsMap = {
		{ "Electric Car Longitudinal", std::make_shared<VehicleTelemetry>() },
		{ "XDOF Car Model", std::make_shared<XdofVehicleTelemetry>() }
	};

protected:
	QTimer* timer;
	QSet<int> keysHeld;     // track pressed keys
	QSet<int> keysReleased; // track released keys

	QList<PlotWidget*> plotWidgets;
	QList<XyPlotWidget*> xyPlotWidgets;

	double simTime;
	QElapsedTimer* realClock;   // wall-clock timer
	
	QGamepad* gamepad;
	double throttlePercent;
	double brakePercent;
	double steering;

	QElapsedTimer lastShiftTime;
	int gear;

	LPDIRECTINPUT8 g_pDI = nullptr;
	LPDIRECTINPUTDEVICE8 g_pJoystick = nullptr;

protected:
	void keyPressEvent(QKeyEvent* event) override;
	void keyReleaseEvent(QKeyEvent* event) override;

	bool initDirectInput();
	static BOOL CALLBACK EnumJoysticksCallback(const DIDEVICEINSTANCE* pdidInstance, VOID* pContext);
	void pollDirectInput();

private:
	std::thread simThread;
	bool running = true;
	void simulateLoop();
	void simulateStep();

private slots:
	void on_actionAdd_Plot_triggered();
	void on_actionAdd_XY_Plot_triggered();
	void updateData(double realTime);
};