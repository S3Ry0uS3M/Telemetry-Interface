#include "MainWindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
	this->setupUi(this);
	setWindowTitle("Telemetry Interface");
	
	// Start simulation thread
	vehTelem = new VehicleTelemetry(fsw);
	realClock = new QElapsedTimer();
	realClock->start();
	simTime = 0.0;
	brakePercent = 0.0;
	throttlePercent = 0.0;
	simThread = std::thread(&MainWindow::simulateLoop, this);

	// Initialize components
	qDebug() << "Connected gamepads:" << QGamepadManager::instance()->connectedGamepads();
	gamepad = new QGamepad(0, this);

	// Connect pedal signals
	connect(gamepad, &QGamepad::buttonL2Changed, this, [this](double value) {
		brakePercent = value * 100;
	});

	connect(gamepad, &QGamepad::buttonR2Changed, this, [this](double value) {
		throttlePercent = value * 100;
	});

	// Example: steering wheel axis
	connect(gamepad, &QGamepad::axisLeftXChanged, this, [](double value) {
		//qDebug() << "Steering:" << value;
	});

	timer = new QTimer(this);
	connect(timer, &QTimer::timeout, this, [=]() {
		// show only up to current real elapsed time
		double realTime = realClock->elapsed() / 1000.0;
		updateData(realTime);
	});
	timer->start(millis);
}

void MainWindow::on_actionAdd_Plot_triggered()
{
	// Initialize widget
	PlotWidget* widget = new PlotWidget();
	widget->setSelectableChannels(this->vehTelem->getChannels());
	widget->setAvailableChannels(this->vehTelem->getMap());
	plotWidgets.append(widget);

	QDockWidget* dock = new QDockWidget("Plot " + QString::number(this->plotWidgets.size()), this);
	dock->setWidget(widget);                         // Assign content
	dock->setFloating(false);                        // Optional: docked by default
	dock->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetClosable);
	dock->raise();
	addDockWidget(Qt::LeftDockWidgetArea, dock);   // Add to main window
}

void MainWindow::simulateLoop()
{
	realClock->start();

	while (running) {
		simulateStep();      // your physics/logic
		simTime += dt;

		// sync to wall-clock
		double realTime = realClock->elapsed() / 1000.0; // ms -> s
		if (simTime > realTime) {
			auto sleepTime = (simTime - realTime) * 1e6; // in us
			std::this_thread::sleep_for(std::chrono::microseconds((int)sleepTime));
		}
	}
}

void MainWindow::simulateStep()
{
	Vehicle* veh = vehTelem;
	Powertrain* powertrain = veh->powertrain;
	//double tau_star = powertrain->pidDriver->update(speed_ref, veh->getSpeed(), dt) * veh->getTractiveRatio();
	
	Motor* motor = powertrain->motor;
	double tau_star = (throttlePercent / 100) * motor->getMaxTorque();
	if (throttlePercent < 1e-3 && veh->getSpeed() > 1.0) {
		tau_star = -motor->getCoastRegenTorque();  // small negative
	}
	if ((veh->getSpeed() != 0 && throttlePercent < 1e-3) || throttlePercent > 1e-3)
	{
		double vd_star = powertrain->pidId->update(id_target, motor->getId(), dt) - motor->getMutualInductaceIq();
		double vq_star = powertrain->pidIq->update(motor->getIqTarget(tau_star), motor->getIq(), dt) + motor->getMutualInductaceId();

		Battery* battery = powertrain->battery;
		battery->update((motor->getMechanicalPowerKW() * 1000) / battery->getVbat(), dt);

		Inverter* inverter = powertrain->inverter;
		inverter->update(battery->getVbat(), vd_star, vq_star, motor->getElectricalAngle(), inverter->carrier->getCarrierValue(simTime), (int)(1.0 / (fsw * dt)));

		motor->update(veh->getOmega(), inverter->getVd(), inverter->getVq(), dt);  // vd_star, vq_star, dt); // inverter->getVd(), inverter->getVq(), dt);
		veh->update(motor->getTorque(), 0, brakePercent / 100, dt);
	}
	else if (throttlePercent < 1e-3)
		veh->update(0, 0, brakePercent / 100, dt);
}

void MainWindow::updateData(double realTime)
{
	//if (keysReleased.contains(Qt::Key_Up) && throttlePercent > 0)
	//	throttlePercent -= 5;
	//if (keysHeld.contains(Qt::Key_Up))
	//	throttlePercent += 5;

	//if (keysReleased.contains(Qt::Key_Down) && brakePercent < 0)
	//	brakePercent += 5;
	//if (keysHeld.contains(Qt::Key_Down))
	//	brakePercent -= 1;

	//throttlePercent = std::clamp(throttlePercent, 0.0, 100.0);
	//brakePercent = std::clamp(brakePercent, -100.0, 0.0);

	for (PlotWidget* widget : plotWidgets)
	{
		if (!widget) continue;

		widget->updatePlot(simTime);
	}
}

void MainWindow::keyPressEvent(QKeyEvent* event)
{
	keysHeld.insert(event->key());
	keysReleased.remove(event->key());
}


void MainWindow::keyReleaseEvent(QKeyEvent* event)
{
	keysHeld.remove(event->key());
	keysReleased.insert(event->key());
}