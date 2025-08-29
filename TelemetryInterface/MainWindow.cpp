#include "MainWindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
	this->setupUi(this);
	setWindowTitle("Telemetry Interface");

	// Add telemetry selection
	QToolBar* toolbar = new QToolBar("Top Toolbar", this);
	addToolBar(Qt::TopToolBarArea, toolbar);
	QComboBox* combo = new QComboBox();
	combo->setFixedWidth(300);
	QStringList keys;
	for (const auto& pair : objectsMap) {
		keys.push_back(pair.first);
	}
	combo->addItems(keys);
	combo->setCurrentIndex(-1);
	connect(combo, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, [=]() {
		selectedObject = combo->currentText();
		auto it = objectsMap.find(selectedObject);
		if (it != objectsMap.end()) {
			selectedSimulation = &(it->second);   // store pointer to active simulation object
			// Update plots comboboxes
			for (PlotWidget* widget : plotWidgets)
			{
				if (!widget) continue;

				std::visit([&](auto& sim) {
					widget->setSelectableChannels(sim->getChannels());
					widget->setAvailableChannels(sim->getMap());
				}, *selectedSimulation);
			}
		}
	});
	toolbar->addWidget(combo);
	
	// Start simulation thread
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
		throttlePercent = ((value * 1) / 0.85) * 100;
		throttlePercent = std::clamp(throttlePercent, 0.0, 100.0);
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
	if (!selectedSimulation)
		return;

	// Initialize widget
	PlotWidget* widget = new PlotWidget();
	std::visit([&](auto& sim) {
		widget->setSelectableChannels(sim->getChannels());
		widget->setAvailableChannels(sim->getMap());
	}, *selectedSimulation);
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
	if (selectedSimulation) {
		std::visit([&](auto& sim) {
			using T = std::decay_t<decltype(sim)>;

			if constexpr (std::is_same_v<T, std::shared_ptr<VehicleTelemetry>>) {
				sim->simulateStep(throttlePercent, brakePercent, simTime, dt);
			}
			else if constexpr (std::is_same_v<T, std::shared_ptr<XdofVehicleTelemetry>>) {
				sim->simulateStep(dt);
			}
		}, *selectedSimulation);
	}
}

void MainWindow::updateData(double realTime)
{
	if (keysReleased.contains(Qt::Key_Up) && throttlePercent > 0)
		throttlePercent -= 5;
	if (keysHeld.contains(Qt::Key_Up))
		throttlePercent += 5;

	if (keysReleased.contains(Qt::Key_Down) && brakePercent > 0)
		brakePercent -= 5;
	if (keysHeld.contains(Qt::Key_Down))
		brakePercent += 5;

	throttlePercent = std::clamp(throttlePercent, 0.0, 100.0);
	brakePercent = std::clamp(brakePercent, 0.0, 100.0);

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