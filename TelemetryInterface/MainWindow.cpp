#include "MainWindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
	this->setupUi(this);
	setWindowTitle("Telemetry Interface");

	if (!initDirectInput()) {
		qDebug() << "Failed to initialize DirectInput!";
	}

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
	steering = 0.0;

	gear = 0;
	lastShiftTime.start();

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

	connect(gamepad, &QGamepad::axisLeftXChanged, this, [this](double value) {
		steering = value;
	});

	connect(gamepad, &QGamepad::buttonAChanged, this, [this](bool pressed) {
		if (pressed && gear < 7 && lastShiftTime.elapsed() > 200)
		{
			gear++; // Gear up
			lastShiftTime.restart();
		}
	});

	connect(gamepad, &QGamepad::buttonBChanged, this, [this](bool pressed) {
		if (pressed && gear > 1 && lastShiftTime.elapsed() > 200)
		{
			gear--; // Gear down
			lastShiftTime.restart();
		}
	});

	connect(gamepad, &QGamepad::buttonStartChanged, this, [this](bool pressed) {
		gear = 0; // Neutral
	});

	timer = new QTimer(this);
	connect(timer, &QTimer::timeout, this, [=]() {
		pollDirectInput();   // <- poll T150 state here
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

void MainWindow::on_actionAdd_XY_Plot_triggered()
{
	if (!selectedSimulation)
		return;
	
	// Initialize widget
	XyPlotWidget* widget = new XyPlotWidget();
	std::visit([&](auto& sim) {
		widget->setSelectableChannels(sim->getChannels());
		widget->setAvailableChannels(sim->getMap());
	}, *selectedSimulation);
	xyPlotWidgets.append(widget);
	QDockWidget* dock = new QDockWidget("XY-Plot " + QString::number(this->xyPlotWidgets.size()), this);
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
				sim->simulateStep(throttlePercent, brakePercent, gear, steering, dt);
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

	for (XyPlotWidget* widget : xyPlotWidgets)
	{
		if (!widget) continue;

		widget->updatePlot();
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

BOOL CALLBACK MainWindow::EnumJoysticksCallback(const DIDEVICEINSTANCE* pdidInstance, VOID* pContext) {
	MainWindow* window = reinterpret_cast<MainWindow*>(pContext);
	if (FAILED(window->g_pDI->CreateDevice(pdidInstance->guidInstance, &window->g_pJoystick, NULL))) {
		return DIENUM_CONTINUE;
	}
	return DIENUM_STOP;
}

bool MainWindow::initDirectInput() {
	if (FAILED(DirectInput8Create(GetModuleHandle(NULL), DIRECTINPUT_VERSION, IID_IDirectInput8,
		(VOID**)&g_pDI, NULL))) {
		return false;
	}

	if (FAILED(g_pDI->EnumDevices(DI8DEVCLASS_GAMECTRL, EnumJoysticksCallback,
		this, DIEDFL_ATTACHEDONLY))) {
		return false;
	}

	if (!g_pJoystick) return false;

	if (FAILED(g_pJoystick->SetDataFormat(&c_dfDIJoystick2))) return false;

	if (FAILED(g_pJoystick->SetCooperativeLevel((HWND)winId(),
		DISCL_NONEXCLUSIVE | DISCL_BACKGROUND)))
		return false;

	g_pJoystick->Acquire();
	return true;
}

void MainWindow::pollDirectInput() {
	if (!g_pJoystick) return;

	DIJOYSTATE2 js;
	ZeroMemory(&js, sizeof(js));

	if (FAILED(g_pJoystick->Poll())) {
		g_pJoystick->Acquire();
		return;
	}
	if (FAILED(g_pJoystick->GetDeviceState(sizeof(DIJOYSTATE2), &js))) {
		return;
	}

	// Normalize values (example ranges: 0..65535)
	steering = (js.lX - 32767) / 32767.0;   // -1.0 .. +1.0
	brakePercent = (65535 - js.lY) / 65535.0 * 100.0;
	throttlePercent = (65535 - js.lRz) / 65535.0 * 100.0;

	// Paddles as gear shifters
	bool paddleRight = (js.rgbButtons[1] & 0x80);  // Gear up
	bool paddleLeft = (js.rgbButtons[0] & 0x80);  // Gear down
	bool startButton = (js.rgbButtons[12] & 0x80);  // Neutral

	// Debug only - understand which button is what
	//for (int i = 0; i < 32; i++) {
	//	if (js.rgbButtons[i] & 0x80) {
	//		qDebug() << "Button pressed:" << i;
	//	}
	//}

	if (paddleRight) {
		if (gear < 7 && lastShiftTime.elapsed() > 200) {
			gear++;
			lastShiftTime.restart();
		}
	}

	if (paddleLeft) {
		if (gear > 1 && lastShiftTime.elapsed() > 200) {
			gear--;
			lastShiftTime.restart();
		}
	}

	if (startButton) {
		gear = 0;
	}
}