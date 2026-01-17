# Telemetry-Interface

## Overview
Telemetry-Interface is a C++20/Qt solution for simulating and visualising high-performance vehicle telemetry. The main application (`TelemetryInterface`) provides a dockable Qt Widgets UI, runs physics in a dedicated real-time–synchronised thread, and lets you inspect dozens of channels through scrolling or XY plots. Two simulation models ship out of the box:
1. **Electric Car Longitudinal** (`VehicleTelemetry`), emulating the longitudinal behaviour of an EV powertrain, battery, and inverter.
2. **XDOF Car Model** (`XdofVehicleTelemetry`), a four-wheel model with full vehicle dynamics, Burckhardt tyres, aero load, and traction control.

The solution also contains the `EMotorCharacterization` project (currently scaffolding only) reserved for future motor-characterisation tooling.

## Key Features
- ✅ Dynamic simulation selection via toolbar (std::variant between `VehicleTelemetry` and `XdofVehicleTelemetry`).
- ✅ Simulation thread (`simulateLoop`) that integrates with fixed `dt` and synchronises to a `QElapsedTimer` to stay in real time.
- ✅ Multi-input support: keyboard, `QGamepad` (e.g., Xbox) and DirectInput devices (e.g., Thrustmaster T150) with paddle/button gear shifting.
- ✅ Dockable plots powered by QCustomPlot (`PlotWidget`, `XyPlotWidget`) with live legend, 20 s auto-scale window, and configurable channel combinations.
- ✅ Rich telemetry exposure through `std::map<QString, std::function<double()>>`, ready to extend.
- ✅ Engine audio for the XDOF model using `sf::Music`, with RPM-driven pitch and volume.

## Repository Structure
| Directory/Project | Description |
| --- | --- |
| `TelemetryInterface/` | Qt Widgets application: Designer-generated UIs (`Ui::MainWindow`, `Ui::PlotForm`, `Ui::XyPlotForm`), simulation models, plotting widgets, input manager, and all physics (powertrain, tyres, motor, inverter, battery, PID, etc.). |
| `EMotorCharacterization/` | Stand-alone Visual Studio project, currently containing only `.vcxproj`/`.filters` placeholders for a future characterisation utility. |

## Included Simulations
### 1. Electric Car Longitudinal (`VehicleTelemetry`)
- **Vehicle model (`Vehicle`)**: integrates longitudinal speed using mass, aero/rolling drag, and linear brakes.
- **Powertrain (`Powertrain`)**: couples `Motor`, `Battery`, `Inverter`, and three PID controllers (`pidDriver`, `pidId`, `pidIq`). The virtual driver maps throttle to torque (`tau_star`); current controllers generate `vd`/`vq` commands for the SPWM inverter (`Carrier`).
- **Battery**: Rint + RC branch model with SoC-dependent polynomials and dynamic updates of `SoC`, `Vrc`, and `Vbat`.
- **Motor**: full d-q model (id/iq) delivering torque `1.5 * PolePairs` with SoC-based torque limiting and max-current calculation from voltage limits.
- **Inverter**: space-vector PWM, moving-average filtering to recover sinusoidal phase voltages, inverse dq transform.
- **simulateStep**: computes `tau_star`, updates battery/inverter/motor, integrates the vehicle via `Vehicle::update`, applying regen when throttle is released.

### 2. XDOF Car Model (`XdofVehicleTelemetry`)
- **Dynamics**: states include longitudinal/lateral velocities (`Vx`, `Vy`), yaw rate, and global pose (`X`, `Y`, `Yaw`). Explicit integration with clamping to keep `Vx` stable during heavy braking.
- **Tyres (`Tyre`)**: Burckhardt longitudinal/lateral forces with slip computation, saturation, rolling resistance, and lateral damping. `calculateLoad` relies on Eigen/SVD to solve vertical loads via pseudo-inverse.
- **Aero & drag**: constants `CxSmax`, `CzSmax`, frontal area `af`, air density `rho`, projecting drag along velocity.
- **ICE (`Engine`)**: sampled torque–RPM curve (4400–8300 rpm), gear ratios `gearRatios` + `r_final`, ramp-up/down RPM logic, and torque split to RL/RR. Includes traction control parameters (`lambda_target`, `Kp_tc`, `min_factor`, `v_min_for_tc`).
- **Audio**: `.wav` playback (`TelemetryInterface/Resources/sc63_in_idle.wav`) initialised once, with RPM-based pitch and throttle/RPM-based volume.
- **Thread safety**: `QMutex`/`QMutexLocker` guard shared state such as `tyreLoads`.
- **simulateStep**: handles pedal/brake/steer/gear inputs, updates tyres, computes loads, applies 55% front brake bias, updates wheel spin (`Tyre::update`), and integrates rigid-body dynamics.

## Core Components
- **Powertrain stack**: `Battery`, `Motor`, `Inverter`, `Carrier`, and `PIDController` implement a complete FOC chain.
- **Tyres & brakes**: `Tyre::calculateForcePerUnitLoad`, `calculateForce`, `calculateBrakeTorque`, and `update` produce body-frame forces (`Fb_x`, `Fb_y`) and braking torques.
- **Engine & Traction Control**: `Engine::update` feeds torque to RL/RR; the TC loop trims torque when `longSlip` exceeds `lambda_target` at speeds > `v_min_for_tc`.
- **Plotting**: `PlotWidget` (time series) and `XyPlotWidget` (parametric) rely on QCustomPlot/QCPCurve. Combos are populated via `setSelectableChannels`; plots dock through the `Add Plot` / `Add XY Plot` actions.
- **Input manager**: keyboard (`keyPressEvent`/`keyReleaseEvent`), `QGamepad` (R2/L2, left stick, A/B/Start), and DirectInput (`pollDirectInput`) share the same `throttlePercent`, `brakePercent`, `steering`, and `gear`.
- **Scheduler**: `simulateLoop` runs in a `std::thread`, increments `simTime` by `dt`, and sleeps whenever simulated time runs ahead. A UI `QTimer` (~30 FPS) polls inputs and refreshes plots.

## Telemetry Channels
### VehicleTelemetry
`CarSpeed`, `CarOmega`, `CarLoadTorque`, `CarFrolling`, `CarFaero`, `CarFtractive`, `CarFbrake`, `BatteryVoltage`, `BatterySoC`, `MotorIq`, `MotorId`, `MotorMut.InductanceIq`, `MotorMut.InductanceId`, `MotorMechPower`, `MotorElec.Angle`, `MotorTorque`, `MotorSpeedRPM`, `InverterVd`, `InverterVq`.

### XdofVehicleTelemetry
- **Vehicle states**: `CarSpeedX`, `CarSpeedY`, `CarOmega`, `CarX`, `CarY`, `CarYaw`, `Gear`.
- **Engine**: `EngineTorque`, `EngineRPM`.
- **Brakes**: `Brake1_Force` … `Brake4_Force`.
- **Vertical loads**: `Wheel1_Load` … `Wheel4_Load`.
- **Wheel kinematics**: `Wheel*_SpeedX`, `Wheel*_SpeedY`.
- **Tyre forces**: `Wheel*_ForceX`, `Wheel*_ForceY`.
- **Slip**: `Wheel*_Beta`, `Wheel*_Lambda`.
- **Steering**: `Steering_w1`, `Steering_w4`.

## Execution Flow
1. User picks a simulation from the toolbar.
2. `simulateLoop` (thread) calls `simulateStep` on the active model via `std::visit`.
3. The simulation updates states and telemetry maps.
4. The UI `QTimer` invokes `updateData`, applying input deltas, clamping `throttlePercent`, `brakePercent`, `steering`, `gear`, then requests plots to pull fresh values.
5. Plot widgets query their bound `std::function` and refresh curves.

## Supported Controls
| Device | Action |
| --- | --- |
| Keyboard | Arrow ↑/↓ adjust throttle/brake (±5%), ←/→ steer (±0.05 per tick), `Shift` upshift, `Ctrl` downshift. |
| Qt Gamepad | `R2` throttle, `L2` brake, left stick X steer, `A` upshift, `B` downshift, `Start` neutral. |
| DirectInput wheel | Axis X steer, axes Y/Rz pedals (0–100%), right/left paddles shift, button 12 neutral. |

## Dependencies
- Microsoft Visual Studio 2022, MSVC v143 toolset.
- Qt 5.14.2 (Widgets, Gui, Core, Gamepad, Concurrent, etc.).
- QCustomPlot (bundled via generated headers).
- Eigen (included through `Dense`).
- SFML Audio (`sf::Music`).
- DirectX SDK / DirectInput 8.
- Windows (DirectInput and QtGamepad backend rely on Win32 APIs such as `HWND`, `Sleep`).

## Quick Build
1. Install Qt 5.14.2 with the MSVC 2017/2019 64-bit kit and configure `QTDIR` in Visual Studio.
2. Install SFML (Audio module) and adjust include/lib paths inside the project if required.
3. Clone the repo and open `TelemetryInterface.sln` in Visual Studio 2022.
4. Set `TelemetryInterface` as the startup project, target `x64` Debug or Release.
5. Ensure `TelemetryInterface/Resources/sc63_in_idle.wav` exists or update the hard-coded path in `XdofVehicle::update`.
6. Build and run. On first launch, choose a model from the toolbar and use `Add Plot` / `Add XY Plot` to start monitoring.

## Extending the System
- **Add channels**: push new entries into each model’s `telemetry` map; widgets pick them up automatically.
- **New models**: implement a class exposing `getChannels`, `getMap`, `simulateStep`, then register it in `MainWindow`’s `objectsMap` and variant type.
- **Custom inputs**: expand `pollDirectInput` or the `QGamepad` lambda set to map extra buttons (e.g., DRS, mode selectors).
- **Audio/resources**: replace the `.wav` asset or add more samples with crossfade logic.
- **Logging**: tap into telemetry maps to dump CSV or stream over sockets.

## Current Status & Roadmap
- Working features: EV and XDOF models, real-time plotting, keyboard/gamepad/wheel support, engine audio.
- Pending: `EMotorCharacterization` functionality, UI-driven vehicle/TC parameter editing, data export/logging, configurable resource paths, cross-platform CMake packaging.