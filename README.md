# Active Flight Control (Active Airbrake) 3D 6-DOF Simulation & PID Control

A complete MATLAB & Simulink aerospace simulation suite modeling **Active Airbrake Control** for a high-power sounding rocket with **actual COTS avionics electronics** in **full 3D 6-Degrees-of-Freedom (6-DOF)**.

---

## Key Features

1. **Full 3D 6-Degrees-of-Freedom (6-DOF) Dynamics**:
   - Rigid-body translational and rotational equations of motion integrated via unit quaternions (zero pitch gimbal lock).
   - Solid rocket motor mass depletion, dynamic center of gravity $x_{CG}(t)$, and varying moments of inertia $I_{xx}(t), I_{yy}(t), I_{zz}(t)$.
   - US Standard Atmosphere 1976 with altitude density stratification and speed of sound calculation.
   - 3D boundary layer shear wind field with atmospheric gust models.
   - Comprehensive aerodynamics: axial drag, normal force derivative $C_{N\alpha}$, side force derivative $C_{Y\beta}$, pitch damping $C_{mq}$, yaw damping $C_{nr}$, roll damping $C_{lp}$, and stability margin calculation.

2. **Comparative Active Airbrake Designs**:
   - **Design 1: Downward-Opening Hinged Flaps ($0^\circ \to 90^\circ$)**:
     - Variable angle $\theta \in [0, 90^\circ]$ against the rocket body.
     - Aerodynamic drag varies with $\sin^3(\theta)$.
     - Aerodynamic hinge moment $\tau_{hinge}(\theta)$ calculated at the flap center of pressure.
     - Actuated by **Savöx SC-1258TG** High-Torque Digital Titanium-Gear Coreless Servo with 2nd-order electromechanical dynamics, slew-rate limit ($750^\circ/\text{s}$), and load-stall modeling.
   - **Design 2: Radially Outward Sliding Flaps ($90^\circ$ Constant Orientation)**:
     - Variable stroke $s \in [0, 35\,\text{mm}]$ perpendicular to oncoming airflow.
     - Linear drag authority $\Delta C_D(s) \propto s$.
     - Normal aerodynamic pressure generates sliding friction on guide rails: $F_{friction} = \mu \cdot F_{normal} + F_{preload}$.
     - Actuated by **Actuonix L16-R** Micro Linear Actuator with force-speed derating and tests across variable sliding speeds ($16\,\text{mm/s}$, $32\,\text{mm/s}$, and $45\,\text{mm/s}$).

3. **Actual Existing Avionics & Electronics Hardware**:
   - **Flight Computer**: PJRC Teensy 4.1 (ARM Cortex-M7 @ 600 MHz) running a $50\,\text{Hz}$ discrete flight software loop with 16-bit PWM timer resolution and transport delay.
   - **Barometric Altimeter**: Bosch BMP388 ($50\,\text{Hz}$ rate, $0.12\,\text{m}$ RMS noise, barometric altitude inversion).
   - **6-DOF IMU**: Bosch BMI088 ($\pm 24\,\text{g}$ accelerometer, $\pm 2000^\circ/\text{s}$ gyroscope, noise spectral density, and in-run bias drift).
   - **Sensor Fusion**: 1D vertical Kalman Filter fusing BMP388 baro altitude and BMI088 axial acceleration.
   - **GNC Controller**: Ballistic energy-balance apogee predictor and Discrete PID Controller with anti-windup clamping and low-pass filtered derivative.
   - **Power Bus**: 2S 800mAh 45C LiPo battery with internal resistance ($45\,\text{m}\Omega$) modeling bus voltage sag under peak actuator current loads.

4. **Dual Execution Modes**:
   - **Simulink Model**: [`models/Rocket_Active_Airbrake_6DOF.slx`](file:///Users/wen-chungcheng/tac-fau-project/models/Rocket_Active_Airbrake_6DOF.slx) (clean, modular subsystems with scopes and outports).
   - **Standalone MATLAB 6-DOF Engine**: Fast batch simulator generating publication-quality figures.

---

## Directory Structure

```text
tac-fau-project/
├── models/
│   └── Rocket_Active_Airbrake_6DOF.slx    # Native Simulink 6-DOF Model
├── scripts/
│   ├── init_airbrake_params.m             # Master parameter initialization
│   ├── build_simulink_model.m             # Programmatic Simulink model generator
│   ├── set_airbrake_design.m              # Quick selector helper (MATLAB command or interactive)
│   ├── run_airbrake_comparison.m          # 5-Scenario comparative runner & plot generator
│   └── test_active_airbrake_suite.m       # Automated test & verification suite
├── functions/
│   ├── rocket_6dof_simulator.m            # Standalone 3D 6-DOF dynamic simulator
│   ├── airbrake_aerodynamics_6dof.m       # 3D aerodynamic forces, moments & loads
│   ├── airbrake_apogee_predictor.m        # Closed-form dynamic ballistic apogee predictor
│   ├── actuator_electronics_model.m       # Savox servo & Actuonix linear actuator dynamics
│   ├── flight_computer_gnc.m              # Teensy 4.1 FSW, Kalman Filter & Discrete PID
│   └── sensor_avionics_model.m            # BMP388 Barometer & BMI088 IMU sensor emulation
├── docs/
│   └── ACTIVE_AIRBRAKE_ENGINEERING_REPORT.md # Deep technical engineering report
└── results/                               # Saved publication comparison plots (PNG)
    ├── figure1_3d_trajectory.png
    ├── figure2_flight_dynamics.png
    ├── figure3_airbrake_kinematics.png
    ├── figure4_mechanical_loads.png
    ├── figure5_avionics_power.png
    └── figure6_pid_control.png
```

---

## Quick Start Guide

### 1. Run Complete 5-Scenario Comparison (MATLAB Command Window)
```matlab
addpath('scripts'); addpath('functions');
run_airbrake_comparison;
```
This executes:
1. Baseline Uncontrolled Rocket (overshoots to $3433\,\text{m}$)
2. Design 1 (Downward Hinged Flaps with Savox servo)
3. Design 2 Nominal (Radially Outward Sliding Flaps @ $32\,\text{mm/s}$)
4. Design 2 Fast (Sliding Flaps @ $45\,\text{mm/s}$)
5. Design 2 Slow (Sliding Flaps @ $16\,\text{mm/s}$)

And generates 6 publication-quality figures in [`results/`](file:///Users/wen-chungcheng/tac-fau-project/results/).

### 2. Run the Simulink Model
```matlab
open_system('models/Rocket_Active_Airbrake_6DOF.slx');
sim_out = sim('Rocket_Active_Airbrake_6DOF');
```
- **Switching Airbrake Designs**:
  - **Method A (Interactive Dialog)**: Double-click the prominent light-blue **`AIRBRAKE SELECTOR`** block on the top-left of the Simulink canvas. A dedicated Block Parameters modal pops up with a clean dropdown menu:
    - *Design 1: Downward Hinged Flaps (0 - 90 deg, Savox SC-1258TG)*
    - *Design 2: Radially Outward Sliding Flaps (90 deg, Actuonix L16-R)*
  - **Method B (MATLAB Command Shortcut)**: Run either in the Command Window:
    ```matlab
    set_airbrake_design(1);  % Switch to Design 1 (Hinged)
    set_airbrake_design(2);  % Switch to Design 2 (Sliding)
    set_airbrake_design;     % Query current active design
    ```

### 3. Run Automated Test Suite
```matlab
addpath('scripts'); addpath('functions');
test_active_airbrake_suite;
```

---

## Comparative Performance Summary

| Configuration | Apogee Achieved | Error from Target ($3000\,\text{m}$) | Peak Mach | Peak Mechanical Load | Peak Current | Battery Energy |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **Baseline (No Brakes)** | **3433.0 m** | **+433.0 m** (+14.4%) | 1.02 | N/A | 0.27 A | 1.99 mAh |
| **Design 1 (Hinged 0-90°)** | **2977.6 m** | **-22.4 m** (-0.7%) | 1.02 | **3.84 N·m** (Hinge torque) | **3.78 A** | 7.48 mAh |
| **Design 2 (Sliding 32 mm/s)**| **2983.5 m** | **-16.5 m** (-0.5%) | 1.02 | **32.7 N** (Rail friction) | **0.81 A** | 3.15 mAh |
| **Design 2 (Sliding 45 mm/s)**| **2981.2 m** | **-18.8 m** (-0.6%) | 1.02 | **34.3 N** (Rail friction) | **0.83 A** | 3.17 mAh |
| **Design 2 (Sliding 16 mm/s)**| **2986.6 m** | **-13.4 m** (-0.4%) | 1.02 | **28.6 N** (Rail friction) | **0.81 A** | 3.25 mAh |

### Simulink Model Verification (`Rocket_Active_Airbrake_6DOF.slx`):
- **Design 1**: Apogee = **$3038.2\,\text{m}$** (Error: $+38.2\,\text{m}$ / $+1.27\%$).
- **Design 2**: Apogee = **$2997.8\,\text{m}$** (Error: **$-2.15\,\text{m}$** / **$-0.07\%$**).
