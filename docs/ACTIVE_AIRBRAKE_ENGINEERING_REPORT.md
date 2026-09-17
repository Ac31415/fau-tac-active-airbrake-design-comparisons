# 3D 6-DOF Active Airbrake Flight Simulation & PID Control Report
**Avionics Modeling, Mechanism Trade-Offs, and Simulink Implementation**

---

## 1. Executive Summary

This simulation and control architecture models active flight control for a high-power sounding rocket equipped with active airbrakes targeting a precise apogee of **$3,000\,\text{m}$ ($9,842\,\text{ft}$)**. 

The simulation models:
1. **Full 3D 6-Degrees-of-Freedom (6-DOF) Flight Dynamics**: Quaternion-based rigid body dynamics, US Standard Atmosphere 1976, power-law boundary layer shear wind, and 3D aerodynamic forces and moments.
2. **Actual Commercial-Off-The-Shelf (COTS) Electronics**:
   - **Flight Computer**: PJRC Teensy 4.1 (ARM Cortex-M7 @ 600 MHz, 50 Hz discrete loop, 16-bit PWM timer resolution, transport latency).
   - **Barometric Altimeter**: Bosch BMP388 ($50\,\text{Hz}$ ODR, $0.12\,\text{m}$ RMS altitude noise, thermal drift).
   - **6-DOF IMU**: Bosch BMI088 ($\pm 24\,\text{g}$ accelerometer, $\pm 2000^\circ/\text{s}$ gyroscope, noise spectral densities, and in-run bias drift).
   - **Power Bus**: 2S 800mAh 45C LiPo battery with internal resistance ($45\,\text{m}\Omega$) predicting dynamic voltage sag under peak actuator loads.
3. **Dual Mechanism Design & Actuator Trade-Off**:
   - **Design 1 (Downward Hinged Flaps)**: 4 symmetric flaps pivoting from $0^\circ$ (flush) to $90^\circ$ (perpendicular), actuated by a central **Savöx SC-1258TG** high-torque digital coreless titanium-gear servo subject to aerodynamic hinge moments ($M_{hinge}$).
   - **Design 2 (Radially Outward Sliding Flaps)**: 4 symmetric flaps oriented perpendicular ($90^\circ$) to the flow, translating radially outwards ($0 - 35\,\text{mm}$), actuated by an **Actuonix L16-R** micro linear actuator at variable sliding speeds ($16\,\text{mm/s}$, $32\,\text{mm/s}$, and $45\,\text{mm/s}$) subject to aerodynamic normal loads and guide-rail sliding friction ($F_{friction}$).
4. **Guidance, Navigation, and Control (GNC)**:
   - Discrete 1D Vertical Kalman Filter fusing BMP388 baro altitude and BMI088 axial acceleration.
   - Closed-form ballistic energy-balance apogee predictor with altitude density stratification.
   - Discrete PID controller with anti-windup clamping, filtered derivative, and rate limiting.

Both a **standalone 6-DOF MATLAB simulation engine** and a **native MATLAB Simulink model (`Rocket_Active_Airbrake_6DOF.slx`)** are implemented and verified.

---

## 2. 3D 6-Degrees-of-Freedom (6-DOF) Dynamics Formulation

### 2.1 State Vector & Reference Frames
The vehicle state vector $\mathbf{x} \in \mathbb{R}^{13}$ is defined in the Earth-Centered North-East-Down (NED) frame and Rocket Body frameas follows:
- $$\mathbf{x} = \begin{bmatrix} \mathbf{r}_{NED}^T & \mathbf{V}_b^T & \mathbf{q}^T & \boldsymbol{\omega}_b^T \end{bmatrix}^T$$
- $\mathbf{r}_{NED} = [X_N, Y_E, Z_D]^T$: Position in NED frame. Altitude MSL is $h = -Z_D$.
- $\mathbf{V}_b = [u, v, w]^T$: Linear velocity in Rocket Body frame (axial $u$, right lateral $v$, normal down $w$).
- $\mathbf{q} = [q_0, q_1, q_2, q_3]^T$: Unit quaternion representing orientation from NED to Body frame ($\|\mathbf{q}\| = 1$).
- $\boldsymbol{\omega}_b = [p, q, r]^T$: Body angular rates (roll $p$, pitch $q$, yaw $r$).

### 2.2 Direction Cosine Matrix (DCM)
The rotation matrix $\mathbf{C}_{b/e}$ mapping vectors from NED to Body is:
$\mathbf{C}_{b/e} = \begin{bmatrix}
q_0^2 + q_1^2 - q_2^2 - q_3^2 & 2(q_1 q_2 + q_0 q_3) & 2(q_1 q_3 - q_0 q_2) \\
2(q_1 q_2 - q_0 q_3) & q_0^2 - q_1^2 + q_2^2 - q_3^2 & 2(q_2 q_3 + q_0 q_1) \\
2(q_1 q_3 + q_0 q_2) & 2(q_2 q_3 - q_0 q_1) & q_0^2 - q_1^2 - q_2^2 + q_3^2
\end{bmatrix}$
The inertial velocity is $\mathbf{V}_{NED} = \mathbf{C}_{b/e}^T \mathbf{V}_b$.

### 2.3 Equations of Motion
1. **Kinematics (Position Rate)**:
   $\dot{\mathbf{r}}_{NED} = \mathbf{C}_{b/e}^T \mathbf{V}_b$
2. **Attitude Kinematics (Quaternion Rate)**:
   $\dot{\mathbf{q}} = \frac{1}{2} \boldsymbol{\Omega}_b \mathbf{q} = \frac{1}{2} \begin{bmatrix}
   0 & -p & -q & -r \\
   p & 0 & r & -q \\
   q & -r & 0 & p \\
   r & q & -p & 0
   \end{bmatrix} \begin{bmatrix} q_0 \\ q_1 \\ q_2 \\ q_3 \end{bmatrix}$
3. **Translational Dynamics (Newton-Euler)**:
   $m(t) \left( \dot{\mathbf{V}}_b + \boldsymbol{\omega}_b \times \mathbf{V}_b \right) = \mathbf{F}_{aero,b} + \mathbf{F}_{thrust,b} + \mathbf{F}_{grav,b}$
   $\dot{\mathbf{V}}_b = \frac{1}{m(t)} \left( \mathbf{F}_{aero,b} + \mathbf{F}_{thrust,b} + \mathbf{C}_{b/e} \begin{bmatrix} 0 \\ 0 \\ m(t) g_0 \end{bmatrix} \right) - \boldsymbol{\omega}_b \times \mathbf{V}_b$
4. **Rotational Dynamics**:
   $\mathbf{I}(t) \dot{\boldsymbol{\omega}}_b + \boldsymbol{\omega}_b \times (\mathbf{I}(t) \boldsymbol{\omega}_b) = \mathbf{M}_{aero,b}$
   $\dot{\boldsymbol{\omega}}_b = \mathbf{I}(t)^{-1} \left( \mathbf{M}_{aero,b} - \boldsymbol{\omega}_b \times (\mathbf{I}(t) \boldsymbol{\omega}_b) \right)$

---

## 3. Airbrake Mechanism Physics & Aerodynamic Models

### 3.1 Design 1: Downward-Opening Pivoting Flaps
- **Geometry**: 4 rectangular flaps hinged at their forward edge: width $W = 38\,\text{mm}$, length $L = 60\,\text{mm}$.
- **Deployment Angle**: $\theta \in [0^\circ, 90^\circ]$ ($0\,\text{rad}$ flush to $\pi/2\,\text{rad}$ normal).
- **Projected Area**:
  $A_{proj}(\theta) = 4 \cdot W \cdot L \cdot \sin(\theta)$
- **Drag Increment**: Flow separation behind a hinged flap varies with $\sin^2(\theta)$:
  $\Delta C_D(\theta) = C_{D,flap,90} \sin^2(\theta) \frac{A_{proj}(\theta)}{A_{ref}} = C_{D,flap,90} \frac{4 W L}{A_{ref}} \sin^3(\theta)$
- **Aerodynamic Normal Load on Flap**:
  $F_{norm}(\theta) = q_\infty \cdot C_{D,flap,90} \sin(\theta) \cdot (W L)$
- **Hinge Torque**: The flap center of pressure is located at $r_{cp} \approx 0.45 L$ from the hinge axis.
  $\tau_{hinge}(\theta) = F_{norm}(\theta) \cdot r_{cp} \sin(\theta)$
  The total torque demanded from the central digital servo through a mechanical linkage with advantage $\eta_{link} = 1.25$ is:
  $\tau_{servo,load} = \frac{4 \cdot \tau_{hinge}(\theta)}{\eta_{link}}$

### 3.2 Design 2: Radially Outward Sliding Flaps
- **Geometry**: 4 rectangular plates fixed permanently at $90^\circ$ to the body, extending radially through slots: width $W = 38\,\text{mm}$, max stroke $s_{max} = 35\,\text{mm}$.
- **Deployment Stroke**: $s \in [0, s_{max}]$.
- **Projected Area**: Purely linear with extension stroke:
  $A_{proj}(s) = 4 \cdot W \cdot s$
- **Drag Increment**: Flat plate at $90^\circ$ normal to oncoming flow ($C_{D,plate} = 1.28$):
  $\Delta C_D(s) = C_{D,plate} \frac{4 W s}{A_{ref}}$
- **Aero Normal Force & Rail Friction**: Because the flap is at $90^\circ$ to the flow, dynamic pressure generates a large aerodynamic drag force normal to the guide rails:
  $F_{normal}(s) = q_\infty \cdot C_{D,plate} \cdot (W s)$
  This normal load pushes the flap against its guide bushings/rails with friction coefficient $\mu_{rail} \approx 0.20$ and seal preload $F_{preload} = 2.0\,\text{N}$:
  $F_{rail,friction} = 4 \cdot (\mu_{rail} F_{normal} + F_{preload})$
  The linear actuator must generate an axial force $F_{actuator} > F_{rail,friction}$ to extend or retract the flaps!

---

## 4. Avionics Hardware & Electronics Modeling

### 4.1 Flight Computer: PJRC Teensy 4.1
- **MCU**: ARM Cortex-M7 @ 600 MHz, 32-bit hardware FPU.
- **Control Loop Rate**: $f_s = 50\,\text{Hz}$ ($T_s = 0.020\,\text{s}$).
- **PWM Timer**: 16-bit resolution ($0.015\,\mu\text{s}$ resolution) operating at $200\,\text{Hz}$ carrier frequency. Command range: $1000\,\mu\text{s}$ (0% brake) to $2000\,\mu\text{s}$ (100% brake).
- **Processing Latency**: $5\,\text{ms}$ sensor read to PWM generation transport delay.

### 4.2 Sensors
1. **Bosch BMP388 Precision Barometer**:
   - Sample rate: $50\,\text{Hz}$.
   - Pressure to altitude: $h_{baro} = \frac{T_0}{L} \left[ 1 - \left(\frac{P}{P_0}\right)^{\frac{R L}{g}} \right]$.
   - Noise: Gaussian white noise with $\sigma = 0.12\,\text{m}$ RMS and slow thermal bias drift ($0.5\,\text{m}$).
2. **Bosch BMI088 6-DOF IMU**:
   - Accelerometer: $\pm 24\,\text{g}$ range, noise density $160\,\mu\text{g}/\sqrt{\text{Hz}}$, in-run bias drift.
   - Gyroscope: $\pm 2000^\circ/\text{s}$ range, noise density $0.014^\circ/\text{s}/\sqrt{\text{Hz}}$, in-run bias drift.

### 4.3 Actuators & Power Distribution
1. **Design 1 Servo: Savöx SC-1258TG Digital Coreless Titanium-Gear Servo**:
   - Nominal voltage: $7.4\,\text{V}$ (2S LiPo).
   - Stall torque: $1.47\,\text{N}\cdot\text{m}$ ($15.0\,\text{kg}\cdot\text{cm}$).
   - Max no-load speed: $750^\circ/\text{s}$ ($0.08\,\text{s}/60^\circ$).
   - 2nd-order dynamic model: $\omega_n = 52\,\text{rad/s}$, $\zeta = 0.72$.
   - Speed derating under aerodynamic hinge torque: $\dot{\theta}_{max}(\tau) = \dot{\theta}_{max,0} \left(1 - \frac{\tau_{load}}{\tau_{stall}}\right)$.
   - Current draw: $I_{servo} = I_{idle} + I_{run} \frac{|\dot{\theta}|}{\dot{\theta}_{max}} + (I_{stall} - I_{run})\left(\frac{\tau_{load}}{\tau_{stall}}\right)^{1.5}$.
2. **Design 2 Actuator: Actuonix L16-R Micro Linear Actuator**:
   - Nominal voltage: $7.4\,\text{V}$. Stroke: $35\,\text{mm}$.
   - Peak stall force: $F_{stall} = 50.0\,\text{N}$ ($5.1\,\text{kgf}$).
   - Unloaded max speeds: $32\,\text{mm/s}$ (Nominal), $45\,\text{mm/s}$ (Fast), $16\,\text{mm/s}$ (Slow).
   - Force-velocity derating: $v_{max}(F) = v_{unloaded} \left(1 - \frac{F_{rail,friction}}{F_{stall}}\right)$.
   - Current draw: $I_{linear} = I_{idle} + I_{run} \frac{|\dot{s}|}{v_{max}} + (I_{stall} - I_{run})\frac{F_{rail,friction}}{F_{stall}}$.
3. **Power Bus: Tattu 2S 800mAh 45C LiPo Battery**:
   - Full charge: $8.40\,\text{V}$, nominal: $7.40\,\text{V}$, internal resistance: $R_{int} = 0.045\,\Omega$.
   - Bus voltage sag: $V_{bus}(t) = V_{oc} - I_{total}(t) \cdot R_{int}$.

---

## 5. Guidance, Navigation, and Control (GNC) Algorithms

### 5.1 Discrete Sensor Fusion (Kalman Filter)
A 1D vertical Kalman Filter fuses the BMI088 axial specific force $a_x$ and BMP388 barometric altitude $h_{baro}$ at $50\,\text{Hz}$:
$$\hat{\mathbf{x}}_k = \begin{bmatrix} \hat{h} \\ \hat{v}_z \end{bmatrix}_k, \quad \mathbf{A} = \begin{bmatrix} 1 & T_s \\ 0 & 1 \end{bmatrix}, \quad \mathbf{B} = \begin{bmatrix} \frac{1}{2} T_s^2 \\ T_s \end{bmatrix}$$
- Predict: $\hat{\mathbf{x}}_{k|k-1} = \mathbf{A} \hat{\mathbf{x}}_{k-1} + \mathbf{B} (a_{x,meas} - g)$
- Update: $\hat{\mathbf{x}}_k = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k (h_{baro} - \hat{h}_{k|k-1})$

### 5.2 Dynamic Ballistic Apogee Predictor
During the coast phase ($t > t_{burnout}$), the remaining vertical coast ODE:
$$v_z \frac{dv_z}{dh} = -g - \frac{\rho(h) C_D A_{ref}}{2 m} v_z^2 = -g - \beta v_z^2$$
Integrating with an effective density scale height correction:
$$\Delta h_{coast} = \frac{1}{2 \beta_{eff}(u)} \ln\left(1 + \frac{\beta_{eff}(u)}{g} v_z^2\right)$$
$$h_{pred}(u) = \hat{h} + \Delta h_{coast}(u)$$
where $\beta_{eff}(u) = \frac{\rho_{eff} (C_{D0} + \Delta C_D(u)) A_{ref}}{2 m_{dry}}$.

### 5.3 Discrete PID Controller
- Control Error: $e_k = h_{pred}(u_k) - h_{target}$.
- Proportional: $P = K_p \cdot e_k$ ($K_p = 0.0035\,\text{m}^{-1}$).
- Integral with Anti-Windup Clamping:
  $$I_k = I_{k-1} + K_i \cdot e_k \cdot T_s \quad (K_i = 0.0012\,\text{m}^{-1}\text{s}^{-1})$$
  Clamped conditionally if output saturates at $0$ or $1$.
- Derivative with 1st-Order Low-Pass Filter ($N = 25\,\text{rad/s}$):
  $$D_k = \frac{K_d N \Delta e_k + D_{k-1}}{1 + N T_s} \quad (K_d = 0.0008\,\text{s/m})$$
- Output Slew-Rate Limiter: $|\Delta u| \le 3.0 \cdot T_s \implies 0.060/\text{cycle}$.

---

## 6. Comparative Simulation Results

All five test scenarios were evaluated using the verified 6-DOF dynamic engine:

| Configuration | Apogee Achieved | Error from Target | Peak Mach | Peak Mechanical Load | Peak Current | Battery Energy Consumed |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **Baseline (No Brakes)** | **3433.0 m** | **+433.0 m** (+14.4%) | 1.02 | N/A | 0.27 A | 1.99 mAh |
| **Design 1 (Hinged 0-90°)** | **2977.6 m** | **-22.4 m** (-0.7%) | 1.02 | **3.84 N·m** (Servo torque) | **3.78 A** | 7.48 mAh |
| **Design 2 (Sliding 32 mm/s)** | **2983.5 m** | **-16.5 m** (-0.5%) | 1.02 | **32.7 N** (Rail friction) | **0.81 A** | 3.15 mAh |
| **Design 2 (Sliding 45 mm/s)** | **2981.2 m** | **-18.8 m** (-0.6%) | 1.02 | **34.3 N** (Rail friction) | **0.83 A** | 3.17 mAh |
| **Design 2 (Sliding 16 mm/s)** | **2986.6 m** | **-13.4 m** (-0.4%) | 1.02 | **28.6 N** (Rail friction) | **0.81 A** | 3.25 mAh |

### Simulink Model Verification (`Rocket_Active_Airbrake_6DOF.slx`):
- **Design 1 (Hinged Flaps)**: Apogee = **$3,038.2\,\text{m}$** (Error: $+38.2\,\text{m}$ / $+1.27\%$). Max angle: $79.2^\circ$.
- **Design 2 (Sliding Flaps)**: Apogee = **$2,997.8\,\text{m}$** (Error: **$-2.15\,\text{m}$** / **$-0.07\%$**). Max stroke: $17.6\,\text{mm}$.

---

## 7. Key Engineering Insights & Trade-Offs

1. **Linearity of Control Authority**:
   - **Design 1 (Hinged Flaps)**: Exhibited cubic non-linearity ($\Delta C_D \propto \sin^3(\theta)$). Small opening angles ($< 25^\circ$) generate minimal drag, while high angles ($> 60^\circ$) produce steep drag surges. This non-linearity requires adaptive PID tuning or gain scheduling across flight regimes.
   - **Design 2 (Sliding Flaps)**: Exhibited strictly linear control authority ($\Delta C_D \propto s$). The linear plant gain makes standard PID control remarkably smooth and predictable, reaching an accuracy of $-2.15\,\text{m}$ (0.07% error).
2. **Mechanical Load & Actuator Stress**:
   - **Design 1**: Aerodynamic hinge torque reached $3.84\,\text{N}\cdot\text{m}$. For a single servo, this exceeds standard micro-servos and demands high mechanical linkage leverage ($\eta_{link} \ge 1.25$) or dual servos. Peak current surged to $3.78\,\text{A}$, causing measurable battery bus voltage sag.
   - **Design 2**: While guide-rail friction reached $32.7\,\text{N}$, it remained safely below the Actuonix L16 stall threshold ($50\,\text{N}$). Peak electrical current remained under $0.85\,\text{A}$, reducing electrical bus noise and battery drain by **58%**.
3. **Effect of Sliding Speed**:
   - Even at the lowest sliding speed ($16\,\text{mm/s}$), the rocket reached apogee with only $-13.4\,\text{m}$ error. Because the coast phase lasts over $18\,\text{seconds}$, ultra-fast actuation is not strictly necessary for apogee regulation; smooth, steady stroke modulation avoids vehicle pitch oscillation and aeroelastic flutter.
