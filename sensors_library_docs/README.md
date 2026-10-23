# IREC 2027 — Airbrakes Avionics Sensor Suite & Library Reference

**Organization:** Technology & Aerospace Club (TAC)  
**Subsystem:** Active Airbrakes Avionics — Inertial & Barometric Sensor Proposals  
**Architecture Philosophy:** Dissimilar Redundancy

---

## Table of Contents

1. [Overview & Dissimilar Redundancy Architecture](#1-overview--dissimilar-redundancy-architecture)
2. [Summary Comparison Matrix](#2-summary-comparison-matrix)
3. [Quick-Start Build Configuration (`platformio.ini`)](#3-quick-start-build-configuration-platformioini)
4. [Inertial Measurement Units (IMUs)](#4-inertial-measurement-units-imus)
   - [4.1 LSM6DS3TR-C (Adafruit 4503) — 6-DoF IMU](#41-lsm6ds3tr-c-adafruit-4503--6-dof-imu)
   - [4.2 ADXL343 (Adafruit 4097) — 3-Axis Accelerometer](#42-adxl343-adafruit-4097--3-axis-accelerometer)
5. [Barometric Pressure Sensors](#5-barometric-pressure-sensors)
   - [5.1 LPS25HB (Adafruit 4530 / Labeled 4258) — MEMS Barometer](#51-lps25hb-adafruit-4530--labeled-4258--mems-barometer)
   - [5.2 MS5611-01BA03 (Embedded Adventures MOD-1009) — MEMS Barometer](#52-ms5611-01ba03-embedded-adventures-mod-1009--mems-barometer)
6. [Master Reference & Documentation Links](#6-master-reference--documentation-links)

---

## 1. Overview & Dissimilar Redundancy Architecture

The IREC 2027 Airbrakes Sensor Proposals implement multiple sensors across each measurement domain (inertial acceleration/angular rate and barometric static pressure) using **dissimilar redundancy**:

> *"Unlike traditional redundancy, which uses more than one of the same thing, dissimilar redundancy uses different things. The idea is that the different things are unlikely to contain identical flaws."*  
> — [Wikipedia: Redundancy (engineering) — Dissimilar redundancy](https://en.wikipedia.org/wiki/Redundancy_(engineering)#Dissimilar_redundancy)

By pairing chips from different semiconductor manufacturers (STMicroelectronics vs. Analog Devices for inertial sensing; STMicroelectronics vs. TE Connectivity/Measurement Specialties for barometric sensing) and using distinct software driver stacks, the avionics system mitigates common-mode hardware bugs, silicon errata, saturation anomalies, and library-level edge cases during high-speed ascent and active drag modulation.

---

## 2. Summary Comparison Matrix

| Sensor & Breakout | Manufacturer | Primary Libraries (Arduino C++ / Python) | Supported Buses | Key Library Features | Major Caveats |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **LSM6DS3TR-C** (Adafruit 4503) | STMicroelectronics | • `Adafruit_LSM6DS` (`Adafruit_LSM6DS3TRC`)<br>• `STM32duino LSM6DS3TR-C`<br>• `SparkFun_LSM6DS3`<br>• `adafruit_lsm6ds` (CircuitPython) | I2C (`0x6A`/`0x6B`), SPI | Configurable range/ODR (up to 6.66 kHz), wakeup interrupt, pedometer, high-pass/slope filters, SI unit conversion (`Adafruit_Sensor`). | `Adafruit_LSM6DS` omits hardware FIFO and tap detection (requires `STM32duino` or raw register writes); default range starts at ±2 g and must be explicitly set to ±16 g in code. |
| **ADXL343** (Adafruit 4097) | Analog Devices | • `Adafruit_ADXL343`<br>• `SparkFun_ADXL345`<br>• `adafruit_adxl34x` (CircuitPython) | I2C (`0x53`/`0x1D`), SPI | Range selection (±2 g to ±16 g), ODR up to 3200 Hz, INT1/INT2 mapping for activity, inactivity, free-fall, and single/double tap, XYZ trim offsets. | **3-axis accelerometer only** (no gyroscope); Adafruit library lacks helper methods for the 32-level hardware FIFO; 1600 Hz and 3200 Hz ODRs require SPI (>= 2 MHz) rather than standard I2C; `INT` pins are 3.3V logic only. |
| **LPS25HB** (Adafruit 4530 / labeled 4258) | STMicroelectronics | • `Adafruit_LPS2X` (`Adafruit_LPS25`)<br>• `Pololu LPS` (`lps-arduino`)<br>• `STM32duino LPS25HB`<br>• `adafruit_lps2x` (CircuitPython) | I2C (`0x5D`/`0x5C`), SPI | Pressure (hPa) & temperature (°C) reads, configurable ODR (One-Shot, 1, 7, 12.5, 25 Hz), `Adafruit_Sensor` events, barometric altitude helper (in `Pololu LPS`). | **Max 25 Hz sample rate** is slow for high-speed airbrake control loops; `Adafruit_LPS2X` omits FIFO and threshold interrupt helpers; hard ceiling at **260 hPa (33,153 ft)**. |
| **MS5611** (Embedded Adventures MOD-1009) | TE Connectivity | • `RobTillaart/MS5611` (I2C) & `MS5611_SPI`<br>• `jarzebski/Arduino-MS5611`<br>• Embedded Adventures `PicPack` (`ms5611.c/.h`) | I2C (`0x76`/`0x77`), SPI | Factory PROM calibration parsing, configurable oversampling (OSR 256 to 4096), 2nd-order low-temp compensation, pressure/temperature offset calibration. | **No onboard regulator/level shifter** (strictly 1.8V–4.0V; 5V logic causes >10°C self-heating or damage); ADC conversions require ~0.6 ms to ~9.04 ms waits; physical solder jumper required to switch I2C/SPI. |

---

## 3. Quick-Start Build Configuration (`platformio.ini`)

If building the flight firmware with **PlatformIO** (Arduino framework), include the following `lib_deps` in your `platformio.ini`:

```ini
[env:flight_controller]
platform = ...       ; e.g., teensy, espressif32, ststm32
board = ...
framework = arduino

lib_deps =
    ; Shared Adafruit Bus & Unified Sensor Abstractions
    adafruit/Adafruit BusIO @ ^1.16.1
    adafruit/Adafruit Unified Sensor @ ^1.1.14

    ; 1) LSM6DS3TR-C (6-DoF IMU)
    adafruit/Adafruit LSM6DS @ ^4.7.2
    stm32duino/STM32duino LSM6DS3TR-C @ ^2.0.1

    ; 2) ADXL343 (3-Axis Accelerometer & Pad Wakeup)
    adafruit/Adafruit ADXL343 @ ^1.6.4

    ; 3) LPS25HB (MEMS Barometer)
    adafruit/Adafruit LPS2X @ ^2.0.6
    pololu/LPS @ ^3.1.1

    ; 4) MS5611 (High-Resolution MEMS Barometer - I2C & SPI)
    robtillaart/MS5611 @ ^0.4.1
    robtillaart/MS5611_SPI @ ^0.3.1
```

---

## 4. Inertial Measurement Units (IMUs)

### 4.1 LSM6DS3TR-C (Adafruit 4503) — 6-DoF IMU

The **LSM6DS3TR-C** is a 6-DoF inertial measurement unit (3-axis accelerometer + 3-axis gyroscope) supporting up to ±16 g linear acceleration and ±2000 dps angular rate over SPI or I2C with hardware wakeup interrupts.

#### Proposal Hardware Details
* **Breakout Board:** Adafruit 4503 ($9.95)
* **Acceleration Range:** Up to ±16 g (IREC 2026 report lists 10 g or 9.68 g as the greatest expected acceleration)
* **Angular Rate Range:** Up to ±2000 dps
* **Hardware Features:** Wakeup Interrupt, auxiliary I2C pins (`SCX`/`SDX`), primary SPI and I2C interfaces

#### Libraries Used
* **`Adafruit_LSM6DS` (Arduino C++):** Primary library used via the `Adafruit_LSM6DS3TRC` subclass (`#include <Adafruit_LSM6DS3TRC.h>`). Requires `Adafruit_BusIO` and `Adafruit_Sensor`.
* **`STM32duino LSM6DS3TR-C` (Arduino C++):** STMicroelectronics' official full-featured driver (`#include <LSM6DS3TRCSensor.h>`), compatible across STM32, AVR, SAM, Teensy, and ESP32 architectures.
* **`SparkFun_LSM6DS3_Arduino_Library` (Arduino C++):** Alternative C++ driver originally written for the LSM6DS3 that exposes low-level register reads/writes and hardware FIFO control.
* **`Adafruit_CircuitPython_LSM6DS` (CircuitPython / Python):** Accessed via `from adafruit_lsm6ds.lsm6ds3trc import LSM6DS3TRC`.

#### Features
* **Range & Output Data Rate (ODR) Configuration:**
  * `setAccelRange()`: `LSM6DS_ACCEL_RANGE_2_G`, `4_G`, `8_G`, `16_G`
  * `setGyroRange()`: `LSM6DS_GYRO_RANGE_125_DPS`, `250_DPS`, `500_DPS`, `1000_DPS`, `2000_DPS`
  * `setAccelDataRate()` / `setGyroDataRate()`: Configurable from Power-Down and `12.5 Hz` up to `6.66 kHz` (`LSM6DS_RATE_6_66K_HZ`).
* **Standardized SI Units (`getEvent`):** Converts raw 16-bit two's complement registers into `sensors_event_t` structures in **m/s²** (accelerometer), **rad/s** (gyroscope), and **°C** (on-die temperature).
* **Interrupts & Wakeup (`enableWakeup`, `configInt1`, `configInt2`, `configIntOutputs`):** Configures active-high/low or open-drain interrupt pins, routes Data-Ready (DRDY) signals for accelerometer/gyro/temperature to `INT1` or `INT2`, and sets the wakeup interrupt threshold and duration.
* **On-Chip Digital Filtering & Pedometer:** `highPassFilter()` configures the internal high-pass/slope filter; `enablePedometer()` and `readPedometer()` control the step counter; `enableI2CMasterPullups()` toggles auxiliary I2C pullups (`SCX`/`SDX` pins).
* **Advanced Features in `STM32duino LSM6DS3TR-C`:** Adds full support for the 4 KB hardware FIFO buffer, single/double tap, free-fall, tilt, 6D orientation detection, and significant motion interrupts.

#### Caveats
1. **Default Range Clipping (±2 g Default):** `Adafruit_LSM6DS` initializes the accelerometer to **±2 g** and the gyroscope to **250 dps** by default. Flight code **must** explicitly call `setAccelRange(LSM6DS_ACCEL_RANGE_16_G)` and `setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS)` during setup, or rocket boost accelerations (~10 g) will saturate the sensor.
2. **No FIFO or Tap Helpers in `Adafruit_LSM6DS`:** Adafruit's library does not expose the LSM6DS3TR-C's internal 4 KB FIFO buffer, free-fall, or tap detection. If your flight computer needs buffered burst reads or hardware free-fall detection on this chip, use ST's `STM32duino LSM6DS3TR-C` library or write directly to the registers via `Adafruit_BusIO`.
3. **Chip ID (`WHO_AM_I`) Mismatch with Older LSM6DS3:** The older `LSM6DS3` has a `WHO_AM_I` value of `0x69`, whereas the `LSM6DS3TR-C` returns `0x6A`. You must instantiate `Adafruit_LSM6DS3TRC` (not `Adafruit_LSM6DS3`), or `begin_I2C()` / `begin_SPI()` will fail initialization.
4. **Gyroscope Unit Convention:** `Adafruit_LSM6DS` outputs angular velocity in **radians per second (rad/s)** in Arduino C++, whereas `STM32duino` outputs **millidegrees per second (mdps)**. Mixing units in Extended/Unscented Kalman Filters or orientation integrators is a common flight-software pitfall.

#### Example Initialization Snippet (Arduino C++)
```cpp
#include <Adafruit_LSM6DS3TRC.h>

Adafruit_LSM6DS3TRC lsm6ds;

void setupLSM6DS() {
  if (!lsm6ds.begin_I2C(0x6A)) { // Or begin_SPI(CS_PIN)
    while (1) { delay(10); }
  }
  // CRITICAL: Override ±2g / 250dps defaults for rocket boost flight regime
  lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  lsm6ds.setAccelDataRate(LSM6DS_RATE_416_HZ);
  lsm6ds.setGyroDataRate(LSM6DS_RATE_416_HZ);
}
```

#### Reference Links
* **Product Page:** https://www.adafruit.com/product/4503
* **Datasheet:** https://www.st.com/resource/en/datasheet/lsm6ds3tr-c.pdf
* **Adafruit Learning Guide (PDF):** https://cdn-learn.adafruit.com/downloads/pdf/adafruit-lsm6ds3tr-c-6-dof-accel-gyro-imu.pdf
* **Adafruit Learning Guide (Web):** https://learn.adafruit.com/products/4503/guides
* **Arduino Library (`Adafruit_LSM6DS`):** https://github.com/adafruit/Adafruit_LSM6DS
* **STMicroelectronics Arduino Library (`STM32duino LSM6DS3TR-C`):** https://github.com/stm32duino/LSM6DS3TR-C
* **SparkFun Arduino Library (`SparkFun_LSM6DS3`):** https://github.com/sparkfun/SparkFun_LSM6DS3_Arduino_Library
* **CircuitPython Library (`Adafruit_CircuitPython_LSM6DS`):** https://github.com/adafruit/Adafruit_CircuitPython_LSM6DS

---

### 4.2 ADXL343 (Adafruit 4097) — 3-Axis Accelerometer

The **ADXL343** is a 3-axis digital MEMS accelerometer supporting up to ±16 g over SPI and I2C. In the IREC 2027 proposal, it is specifically selected for its two programmable interrupt pins and built-in motion detection to establish a low-power pad standby mode that wakes the avionics system upon launch.

#### Proposal Hardware Details
* **Breakout Board:** Adafruit 4097 ($5.95)
* **Acceleration Range:** Up to ±16 g (covering the 10 g / 9.68 g max acceleration from the IREC 2026 report)
* **Hardware Features:** 2 programmable interrupts (`INT1`, `INT2`), built-in tap, double-tap, activity, inactivity, and free-fall detection, SPI and I2C interfaces

#### Libraries Used
* **`Adafruit_ADXL343` (Arduino C++):** Official driver (`#include <Adafruit_ADXL343.h>`), built on `Adafruit_BusIO` and `Adafruit_Sensor`.
* **`SparkFun_ADXL345_Arduino_Library` / `Adafruit_ADXL345` (Arduino C++):** Because the ADXL343 is register-compatible with the ADXL345, libraries written for the ADXL345 work directly on the ADXL343.
* **`Adafruit_CircuitPython_ADXL34x` (CircuitPython / Python):** Accessed via `from adafruit_adxl34x import ADXL343`.

#### Features
* **Range & Data Rate Control:** `setRange()` (`ADXL343_RANGE_2_G`, `4_G`, `8_G`, `16_G`) and `setDataRate()` from `0.10 Hz` up to `3200 Hz` (`ADXL343_DATARATE_3200_HZ`).
* **Hardware Interrupt Routing (`enableInterrupts`, `mapInterrupts`, `checkInterrupts`):** Uses `int_config` bitfields to enable and independently map `overrun`, `watermark`, `freefall`, `inactivity`, `activity`, `double_tap`, `single_tap`, and `data_ready` events to physical pins `INT1` or `INT2`.
* **Hardware Offset Calibration:** `getTrimOffsets()` and `setTrimOffsets(x, y, z)` write directly to the sensor's internal offset registers (`OFSX`, `OFSY`, `OFSZ`) to zero out mounting bias in hardware.
* **Raw & Unified Reads:** Supports both `getEvent(&event)` (returning **m/s²**) and direct 16-bit integer reads via `getX()`, `getY()`, `getZ()`.

#### Caveats
1. **3-Axis Accelerometer Only (No Gyroscope):** Although grouped under the "Inertial Measurement Units (IMUs)" heading in the proposal, the ADXL343 only measures 3-axis linear acceleration and has **no gyroscope**. It cannot provide angular rate redundancy for the LSM6DS3TR-C.
2. **Pad Launch Detection Requires Tuning `THRESH_ACT` & AC/DC Coupling:** While `Adafruit_ADXL343` provides `enableInterrupts()` and `mapInterrupts()`, configuring the exact G-threshold (`ADXL3XX_REG_THRESH_ACT`, `0x24`, scaled at **62.5 mg/LSB**) and axis enable bits (`ADXL3XX_REG_ACT_INACT_CTL`, `0x27`) requires calling `writeRegister()`. On the pad, 1 g of gravity is always present, so DC-coupled activity thresholds must be set well above 1 g (e.g., > 2.5 g to 3.5 g) to prevent wind gusts or rail loading from prematurely waking the flight computer.
3. **Interrupt Pins (`INT1`/`INT2`) Are Not 5V-Shifted:** As documented in Adafruit's ADXL343 Breakout Learning Guide, only the `SCL` and `SDA` lines have level shifting on the breakout; auxiliary pins like `INT1` and `INT2` operate at **3.3V logic** and must be level-shifted externally if wired to a 5V microcontroller.
4. **10-Bit vs. 13-Bit Resolution (`FULL_RES` Bit):** The ADXL343 can operate in 10-bit mode (where the scale factor changes with the selected G-range) or 13-bit full-resolution mode (`FULL_RES` bit in `DATA_FORMAT` register `0x31`), which maintains a constant **3.9 mg/LSB** resolution all the way up to ±16 g (`Adafruit_ADXL343::setRange()` automatically sets `FULL_RES = 1`).
5. **High Data Rates (1600 Hz & 3200 Hz) Require SPI:** Standard 100 kHz or 400 kHz I2C is too slow to read all axes at 1600 Hz or 3200 Hz without data overruns; Analog Devices mandates SPI at **>= 2 MHz** for those rates.
6. **No Built-In FIFO Helpers in `Adafruit_ADXL343`:** The register addresses (`ADXL3XX_REG_FIFO_CTL` and `FIFO_STATUS`) are defined in the header, but the library lacks dedicated methods to configure Stream/Trigger FIFO modes.

#### Example Pad-Standby Launch Interrupt Snippet (Arduino C++)
```cpp
#include <Adafruit_ADXL343.h>

Adafruit_ADXL343 adxl = Adafruit_ADXL343(12345);

void setupADXL343LaunchDetect() {
  if (!adxl.begin(0x53)) {
    while (1) { delay(10); }
  }
  adxl.setRange(ADXL343_RANGE_16_G);
  adxl.setDataRate(ADXL343_DATARATE_400_HZ);

  // Set Activity Threshold: 62.5 mg/LSB -> 48 * 0.0625g = 3.0g launch threshold
  adxl.writeRegister(ADXL3XX_REG_THRESH_ACT, 48);
  // Enable DC-coupled activity detection on X, Y, Z axes (0x70)
  adxl.writeRegister(ADXL3XX_REG_ACT_INACT_CTL, 0x70);

  // Map Activity interrupt to INT1 and enable it
  int_config cfg = {0};
  cfg.bits.activity = ADXL343_INT1;
  adxl.mapInterrupts(cfg);

  cfg.bits.activity = 1;
  adxl.enableInterrupts(cfg);
}
```

#### Reference Links
* **Product Page:** https://www.adafruit.com/product/4097
* **Datasheet:** https://www.analog.com/media/en/technical-documentation/data-sheets/adxl343.pdf
* **Adafruit Learning Guide (PDF):** https://cdn-learn.adafruit.com/downloads/pdf/adxl343-breakout-learning-guide.pdf
* **Arduino Library (`Adafruit_ADXL343`):** https://github.com/adafruit/Adafruit_ADXL343
* **Arduino Library API Docs (`Adafruit_ADXL343`):** https://adafruit.github.io/Adafruit_ADXL343/html/index.html
* **SparkFun Arduino Library (`SparkFun_ADXL345`):** https://github.com/sparkfun/SparkFun_ADXL345_Arduino_Library
* **CircuitPython Library (`Adafruit_CircuitPython_ADXL34x`):** https://github.com/adafruit/Adafruit_CircuitPython_ADXL34x

---

## 5. Barometric Pressure Sensors

### 5.1 LPS25HB (Adafruit 4530 / Labeled 4258) — MEMS Barometer

The **LPS25HB** is an STMicroelectronics piezoresistive MEMS absolute pressure sensor covering 260 to 1260 hPa over SPI and I2C.

#### Proposal Hardware Details
* **Breakout Board:** Adafruit 4530 (labeled as "Adafruit 4258" in the proposal slide header; $9.95)
* **Pressure Range:** 260 hPa to 1260 hPa (260 hPa = 33,153 ft pressure altitude)
* **Hardware Features:** SPI and I2C interfaces, `INT` pin, onboard 3.3V regulator and level shifting

#### Libraries Used
* **`Adafruit_LPS2X` (Arduino C++):** Primary Adafruit driver (`#include <Adafruit_LPS2X.h>`, instantiated via `Adafruit_LPS25`), built on `Adafruit_BusIO` and `Adafruit_Sensor`.
* **`Pololu LPS` (`lps-arduino`, Arduino C++):** Lightweight I2C library (`#include <LPS.h>`) supporting LPS25H/HB, LPS331AP, and LPS22DF with built-in barometric altitude conversion methods.
* **`STM32duino LPS25HB` (Arduino C++):** Official STMicroelectronics library (`#include <LPS25HBSensor.h>`) supporting I2C and SPI.
* **`Adafruit_CircuitPython_LPS2X` (CircuitPython / Python):** Accessed via `adafruit_lps2x.LPS25`.

#### Features
* **Data Rate Configuration (`setDataRate`, `getDataRate`):** Configures Output Data Rate across `LPS25_RATE_ONE_SHOT`, `LPS25_RATE_1_HZ`, `LPS25_RATE_7_HZ`, `LPS25_RATE_12_5_HZ`, and `LPS25_RATE_25_HZ`.
* **Unified Sensor Events (`getEvent`):** Populates `sensors_event_t` objects for pressure (in **hPa / mbar**) and die temperature (in **°C**).
* **Direct Altitude Calculation (in `Pololu LPS`):** Includes static helper methods `pressureToAltitudeMeters(pressure_mbar, altimeter_setting_mbar)` and `pressureToAltitudeFeet(pressure_inHg, altimeter_setting_inHg)` alongside auto-detection of device type and slave address (`0x5D` or `0x5C`).

#### Caveats
1. **Low Maximum Sampling Rate (25 Hz):** The LPS25HB hardware maxes out at **25 Hz** (one sample every 40 ms). For an active airbrakes control loop on a high-speed rocket, 25 Hz introduces noticeable phase lag compared to faster barometers or IMU-fused state estimators.
2. **Altitude Ceiling Near 30k Class Apogees:** The sensor's minimum calibrated pressure is **260 hPa**, which corresponds to **33,153 ft** standard pressure altitude. If flying in the IREC 30k ft category from a high-elevation launch site (such as Spaceport America at ~4,600 ft MSL, where 30,000 ft AGL is ~34,600 ft MSL), the rocket will exceed the LPS25HB's 33,153 ft pressure floor.
3. **Minimal Feature Set in `Adafruit_LPS2X`:** The hardware supports a 32-slot FIFO (including hardware moving-average `FIFO_MEAN_MODE` filtering to reduce RMS noise to 0.01 hPa) and high/low pressure threshold interrupts on the `INT` pin, but `Adafruit_LPS2X` only implements basic data rate setting, reset, and pressure/temperature reads. Configuring hardware averaging requires manual register writes.
4. **Product ID Typo in Proposal:** The proposal slide heading lists `LPS25HB Adafruit 4258`, while the hyperlink points to `https://www.adafruit.com/product/4530`. Product **4530** is the actual Adafruit LPS25HB breakout.

#### Reference Links
* **Product Page:** https://www.adafruit.com/product/4530
* **Datasheet:** https://www.st.com/resource/en/datasheet/lps25h.pdf
* **Adafruit Learning Guide:** https://learn.adafruit.com/products/4530/guides
* **Arduino Library (`Adafruit_LPS2X`):** https://github.com/adafruit/Adafruit_LPS2X
* **Pololu Arduino Library (`lps-arduino`):** https://github.com/pololu/lps-arduino
* **STMicroelectronics Arduino Library (`STM32duino LPS25HB`):** https://docs.arduino.cc/libraries/stm32duino-lps25hb/ (Repository: https://github.com/stm32duino/LPS25HB)
* **CircuitPython Library Docs (`Adafruit_CircuitPython_LPS2X`):** https://docs.circuitpython.org/projects/lps2x/en/latest/

---

### 5.2 MS5611-01BA03 (Embedded Adventures MOD-1009) — MEMS Barometer

The **Embedded Adventures MOD-1009** is a breakout module for the TE Connectivity (Measurement Specialties) **MS5611-01BA03** 24-bit high-resolution MEMS barometer, rated from 10 to 1200 mbar/hPa (~102,000 ft) over SPI and I2C.

#### Proposal Hardware Details
* **Breakout Board:** Embedded Adventures MOD-1009 ($9.04)
* **Pressure Range:** 10 to 1200 mbar/hPa (10 mbar ≈ 102,000 ft)
* **Hardware Features:** 24-bit ΔΣ ADC, factory-calibrated PROM, SPI and I2C interfaces with onboard `Pullups` and `SPI I2C` solder jumpers

#### Libraries Used
* **`RobTillaart/MS5611` (I2C) & `RobTillaart/MS5611_SPI` (Arduino C++):** The most widely used, actively maintained Arduino libraries for the MS5611 (`#include <MS5611.h>` or `#include <MS5611_SPI.h>`).
* **`jarzebski/Arduino-MS5611` (Arduino C++):** Popular I2C library (`#include <MS5611.h>`) with built-in sea-level pressure and altitude calculation helpers.
* **Embedded Adventures `PicPack` (`ms5611.c` / `ms5611.h`):** The manufacturer's C driver referenced in the MOD-1009 datasheet, written for PIC microcontrollers using software I2C.

#### Features
* **Factory PROM Calibration & CRC Verification:** Reads the six 16-bit factory calibration coefficients ($C_1$ through $C_6$) stored in the MS5611's internal PROM on `begin()` / `reset()` and computes the 24-bit temperature-compensated pressure.
* **Configurable Oversampling Ratio (OSR):** `setOversampling()` lets you trade conversion speed for altitude resolution across 5 levels:

| OSR Setting (`RobTillaart/MS5611`) | Samples | Max Conversion Time (per ADC read) | RMS Resolution (mbar) | Approx. Altitude Resolution |
| :--- | :---: | :---: | :---: | :---: |
| `OSR_ULTRA_LOW` | 256 | ~0.60 ms | 0.065 mbar | ~54 cm |
| `OSR_LOW` | 512 | ~1.17 ms | 0.042 mbar | ~35 cm |
| `OSR_STANDARD` | 1024 | ~2.28 ms | 0.027 mbar | ~22 cm |
| `OSR_HIGH` | 2048 | ~4.54 ms | 0.018 mbar | ~15 cm |
| `OSR_ULTRA_HIGH` | 4096 | ~9.04 ms | 0.012 mbar | ~10 cm |

* **Second-Order Temperature Compensation (`setCompensation`):** Automatically applies the non-linear second-order temperature compensation algorithm defined in the MS5611 datasheet when ambient temperatures drop below 20°C (and a further correction below -15°C), which is essential at high altitudes.
* **Runtime Offset Calibration & Math Modes:** `setPressureOffset()` and `setTemperatureOffset()` allow zeroing out pad elevation or self-heating offsets; `reset(mathMode)` handles revision-specific PROM calculation differences (`mathMode = 0` for datasheet math, `mathMode = 1` for application-note math if a variant returns 50% pressure).

#### Caveats
1. **Blocking ADC Conversion Delays in Standard Libraries:** Unlike the LPS25HB (which updates output registers in the background), the MS5611 requires sending a "Start Pressure Conversion" ($D_1$) command, waiting up to 9.04 ms (at OSR 4096), reading the 24-bit ADC, sending a "Start Temperature Conversion" ($D_2$) command, and waiting another 9.04 ms. In `RobTillaart/MS5611` and `jarzebski/Arduino-MS5611`, calling `read()` uses blocking `delay()` / `delayMicroseconds()` calls totaling **~18 ms per sample** at highest resolution.
   * *Flight Software Mitigation:* For a real-time airbrakes control loop, either use a lower OSR (`OSR_STANDARD` / `OSR_HIGH`) or implement a **non-blocking state machine** using `micros()` (request $D_1$ on cycle $N$, read $D_1$ and request $D_2$ on cycle $N+1$). Additionally, because die temperature changes slowly compared to pressure during ascent, reading $D_2$ (temperature) only once every 5 to 10 $D_1$ (pressure) reads nearly doubles the effective pressure sampling rate.
2. **Strict 3.3V Logic & Severe Self-Heating Caveat:** Unlike the Adafruit breakouts (which include onboard 3.3V LDO regulators and level shifters), the Embedded Adventures **MOD-1009 has no voltage regulator or level shifter** ($V_{CC}$ is strictly **1.8V–4.0V**, absolute max 4.2V). Furthermore, `RobTillaart/MS5611` explicitly warns that driving the I2C address line, SPI select (`CS`), or protocol select line with **5V** causes **extreme internal self-heating (> 10°C)**, which corrupts the temperature-compensated pressure calculation. Only 3.3V logic lines may be used.
3. **Physical Hardware Jumper Required on MOD-1009 for SPI vs. I2C:** The MOD-1009 PCB includes solder jumpers labeled `Pullups` and `SPI I2C`. It ships with the `SPI I2C` 3-pad jumper bridged for I2C and 10 kΩ pull-ups enabled. In I2C mode, you must tie the `CS` pin high (address `0x76`) or low (`0x77`). To use SPI, you must physically desolder the I2C jumper bridge and solder the `SPI` side of the jumper on the PCB.
4. **Sensitivity to Sunlight & Mach Transients:** The MS5611 MEMS die is exposed through two vents in the metal lid. Direct or reflected sunlight entering through avionics bay static ports creates false pressure spikes due to the photovoltaic effect on the silicon die, so the sensor must be optically shielded (e.g., with breathable black open-cell foam).

#### Reference Links
* **Product Page:** https://www.embeddedadventures.com/barometric_pressure_sensor_module_mod-1009.html
* **Breakout Board Datasheet (MOD-1009):** https://www.embeddedadventures.com/datasheets/MOD-1009_hw_v3_doc_v2.pdf
* **MS5611 Chip Datasheet:** https://www.hpinfotech.ro/MS5611-01BA03.pdf
* **Arduino I2C Library (`RobTillaart/MS5611`):** https://github.com/RobTillaart/MS5611 (Arduino Docs: https://docs.arduino.cc/libraries/ms5611/)
* **Arduino SPI Library (`RobTillaart/MS5611_SPI`):** https://github.com/RobTillaart/MS5611_SPI
* **Alternative Arduino Library (`jarzebski/Arduino-MS5611`):** https://github.com/jarzebski/Arduino-MS5611

---

## 6. Master Reference & Documentation Links

### Links Directly from Proposal Document
* **Dissimilar Redundancy (Wikipedia):** https://en.wikipedia.org/wiki/Redundancy_(engineering)#Dissimilar_redundancy
* **LSM6DS3TR-C Product Page (Adafruit 4503):** https://www.adafruit.com/product/4503
* **LSM6DS3TR-C Datasheet (STMicroelectronics):** https://www.st.com/resource/en/datasheet/lsm6ds3tr-c.pdf
* **ADXL343 Product Page (Adafruit 4097):** https://www.adafruit.com/product/4097
* **ADXL343 Datasheet (Analog Devices):** https://www.analog.com/media/en/technical-documentation/data-sheets/adxl343.pdf
* **LPS25HB Product Page (Adafruit 4530):** https://www.adafruit.com/product/4530
* **LPS25HB Datasheet (STMicroelectronics):** https://www.st.com/resource/en/datasheet/lps25h.pdf
* **MS5611 Breakout Product Page (Embedded Adventures MOD-1009):** https://www.embeddedadventures.com/barometric_pressure_sensor_module_mod-1009.html
* **MS5611 Breakout Datasheet (MOD-1009):** https://www.embeddedadventures.com/datasheets/MOD-1009_hw_v3_doc_v2.pdf
* **MS5611 Chip Datasheet (TE Connectivity):** https://www.hpinfotech.ro/MS5611-01BA03.pdf

### Software Libraries & Manufacturer Guides
* **LSM6DS3TR-C — `Adafruit_LSM6DS` (Arduino):** https://github.com/adafruit/Adafruit_LSM6DS
* **LSM6DS3TR-C — `STM32duino LSM6DS3TR-C` (Arduino):** https://github.com/stm32duino/LSM6DS3TR-C
* **LSM6DS3TR-C — `SparkFun_LSM6DS3` (Arduino):** https://github.com/sparkfun/SparkFun_LSM6DS3_Arduino_Library
* **LSM6DS3TR-C — `Adafruit_CircuitPython_LSM6DS`:** https://github.com/adafruit/Adafruit_CircuitPython_LSM6DS
* **LSM6DS3TR-C — Adafruit Learning Guide:** https://cdn-learn.adafruit.com/downloads/pdf/adafruit-lsm6ds3tr-c-6-dof-accel-gyro-imu.pdf
* **ADXL343 — `Adafruit_ADXL343` (Arduino):** https://github.com/adafruit/Adafruit_ADXL343
* **ADXL343 — `Adafruit_ADXL343` API Documentation:** https://adafruit.github.io/Adafruit_ADXL343/html/index.html
* **ADXL343 — `SparkFun_ADXL345` (Arduino):** https://github.com/sparkfun/SparkFun_ADXL345_Arduino_Library
* **ADXL343 — `Adafruit_CircuitPython_ADXL34x`:** https://github.com/adafruit/Adafruit_CircuitPython_ADXL34x
* **ADXL343 — Adafruit Learning Guide:** https://cdn-learn.adafruit.com/downloads/pdf/adxl343-breakout-learning-guide.pdf
* **LPS25HB — `Adafruit_LPS2X` (Arduino):** https://github.com/adafruit/Adafruit_LPS2X
* **LPS25HB — `Pololu LPS` (Arduino):** https://github.com/pololu/lps-arduino
* **LPS25HB — `STM32duino LPS25HB` (Arduino):** https://github.com/stm32duino/LPS25HB
* **LPS25HB — `Adafruit_CircuitPython_LPS2X` Docs:** https://docs.circuitpython.org/projects/lps2x/en/latest/
* **LPS25HB — Adafruit Learning Guide:** https://learn.adafruit.com/products/4530/guides
* **MS5611 — `RobTillaart/MS5611` (Arduino I2C):** https://github.com/RobTillaart/MS5611
* **MS5611 — `RobTillaart/MS5611_SPI` (Arduino SPI):** https://github.com/RobTillaart/MS5611_SPI
* **MS5611 — `jarzebski/Arduino-MS5611` (Arduino I2C):** https://github.com/jarzebski/Arduino-MS5611
