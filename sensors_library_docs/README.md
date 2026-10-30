# IREC 2027 Airbrakes Sensor Proposals & Firmware Reference

This repository documents the sensor selection, firmware libraries, features, and avionics caveats for the **Technology & Aerospace Club's IREC 2027 Airbrakes** avionics stack. 

To achieve high reliability and mitigate single-point hardware and software failures, the system implements **dissimilar redundancy** (utilizing different hardware components and sensor topologies to eliminate identical failure modes).

---

## Sensor Overview & Library Quick Reference

| Sensor in Proposal | Category | Primary Vendor Library | Key Alternative Libraries | Supported Buses (Library) |
| :--- | :--- | :--- | :--- | :--- |
| **1. ST LSM6DS3TR-C** (Adafruit 4503) | 6-DoF IMU | `Adafruit_LSM6DS` (`Adafruit_LSM6DS3TRC`) | `SparkFun_LSM6DS3_Arduino_Library`, `STM32duino LSM6DS3` | $I^2C$, SPI |
| **2. Analog Devices ADXL343** (Adafruit 4097) | 3-Axis Accelerometer | `Adafruit_ADXL343` | `SparkFun_ADXL345_Arduino_Library`, `ADXL345_WE` | $I^2C$, SPI |
| **3. ST LPS25HB** (Adafruit 4530 / 4258) | Barometer | `Adafruit_LPS2X` (`Adafruit_LPS25`) | `Pololu LPS` (`lps-arduino`), `STM32duino LPS25HB`, `SparkFun_LPS25HB` | $I^2C$, SPI (`Pololu` & `SparkFun` are $I^2C$-only) |
| **4. TE MS5611-01BA03** (Embedded Adventures MOD-1009) | Barometer | Embedded Adventures `PicPack` (`ms5611.c` / `.h`) | `RobTillaart/MS5611` ($I^2C$), `RobTillaart/MS5611_SPI` (SPI), `jarzebski/Arduino-MS5611` | $I^2C$, SPI (split across libraries) |

---

## 1. LSM6DS3TR-C (Adafruit 4503) — 6-DoF IMU

* **Proposal Spec:** $\pm 16\text{ g}$ acceleration range, $\pm 2000\text{ dps}$ gyro range, wakeup interrupts, SPI and $I^2C$.
* **Primary Library:** `Adafruit_LSM6DS` (`Adafruit_LSM6DS3TRC` subclass)
  * **Dependencies:** `Adafruit_BusIO`, `Adafruit_Sensor` (C++/Arduino); `adafruit_lsm6ds`, `adafruit_bus_device`, `adafruit_register` (CircuitPython).
  * **Features:**
    * Supports $I^2C$ (`begin_I2C()`) and hardware/software SPI (`begin_SPI()`).
    * Automatically sets the Block Data Update (`BDU`) bit during initialization (`_init()`) to prevent MSB/LSB register tearing during burst reads.
    * Exposes configurable full-scale ranges ($\pm 2\text{ g}$ to $\pm 16\text{ g}$; $\pm 125\text{ dps}$ to $\pm 2000\text{ dps}$) and Output Data Rates (ODR up to 6.66 kHz).
    * Provides SI-unit `sensors_event_t` structures via `getEvent()` ($m/s^2$, $rad/s$, $^\circ C$) and public 16-bit raw integer fields (`rawAccX/Y/Z`, `rawGyroX/Y/Z`, `rawTemp`).
    * Includes helpers for wakeup interrupts (`enableWakeup()`), data-ready interrupts on `INT1`/`INT2` (`configInt1()`, `configInt2()`), high-pass/slope filters (`highPassFilter()`), and step detection (`enablePedometer()`).

  * **Caveats & Avionics Risks:**
    * **No Hardware FIFO or Tap Detection Support:** Adafruit omits wrappers for the chip's 4 KB hardware FIFO buffer and single/double-tap detection. Use ST's official driver or SparkFun's library if deep FIFO buffering is required.
    * **All-in-One Read & Float Conversion Overhead:** Calling `getEvent()` or `_read()` always burst-reads all 14 bytes (temperature, gyroscope, and accelerometer) and executes floating-point scaling unconditionally for all 3 sensors even if your control loop only needs accelerometer data or raw integers, increasing CPU overhead in fast control loops.
    * **Silent Bus Read Failures:** Internal `_read()` returns `void` and `getEvent()` unconditionally returns `true` without propagating $I^2C$/SPI transaction errors ([GitHub Issue #26](https://github.com/adafruit/Adafruit_LSM6DS/issues/26)). In-flight bus glitches will silently yield stale or corrupted values.
    * **No Public Generic Register Access Methods:** Modifying unexposed registers (such as FIFO or custom filter settings) requires instantiating `Adafruit_BusIO_Register` objects manually rather than calling a simple `writeRegister()` method ([GitHub Issue #47](https://github.com/adafruit/Adafruit_LSM6DS/issues/47)).
    * **`WHO_AM_I` Chip ID Mismatch (`0x69` vs. `0x6A`):** Standard non-C `LSM6DS3` libraries check for `0x69`, whereas the `LSM6DS3TR-C` reports `0x6A` (`LSM6DS3TRC_CHIP_ID`), causing initialization failures unless updated.

### References & Links
* [Adafruit LSM6DS GitHub Repository](https://github.com/adafruit/Adafruit_LSM6DS)
* [Adafruit LSM6DS Class Doxygen Reference](https://adafruit.github.io/Adafruit_LSM6DS/html/class_adafruit___l_s_m6_d_s.html)
* [Adafruit LSM6DS3TRC Source Implementation](https://github.com/adafruit/Adafruit_LSM6DS/blob/master/Adafruit_LSM6DS3TRC.cpp)
* [Adafruit LSM6DS GitHub Issues (#26 Silent Failures, #47 Register Access)](https://github.com/adafruit/Adafruit_LSM6DS/issues)
* [Adafruit LSM6DS3TR-C Learning Guide (FIFO/Tap Caveats)](https://learn.adafruit.com/adafruit-lsm6ds3tr-c-lis3mdl-precision-9-dof-imu/overview)
* [SparkFun LSM6DS3 Arduino Library](https://github.com/sparkfun/SparkFun_LSM6DS3_Arduino_Library)
* [STM32duino LSM6DS3 Library](https://github.com/stm32duino/LSM6DS3)

---

## 2. ADXL343 (Adafruit 4097) — Triple-Axis Accelerometer

* **Proposal Spec:** $\pm 16\text{ g}$ range, 2 programmable interrupts, built-in motion/tap/free-fall detection for low-power pad standby modes.
* **Primary Library:** `Adafruit_ADXL343`
  * **Dependencies:** `Adafruit_BusIO`, `Adafruit_Sensor` (C++/Arduino); `adafruit_adxl34x` (CircuitPython).
  * **Features:**
    * Supports $I^2C$, hardware SPI (`Adafruit_ADXL343(cs, &SPI)`), and software SPI.
    * Provides SI-unit `getEvent()` and low-overhead raw 16-bit integer reads via `getXYZ(int16_t &x, int16_t &y, int16_t &z)`.
    * Configurable g-ranges ($\pm 2\text{ g}$ to $\pm 16\text{ g}$) and data rates from 0.10 Hz to 3200 Hz.
    * Exposes hardware offset trim registers (`getTrimOffsets()`, `setTrimOffsets()`) and low-level `writeRegister()` / `readRegister()` methods.
    * Implements bitfield structures (`int_config`) for routing `overrun`, `watermark`, `freefall`, `inactivity`, `activity`, `double_tap`, `single_tap`, and `data_ready` events to `INT1` or `INT2`.
  * **Caveats & Avionics Risks:**
    * **No High-Level Helpers for Motion Thresholds / FIFO Control:** Macros like `ADXL3XX_REG_THRESH_ACT` are defined, but dedicated setter functions for activity/inactivity thresholds or the 32-level FIFO are missing. Thresholds must be written manually via `writeRegister()`.
    * **ISR Bus-Lockup Hazard in `checkInterrupts()`:** Reading `INT_SOURCE` to clear latched motion interrupts inside an Interrupt Service Routine (ISR) can lock up $I^2C$ transactions and hang the microcontroller. Use pins directly to identify triggers or set `volatile` flags for main-loop handling.
    * **No Warning on High-ODR $I^2C$ Bottlenecks:** The library permits setting 1600 Hz or 3200 Hz ODR over $I^2C$, risking bus saturation and data overruns unless SPI ($\ge 2\text{ Hz}$) is used.
    * **3.3V Logic Limit on Interrupt Pins:** `INT1` and `INT2` on the Adafruit 4097 breakout are 3.3V logic only and require level shifting if interfaced with 5V MCUs.

### References & Links
* [Adafruit ADXL343 GitHub Repository](https://github.com/adafruit/Adafruit_ADXL343)
* [Adafruit ADXL343 Class Doxygen Reference](https://adafruit.github.io/Adafruit_ADXL343/html/class_adafruit___a_d_x_l343.html)
* [Adafruit ADXL343 Interrupts Example (ISR Warning Note)](https://github.com/adafruit/Adafruit_LSM6DS/blob/master/examples/interrupts/interrupts.ino)
* [Adafruit ADXL343 Learning Guide](https://learn.adafruit.com/adxl343-breakout-learning-guide)
* [SparkFun ADXL345 Arduino Library](https://github.com/sparkfun/SparkFun_ADXL345_Arduino_Library)

---

## 3. LPS25HB (Adafruit 4530 / 4258) — Barometer

* **Proposal Spec:** 260 to 1260 hPa range ($260\text{ hPa} \approx 33,153\text{ ft}$), SPI and $I^2C$ MEMS barometer.
* **Primary Library:** `Adafruit_LPS2X` (`Adafruit_LPS25` subclass)
  * **Dependencies:** `Adafruit_BusIO`, `Adafruit_Sensor` (C++/Arduino); `adafruit_lps2x`, `adafruit_bus_device`, `adafruit_register` (CircuitPython).
  * **Features:**
    * Supports $I^2C$ (`begin_I2C()`) and hardware/software SPI (`begin_SPI()`).
    * Returns pressure in hPa and temperature in $^\circ\text{C}$ via `getEvent(&pressure, &temp)`.
    * Supports software `reset()` and Output Data Rate configuration via `setDataRate()` (`LPS25_RATE_ONE_SHOT` up to `25_HZ`).
  * **Caveats & Avionics Risks:**
    * **25 Hz Maximum Data Rate Ceiling:** Hardware-limited to a maximum continuous ODR of 25 Hz (`LPS25_RATE_25_HZ`). Polling faster than 40 ms returns duplicated measurements.
    * **No Wrappers for Hardware FIFO Mean Mode or Internal Averaging:** The chip supports a hardware moving average (`FIFO_MEAN_MODE`) and internal noise-suppression filtering (`RES_CONF`), but `Adafruit_LPS25` does not expose methods to configure them.
    * **No Built-in Altitude Calculation Helper:** Unlike BMP280/BMP3XX libraries, `Adafruit_LPS2X` outputs raw pressure/temperature only; altitude calculations must be implemented manually.
    * **Alternative Library Limitations:** `Pololu LPS` and `SparkFun_LPS25HB` are $I^2C$-only (no SPI). Furthermore, Pololu's built-in altitude conversion caps out at **11 km (~36,000 ft)**.

### References & Links
* [Adafruit LPS2X GitHub Repository](https://github.com/adafruit/Adafruit_LPS2X)
* [Adafruit LPS25 and LPS22 Learning Guide](https://learn.adafruit.com/adafruit-lps25-pressure-sensor)
* [Adafruit CircuitPython LPS2X API Reference](https://docs.circuitpython.org/projects/lps2x/en/stable/api.html)
* [Pololu LPS Arduino Library (Features & 11 km Altitude Limit)](https://github.com/pololu/lps-arduino)
* [STM32duino LPS25HB GitHub Repository](https://github.com/stm32duino/LPS25HB)
* [SparkFun LPS25HB Arduino Library](https://github.com/sparkfun/SparkFun_LPS25HB_Arduino_Library)

---

## 4. MS5611 (Embedded Adventures MOD-1009) — Barometer

* **Proposal Spec:** 10 to 1200 mbar/hPa range ($10\text{ mbar} \approx 102,000\text{ ft}$), SPI and $I^2C$ high-precision barometer.
* **Primary Vendor Library:** Embedded Adventures `PicPack` (`ms5611.c` / `ms5611.h`)
  * **Features:** Official vendor source code implementing PROM coefficient reads (`C1`–`C6`), 24-bit ADC conversions, and temperature compensation.
  * **Caveats & Avionics Risks:**
    * **PIC Microcontroller Target Architecture:** Written specifically for Microchip PIC microcontrollers using `PicPack`'s custom `i2c.c` routines; will not compile in Arduino/PlatformIO (Teensy, STM32, ESP32) without porting.
    * **MOD-1009 Hardware Jumper Requirements:** Operates strictly at **1.8V–4.0V** with no onboard regulator. Ships pre-configured for $I^2C$; switching to SPI requires physically desoldering and re-bridging PCB solder jumpers.
* **Standard C++/Arduino Library:** `RobTillaart/MS5611` ($I^2C$) & `RobTillaart/MS5611_SPI` (SPI)
  * **Features:** 5 selectable Oversampling Ratios (OSR), 2nd-order non-linear temperature compensation (`setCompensation(true)`), manual math-mode correction (`reset(mathMode)`), and runtime offset calibration.
  * **Caveats & Avionics Risks:**
    * **Blocking `read()` Conversions (~18.1 ms Stall at High OSR):** At `OSR_ULTRA_HIGH` (4096), sequential temperature (`D2`) and pressure (`D1`) conversions take ~9.04 ms each, stalling the CPU for **~18.1 ms** per reading. High-rate control loops require lower OSR or non-blocking state machines.
    * **Severe Self-Heating in SPI Mode & 5V Logic Risk:** Operating over SPI or applying 5V logic to `CS`/address lines causes significant internal self-heating ($+0.1^\circ\text{C}$ to $>10^\circ\text{C}$), which skews temperature compensation and drifts pressure readings.
    * **Split Repositories:** $I^2C$ and SPI implementations are split into separate libraries.

### References & Links
* [Embedded Adventures MOD-1009 Datasheet (PicPack & Jumper Details)](https://www.embeddedadventures.com/datasheets/MOD-1009_hw_v3_doc_v2.pdf)
* [RobTillaart MS5611 ($I^2C$) GitHub Repository](https://github.com/RobTillaart/MS5611)
* [RobTillaart MS5611_SPI GitHub Repository (SPI Self-Heating Warnings)](https://github.com/RobTillaart/MS5611_SPI)
* [Jarzebski Arduino-MS5611 GitHub Repository](https://github.com/jarzebski/Arduino-MS5611)
* [Jarzebski Arduino-MS5611 Releases (`v1.1.0` Sub-20°C Fix)](https://github.com/jarzebski/Arduino-MS5611/releases)
