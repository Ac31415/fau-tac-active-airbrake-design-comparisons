# IREC 2027 Airbrakes — Sensor Libraries: Features & Caveats

> Source document: `avionics_airbrakes_sensor_proposals.pdf` — Technology & Aerospace Club, IREC 2027 – Airbrakes Sensor Proposals
> Researched: 2026-10-30
> Method: For every sensor in the PDF, the library / libraries you would actually use to drive it were identified from the breakout vendor's product / learning-guide pages, then the features and caveats below were taken from the library's own GitHub README, its API / Doxygen / Read-the-Docs reference, or the vendor learning guide. **Every bullet gives the exact web source it came from** — page title, URL, and the section / wording on that page. No feature or caveat is listed without a source.
> Quotes are short excerpts / close paraphrases of the heading or sentence on the cited page, included so you can find the spot with Ctrl-F.

---

## 1. Sensors found in the PDF

| # | Type in PDF | Sensor as written in PDF | Breakout in PDF | What the breakout actually is (verified) |
|---|---|---|---|---|
| 1 | IMU | LSM6DS3TR | Adafruit 4503 | ST **LSM6DS3TR-C** 6-DoF accel + gyro. Adafruit's guide: "The board includes an **ST LSM6DS3TR-C**, a great entry-level 6-DoF IMU accelerometer + gyro." — [Adafruit LSM6DS3TR-C 6-DoF Accel + Gyro IMU](https://learn.adafruit.com/adafruit-lsm6ds3tr-c-6-dof-accel-gyro-imu?view=all), *Overview* |
| 2 | IMU / accelerometer | ADXL343 | Adafruit 4097 | Analog Devices ADXL343 triple-axis accelerometer, I2C/SPI. — [Analog Devices ADXL343 Breakout Learning Guide](https://learn.adafruit.com/adxl343-breakout-learning-guide?view=all), *Overview* |
| 3 | Barometer | LPS25HB | "Adafruit 4258" in the heading, but the product link in the PDF is product **4530** | ST LPS25HB. The 4530 ID is confirmed as the LPS25HB breakout in the CircuitPython LPS2X API docs: "Adafruit LPS25 Pressure Sensor LPS25HB Breakout (Product ID: 4530)". — [API Reference — Adafruit LPS2x Library](https://docs.circuitpython.org/projects/lps2x/en/latest/api.html), *Implementation Notes / Hardware*. **Treat "4258" in the PDF as a typo — do not order / code against 4258 for this part.** |
| 4 | Barometer | MS5611 | Embedded Adventures MOD-1009 | MS5611-01BA03 module. Embedded Adventures: "The MOD-1009 is an MS5611 based absolute air pressure sensor module (barometer/altimeter)." — [MOD-1009 HWv3 datasheet](https://www.embeddedadventures.com/datasheets/MOD-1009_hw_v3_doc_v2.pdf), via the product page [Embedded Adventures MOD-1009](http://www.embeddedadventures.com/barometric_pressure_sensor_module_mod-1009.html) |

Plan note from the PDF itself: multiple sensors of each type, for *dissimilar redundancy*. That makes the I2C-address caveats in §6 especially important — every sensor family below only offers **two** I2C addresses.

---

## 2. LSM6DS3TR-C (Adafruit 4503)

### 2.1 Libraries used

| Library | Language / platform | Use it for |
|---|---|---|
| **Adafruit LSM6DS** — `Adafruit_LSM6DS3TRC.h` | Arduino / C++ | Primary library for this breakout |
| **Adafruit CircuitPython LSM6DS** — `adafruit_lsm6ds.lsm6ds3trc.LSM6DS3TRC` | CircuitPython / Python (Blinka) | Python version of the same family driver |
| ST's own LSM6DS3TR-C library (referenced by Adafruit, not an Adafruit library) | Arduino / C++ (advanced) | FIFO management, tap detection and other advanced features Adafruit's library does not expose — see Caveats |
| ⚠️ **Arduino_LSM6DS3** (`Arduino_LSM6DS3.h`, by Arduino) — *different, look-alike library, see Caveats* | Arduino | Only for boards with a built-in LSM6DS3, **not** this breakout |

### 2.2 Adafruit LSM6DS — Arduino

**Features**

- Family driver, not a single-chip driver: the repo contains separate classes/files for LSM6DS3, LSM6DS33, **LSM6DS3TRC**, LSM6DSL, LSM6DSOX, LSM6DSO32 and ISM330DHCX (`Adafruit_LSM6DS3TRC.cpp/.h` are listed in the repo root).
  Source: [GitHub — adafruit/Adafruit_LSM6DS](https://github.com/adafruit/Adafruit_LSM6DS), *Root files*
- Unified-sensor events for all three sub-sensors at once: `getEvent(sensors_event_t *accel, sensors_event_t *gyro, sensors_event_t *temp)` — "Gets the most recent sensor event, Adafruit Unified Sensor format."
  Source: [Adafruit_LSM6DS3TRC Class Reference](https://adafruit.github.io/Adafruit_LSM6DS/html/class_adafruit___l_s_m6_d_s3_t_r_c.html), inherited `Adafruit_LSM6DS` functions
- Configurable accelerometer and gyro, independently: `setAccelRange()` / `getAccelRange()`, `setGyroRange()` / `getGyroRange()`, `setAccelDataRate()` / `getAccelDataRate()`, `setGyroDataRate()` / `getGyroDataRate()`.
  Source: same Class Reference page, function table
- Both buses in the API despite the short README: `begin_I2C(...)`, `begin_SPI(cs_pin, ...)` for hardware SPI, and `begin_SPI(cs, sck, miso, mosi, ...)` for software SPI.
  Source: same Class Reference page, `begin_I2C` / `begin_SPI` rows
- Interrupts: `configIntOutputs(active_low, open_drain)`, `configInt1(drdy_temp, drdy_g, drdy_xl, step_detect, wakeup)`, `configInt2(drdy_temp, drdy_g, drdy_xl)` — "Enables and disables the data ready interrupt on INT 1 / INT 2."
  Source: same Class Reference page, `configInt*` rows
- Wake-up / shake: `enableWakeup(enable, duration, thresh)`, `awake()`, and `shake()` — "Simple shake detection. Must call `enableWakeup()` first."
  Source: same Class Reference page, `enableWakeup` / `shake` rows
- Pedometer (relevant to the PDF's "standby on the pad" idea, for the other IMU): `enablePedometer(bool)`, `resetPedometer()`, `readPedometer()` — "Read the 16-bit pedometer count."
  Source: same Class Reference page, `enablePedometer` rows (there are two: one on the TRC class itself, one inherited)
- High-pass / slope filter: `highPassFilter(enabled, filter)` — "Enables the high pass filter and/or slope filter."
  Source: same Class Reference page, `highPassFilter` row
- Software reset: `reset()` — "Resets the sensor to its power-on state, clearing all registers and memory."
  Source: same Class Reference page, `reset` row
- Raw + cooked values are both exposed: `rawAccX/Y/Z`, `rawGyroX/Y/Z`, `rawTemp`, plus `accX/Y/Z` in m/s² and `gyroX/Y/Z` in rad/s, and `temperature` in °C.
  Source: same Class Reference page, *Public Attributes inherited from Adafruit_LSM6DS*

**Caveats**

- **Dependencies are mandatory:** "Dependencies — Adafruit_BusIO, Adafruit_Sensor." The library will not compile / work standalone.
  Source: [GitHub — adafruit/Adafruit_LSM6DS](https://github.com/adafruit/Adafruit_LSM6DS), *README / Dependencies*
- **README understates the bus support:** the GitHub README says only "This chip uses I2C to communicate, 2 pins are required to interface" — yet the API (above) and the learning guide both provide SPI (`begin_SPI`, and "Both I2C and SPI interfaces are available"). Do not rule out SPI based on the README sentence alone.
  Sources: [GitHub — adafruit/Adafruit_LSM6DS](https://github.com/adafruit/Adafruit_LSM6DS), *README*; [Adafruit LSM6DS3TR-C guide](https://learn.adafruit.com/adafruit-lsm6ds3tr-c-6-dof-accel-gyro-imu?view=all), *Overview* — "Both I2C and SPI interfaces are available"
- **Not firmware-compatible with the LSM6DS33, and you must name the exact chip:** "Note it is not firmware-compatible with the 'DS33, so you will need to recompile code (e.g. our Arduino and Python libraries support the whole family but you *do* have to indicate which exact chip you're using)."
  Source: [Adafruit LSM6DS3TR-C guide](https://learn.adafruit.com/adafruit-lsm6ds3tr-c-6-dof-accel-gyro-imu?view=all), *Overview*
- **No FIFO / tap detection in the Adafruit library — that is in ST's library:** "For advanced Arduino usage, ST has their own fully-featured library that includes extras such as FIFO management and tap detection for the LSM6DS3TR-C."
  Source: [Adafruit product 4503](https://www.adafruit.com/product/4503), product description — "We also wrote libraries… For advanced Arduino usage, ST has their own fully-featured library that includes extras such as FIFO management and tap detection"
- **Only two I2C addresses:** default `0x6A`; "The final address is **0x6A + AD0** which would be **0x6B**" if the AD0 jumper is soldered closed. You cannot put more than two un-multiplexed LSM6DS3TR-C breakouts on one I2C bus.
  Source: [Adafruit LSM6DS3TR-C guide](https://learn.adafruit.com/adafruit-lsm6ds3tr-c-6-dof-accel-gyro-imu?view=all), *Pinouts / Address Jumper*
- **Do not use the base class directly (Arduino_LSM6DS3 look-alike risk, family edition):** in the family API the base `Adafruit_LSM6DS` is documented "Base class for use with LSM6DS series… DO NOT USE DIRECTLY. Specific sensor variants should be subclassed as needed" — use `Adafruit_LSM6DS3TRC`.
  Source: [Adafruit LSM6DS — Adafruit_LSM6DS.h File Reference](https://adafruit.github.io/Adafruit_LSM6DS/html/_adafruit___l_s_m6_d_s_8h.html), class list (found via search result for that page)

### 2.3 Adafruit CircuitPython LSM6DS

**Features**

- Same family approach in Python: import the exact subclass — `from adafruit_lsm6ds.lsm6ds3trc import LSM6DS3TRC`.
  Source: [Adafruit LSM6DS3TR-C guide — Python & CircuitPython](https://learn.adafruit.com/adafruit-lsm6ds3tr-c-6-dof-accel-gyro-imu?view=all), *Python Usage* example code
- Simple properties, SI units: `acceleration` — "The x, y, z acceleration values returned in a 3-tuple and are in m / s ^ 2"; `gyro` — "…in radians / second"; plus `temperature` in Celsius.
  Source: [API Reference — Adafruit LSM6DS Library](https://docs.circuitpython.org/projects/lsm6dsox/en/latest/api.html), `LSM6DS` class — `acceleration` / `gyro` / `temperature`
- Configurable in Python too: `accelerometer_range` ("from +/- 2G to +/-16G"), `gyro_range` ("from 125 Degrees/s to 2000 degrees/s"), `accelerometer_data_rate`, `gyro_data_rate`, `high_pass_filter`.
  Source: same API Reference page, `LSM6DS` class properties
- Pedometer in Python: `pedometer_enable`, `pedometer_steps`, `pedometer_reset` — "The number of steps detected by the pedometer. You must enable with `pedometer_enable` before calling. Use `pedometer_reset` to reset…"
  Source: same API Reference page, `LSM6DS` class — `pedometer_steps`
- Installable from PyPI for CPython / Raspberry Pi use: `pip3 install adafruit-circuitpython-lsm6ds`.
  Source: [GitHub — adafruit/Adafruit_CircuitPython_LSM6DS](https://github.com/adafruit/Adafruit_CircuitPython_LSM6DS), *Installing from PyPI*

**Caveats**

- **Dependencies must be on the CircuitPython filesystem:** "This driver depends on: Adafruit CircuitPython, Bus Device, Register. Please ensure all dependencies are available on the CircuitPython filesystem."
  Source: [GitHub — adafruit/Adafruit_CircuitPython_LSM6DS](https://github.com/adafruit/Adafruit_CircuitPython_LSM6DS), *Dependencies*
- **I2C-only in CircuitPython:** the class constructor is `LSM6DS(i2c_bus, address=0x6A, …)` — there is no SPI constructor in the CircuitPython API, unlike the Arduino version.
  Source: [API Reference — Adafruit LSM6DS Library](https://docs.circuitpython.org/projects/lsm6dsox/en/latest/api.html), `LSM6DS` class signature
- **Docs / examples default to a different chip:** the GitHub README usage example and the API page's main class are written for the LSM6DSOX (`from adafruit_lsm6ds.lsm6dsox import LSM6DSOX`). For this sensor you must swap the import to `lsm6ds3trc` — the learning guide's example explicitly says to comment out the LSM6DSOX import and uncomment the LSM6DS3TRC one.
  Sources: [GitHub — adafruit/Adafruit_CircuitPython_LSM6DS](https://github.com/adafruit/Adafruit_CircuitPython_LSM6DS), *Usage Example*; [Adafruit LSM6DS3TR-C + LIS3MDL guide — Python](https://learn.adafruit.com/adafruit-lsm6ds3tr-c-lis3mdl-precision-9-dof-imu/python-circuitpython), *Python Usage*
- **Larger range = less accurate (applies in both languages, stated in the Python docs):** "Note that larger ranges will be less accurate. Must be an `AccelRange`" / "Must be a `GyroRange`."
  Source: [API Reference — Adafruit LSM6DS Library](https://docs.circuitpython.org/projects/lsm6dsox/en/latest/api.html), `accelerometer_range` / `gyro_range`

### 2.4 ⚠️ Look-alike library: Arduino_LSM6DS3 (by Arduino) — do not pick this for the breakout

- It is a *different* library with a confusingly similar name: "Allows you to read the accelerometer, gyroscope, and temperature values from the LSM6DS3 IMU on your Arduino Nano 33 IoT or Arduino Uno WiFi Rev2 boards."
  Source: [Arduino_LSM6DS3 | Arduino Documentation](https://www.arduino.cc/reference/en/libraries/arduino_lsm6ds3/accelerationsamplerate) (Arduino library reference entry, description text found via search of docs.arduino.cc / the library's docs)
- Its settings are **fixed, not configurable**: "The ArduinoLSM6DS3 library takes care of the sensor initialization and sets its values as follows: Accelerometer range is set at ±4 g… Gyroscope range is set at ±2000 dps… Output data rate is fixed at 104 Hz."
  Source: same Arduino Documentation entry for Arduino_LSM6DS3, *description*
- Repo: https://github.com/arduino-libraries/Arduino_LSM6DS3
  Source: clone / navigate URL given in the library's DeepWiki installation page, *Method 2: Manual Installation from GitHub*

---

## 3. ADXL343 (Adafruit 4097)

### 3.1 Libraries used

| Library | Language / platform | Use it for |
|---|---|---|
| **Adafruit ADXL343** — `Adafruit_ADXL343.h` | Arduino / C++ | Primary library for this breakout (product 4097) |
| **Adafruit CircuitPython ADXL34x** — `adafruit_adxl34x.ADXL343` | CircuitPython / Python (Blinka) | Python driver; one module covers ADXL343 **and** ADXL345 |

### 3.2 Adafruit ADXL343 — Arduino

**Features**

- Unified Sensor driver: "This driver is for the Adafruit ADXL343 Breakout (https://www.adafruit.com/product/4097), and is based on Adafruit's Unified Sensor Library (Adafruit_Sensor)."
  Source: [GitHub — adafruit/Adafruit_ADXL343](https://github.com/adafruit/Adafruit_ADXL343), *README*
- Unified Sensor benefit — data already in SI units and swappable sensors: "It provides a simple abstraction layer… returns standard SI units… You can use the data right away because it's already converted to SI units… rather than meaningless values like 0..1023."
  Source: same GitHub README, *What is the Adafruit Unified Sensor Library?*
- Range and rate are settable: `setRange(adxl34x_range_t)` — "Sets the g range"; `setDataRate(adxl3xx_dataRate_t)` — "Sets the data rate for the ADXL343 (controls power consumption)"; with matching `getRange()` / `getDataRate()`.
  Source: [Adafruit_ADXL343 Class Reference](https://adafruit.github.io/Adafruit_ADXL343/html/class_adafruit___a_d_x_l343.html), function table
- Chip supports, and the guide describes, hardware motion features: "Built-in motion detection features make tap, double-tap, activity, inactivity, and free-fall detection trivial. There's two interrupt pins, and you can map any of the interrupts independently to either of them."
  Source: [ADXL343 Breakout Learning Guide](https://learn.adafruit.com/adxl343-breakout-learning-guide?view=all), *Overview*
- Interrupt control in the Arduino API: `enableInterrupts(int_config)`, `mapInterrupts(int_config)` — "'Maps' the specific interrupt to either pin INT1 (bit=0), or pin INT2 (bit=1)"; `checkInterrupts()` — "Reading this register also clears or deasserts any currently active interrupt."
  Source: [Adafruit_ADXL343 Class Reference](https://adafruit.github.io/Adafruit_ADXL343/html/class_adafruit___a_d_x_l343.html), `enableInterrupts` / `mapInterrupts` / `checkInterrupts`
- Raw access + calibration helpers: `getX()` / `getY()` / `getZ()` / `getXYZ()`, `readRegister()` / `writeRegister()` / `read16()`, `getDeviceID()`, and trim offsets `getTrimOffsets()` / `setTrimOffsets()`.
  Source: same Class Reference page, function table
- Three constructor forms: I2C (`sensorID`), I2C on an alternate Wire bus (`sensorID, TwoWire*`), software SPI (`clock, miso, mosi, cs, sensorID`), and hardware SPI (`cs, SPIClass*, sensorID`).
  Source: same Class Reference page, constructor list
- Compatible with all Arduino architectures in the Arduino library index: "Unified driver for the ADXL343 Accelerometer… This library is compatible with **all** architectures."
  Source: [Adafruit ADXL343 | Arduino Documentation](https://docs.arduino.cc/libraries/adafruit-adxl343/), library entry

**Caveats**

- **README positions it as an I2C driver even though the chip / constructors support SPI:** "The ADXL343 is a digital accelerometer that supports both SPI and I2C mode… The Adafruit_ADXL343 driver takes advantage of I2C mode to reduce the total pin count required to use the sensor."
  Source: [GitHub — adafruit/Adafruit_ADXL343](https://github.com/adafruit/Adafruit_ADXL343), *README / About the ADXL343*
- **SPI mode is "not actively support[ed]" on the breakout (guide's wording):** "Please note that SPI mode is not actively support and the SPI pins are not all 5V safe and level shifted, so care will be required when using it!" — and the guide's technical list says "HW support for both I2C and SPI, although I2C is recommended for ease of use."
  Source: [ADXL343 Breakout Learning Guide](https://learn.adafruit.com/adxl343-breakout-learning-guide?view=all), *Pinout / CS pin* and *Technical Characteristics*
- **Only SCL / SDA (and CS) are 5 V-safe on the breakout; INT pins are not:** "This breakout board can be run on **3.3V** and **5V** systems, although only the **SCL** and **SDA** lines are 5V safe (other pins like INT will need to be manually level-shifted by you)" and "NOTE: Only CS, SCL and SDA are 5V safe on this board." This matters directly for the PDF's plan to use the interrupts for a low-power standby / wake-on-launch mode on a 5 V system.
  Source: same Learning Guide, *Pinout / Power Pins*
- **Only two I2C addresses:** default `0x53`; "If you set this pin high (to 3.3V), and reset, the I2C address will be updated to **0x1D**" (SDO / ALT ADDR pin).
  Source: same Learning Guide, *Pinout / SDO/ALT ADDR*
- **Guide naming trap:** the Arduino installation section is headed "The Adafruit_ADXL345 library can be installed using the Arduino Library Manager" and then tells you to search for "Adafruit ADXL343" and install the **Adafruit ADXL343** library. Install the 343-named library for this breakout.
  Source: same Learning Guide, *Arduino / Installation*
- **ADXL345 code reuse is "likely", not guaranteed:** "The ADXL343 is nearly identical in specifications to the ADXL345, and code written for the '345 will likely work on the '343 as-is."
  Source: same Learning Guide, *Overview*

### 3.3 Adafruit CircuitPython ADXL34x

**Features**

- One module for both chips, with event helpers, not just raw reads: "This module allows you to easily write Python code that reads the acceleration, taps, motion and more from the breakout."
  Source: [ADXL343 Breakout Learning Guide — Python and CircuitPython](https://learn.adafruit.com/adxl343-breakout-learning-guide/circuitpython.md), *Python and CircuitPython*
- Properties / methods in the API: `acceleration` (3-tuple in m/s²), `data_rate`, `range`, `raw_x/y/z`, `offset`; `enable_motion_detection(threshold=18)`, `enable_tap_detection(tap_count=1, threshold=20, duration=50, latency=20, window=255)`, `enable_freefall_detection(threshold=10, time=25)`, matching `disable_*` methods, and an `events` dict with `tap` / `motion` / `freefall` keys.
  Source: [API Reference — Adafruit ADXL34x Library](https://docs.circuitpython.org/projects/adxl34x/en/latest/api.html), `ADXL345` class (the driver class — see Caveat below)
- Wide rate / range enums: `DataRate.RATE_3200_HZ` down to `RATE_0_10_HZ`; `Range.RANGE_16_G / 8_G / 4_G / 2_G`.
  Source: same API Reference page, `DataRate` / `Range`
- Installable from PyPI: `pip3 install adafruit-circuitpython-adxl34x`.
  Source: [GitHub — adafruit/Adafruit_CircuitPython_ADXL34x](https://github.com/adafruit/Adafruit_CircuitPython_ADXL34x), *Installing from PyPI*

**Caveats**

- **In the CircuitPython API docs, `ADXL343` is literally documented as a stub:** "class adafruit_adxl34x.ADXL343… **Stub class for the ADXL343 3-axis accelerometer**", while the full documented driver class on the same page is `ADXL345` ("Driver for the ADXL345 3 axis accelerometer"). Check which class your installed version actually implements before relying on ADXL343-specific behaviour in Python.
  Source: [API Reference — Adafruit ADXL34x Library](https://docs.circuitpython.org/projects/adxl34x/en/latest/api.html), class list at top of page
- **I2C-only in CircuitPython:** both classes take only `(i2c, address=83)` — i.e. `0x53` — with no SPI constructor, unlike the Arduino driver.
  Source: same API Reference page, `ADXL343` / `ADXL345` signatures
- **Dependency:** "This driver depends on: Adafruit CircuitPython, Bus Device. Please ensure all dependencies are available on the CircuitPython filesystem." (Note: unlike the LSM6DS / LPS2X Python drivers, no Register dependency is listed here.)
  Source: [GitHub — adafruit/Adafruit_CircuitPython_ADXL34x](https://github.com/adafruit/Adafruit_CircuitPython_ADXL34x), *Dependencies*
- **Event parameters must be passed as keywords if you change them:** e.g. for free-fall, "If you wish to set them yourself rather than using the defaults, you must use keyword arguments: `accelerometer.enable_freefall_detection(time=30)`" — same note is repeated for motion and tap.
  Source: [API Reference — Adafruit ADXL34x Library](https://docs.circuitpython.org/projects/adxl34x/en/latest/api.html), `enable_freefall_detection` / `enable_motion_detection` / `enable_tap_detection`

---

## 4. LPS25HB (Adafruit 4530 — see §1 note on "4258")

### 4.1 Libraries used

| Library | Language / platform | Use it for |
|---|---|---|
| **Adafruit LPS2X** — `Adafruit_LPS25` in `Adafruit_LPS2X.h` | Arduino / C++ | Primary library for this breakout |
| **Adafruit CircuitPython LPS2X** — `adafruit_lps2x.LPS25` | CircuitPython / Python (Blinka) | Python driver; one library covers LPS22 **and** LPS25 |
| STM32duino LPS25HB — `LPS25HBSensor` | Arduino, ST-oriented | ST's Arduino library; I2C **or** SPI, lower-level API |
| Pololu LPS — `LPS` | Arduino | Alternative; raw pressure + altitude helpers, auto-detection |
| SparkFun LPS25HB Arduino Library — `LPS25HB` | Arduino, Qwiic-oriented | Alternative; deliberately "basic" — see Caveats |

### 4.2 Adafruit LPS2X — Arduino

**Features**

- Family library: "This is the Library for the LPS2X family of barometric pressure sensors for Arduino. Tested and works great with the Adafruit LPS2X Breakout Board."
  Source: [GitHub — adafruit/Adafruit_LPS2X](https://github.com/adafruit/Adafruit_LPS2X), *README*
- Sensor range / speed from the guide (this is the sensor + library example context): "The **LPS25** has a wide measurement range of **260-1260 hPa** with 24-bit pressure data measurements that can be read up to **25 times a second (Hz)**."
  Source: [Adafruit LPS25 and LPS22 Barometric Pressure and Temperature Sensors](https://learn.adafruit.com/adafruit-lps25-pressure-sensor?view=all), *Overview*
- Both buses in the API: `begin_I2C(i2c_addr, wire, sensor_id)`, `begin_SPI(cs_pin, theSPI, sensor_id)` (hardware SPI) and `begin_SPI(cs, sck, miso, mosi, sensor_id)` (software SPI).
  Source: [Adafruit_LPS25 Class Reference](https://adafruit.github.io/Adafruit_LPS2X/html/class_adafruit___l_p_s25.html), inherited `Adafruit_LPS2X` functions
- Unified-sensor readout: `getEvent(sensors_event_t *pressure, sensors_event_t *temp)`, plus `getPressureSensor()` / `getTemperatureSensor()` returning Unified Sensor objects.
  Source: same Class Reference page, function table
- Configurable data rate, power-down, and pressure-threshold interrupts: `setDataRate(lps25_rate_t)` / `getDataRate()`, `powerDown(bool)` — "Puts the sensor into power down mode, shutting the sensor down", `setPresThreshold(uint16_t hPa_delta)`, and `configureInterrupt(activelow, opendrain, pres_high, pres_low)` — "Configures the INT pin, by default it will output DRDY signal."
  Source: same Class Reference page, `Adafruit_LPS25` functions
- Software reset: `reset()` — "Performs a software reset initializing registers to their power on state."
  Source: same Class Reference page, `reset` row

**Caveats**

- **Dependencies are mandatory:** "Dependencies — Adafruit_BusIO, Adafruit_Sensor." The learning guide repeats the same install sequence (LPS2X, then BusIO, then Unified Sensor).
  Sources: [GitHub — adafruit/Adafruit_LPS2X](https://github.com/adafruit/Adafruit_LPS2X), *README / Dependencies*; [Adafruit LPS25 guide](https://learn.adafruit.com/adafruit-lps25-pressure-sensor?view=all), *Arduino / Library Installation*
- **README again understates the bus support:** GitHub README says only "This chip uses I2C to communicate, 2 pins are required to interface" — yet the API above provides `begin_SPI` in both hardware and software forms. Do not rule out SPI based on the README sentence alone.
  Source: [GitHub — adafruit/Adafruit_LPS2X](https://github.com/adafruit/Adafruit_LPS2X), *README*
- **You must pick the right subclass / example:** "The examples below show using an LPS25, however the LPS2X library can be used with either the LPS25 or LPS22. For the LPS22, use 'adafruit_lps22_test'" — i.e. for this sensor use the `adafruit_lps25_test` example and the `Adafruit_LPS25` class, not LPS22.
  Source: [Adafruit LPS25 guide](https://learn.adafruit.com/adafruit-lps25-pressure-sensor?view=all), *Arduino / Library Installation*

### 4.3 Adafruit CircuitPython LPS2X

**Features**

- Pressure + temperature as simple properties: usage example prints `lps.pressure` ("Pressure: %.2f hPa") and `lps.temperature` ("Temperature: %.2f C") from `adafruit_lps2x.LPS25(i2c)`.
  Source: [GitHub — adafruit/Adafruit_CircuitPython_LPS2X](https://github.com/adafruit/Adafruit_CircuitPython_LPS2X), *Usage Example*
- API properties: `pressure` — "The current pressure measurement in hPa"; `temperature` — "…in degrees Celsius"; `data_rate`; `reset()`; `initialize()` — "Configure the sensor with the default settings. For use after calling `reset()`"; and on LPS25, `enabled` — "Controls the power down state of the sensor. Setting to `False` will shut the sensor down."
  Source: [API Reference — Adafruit LPS2x Library](https://docs.circuitpython.org/projects/lps2x/en/latest/api.html), `LPS2X` / `LPS25` classes
- LPS25 rates are explicit and limited: `Rate.LPS25_SHUTDOWN`, `LPS25_RATE_1_HZ`, `LPS25_RATE_7_HZ`, `LPS25_RATE_12_5_HZ`, `LPS25_RATE_25_HZ`. (The higher 50 / 75 Hz rates on the same page belong to LPS22, not LPS25.)
  Source: same API Reference page, `Rate` table
- Installable from PyPI: `pip3 install adafruit-circuitpython-lps2x`.
  Source: [GitHub — adafruit/Adafruit_CircuitPython_LPS2X](https://github.com/adafruit/Adafruit_CircuitPython_LPS2X), *Installing from PyPI*

**Caveats**

- **I2C-only in CircuitPython:** the classes are `LPS2X(i2c_bus, address=93, chip_id=-1)` and `LPS25(i2c_bus, address=93)` — no SPI constructor, unlike the Arduino version.
  Source: [API Reference — Adafruit LPS2x Library](https://docs.circuitpython.org/projects/lps2x/en/latest/api.html), class signatures
- **Only two I2C addresses:** "Default is `0x5d` but will accept `0x5c` when the `SDO` pin is connected to Ground."
  Source: same API Reference page, `LPS25` parameters
- **Dependencies must be on the filesystem:** "This driver depends on: Adafruit CircuitPython, Bus Device, Register. Please ensure all dependencies are available on the CircuitPython filesystem."
  Source: [GitHub — adafruit/Adafruit_CircuitPython_LPS2X](https://github.com/adafruit/Adafruit_CircuitPython_LPS2X), *Dependencies*

### 4.4 Alternative Arduino libraries for LPS25HB

**STM32duino LPS25HB**

- Features: "Arduino library to support the LPS25HB 260-1260 hPa absolute digital output barometer… This sensor uses I2C or SPI to communicate." Values are read with `PressTemp.GetPressure(&pressure)` / `GetTemperature(&temperature)` after `begin()` + `Enable()`.
  Source: [GitHub — stm32duino/LPS25HB](https://github.com/stm32duino/LPS25HB), *README / API*
- Caveat — you must construct and begin the bus object yourself first: "For I2C it is then required to create a TwoWire interface before accessing to the sensors" (`TwoWire dev_i2c(...); dev_i2c.begin();`), and likewise a `SPIClass` for SPI.
  Source: same GitHub README, *API*
- Caveat — positioning: the PlatformIO / Arduino listings describe it as providing "Arduino support for the… LPS25HB **for STM32 boards**", even though the Arduino docs list wider architecture compatibility — verify your exact board/core before committing to it.
  Source: [stm32duino/STM32duino LPS25HB — PlatformIO Registry](https://registry.platformio.org/libraries/stm32duino/STM32duino%20LPS25HB), library description

**Pololu LPS**

- Features: "interfaces with ST LPS22DF, LPS25HB, LPS25H, and LPS331AP pressure sensors on Pololu boards… makes it simple to read the raw pressure data", with `readPressureMillibars()`, `readPressureRaw()` ("Returns a raw 24-bit pressure reading"), `readTemperatureC()`, and altitude helpers such as `pressureToAltitudeMeters(pressure)`.
  Sources: [LPS — Arduino Libraries listing](https://www.arduinolibraries.info/libraries/lps), library description; Pololu LPS README summary via [GitPlanet — lps-arduino](https://gitplanet.com/project/lps-arduino) / search result text for the Pololu repo https://github.com/pololu/lps-arduino, *library functions*
- Caveat: it is written for **Pololu carrier boards** and auto-detection (`ps.init()` can fail with "Failed to autodetect pressure sensor!") — it is not the Adafruit breakout's vendor library.
  Source: same Pololu README summary, *example / `init()`*

**SparkFun LPS25HB Arduino Library**

- Features: "An Arduino library that provides basic functions for getting pressure and temperature readings. Allows the user to support up to two sensors on a given I2C bus, and allows use of multiple I2C busses. Additional access to settings is available for advanced use."
  Source: [SparkFun LPS25HB Pressure Sensor Library — PlatformIO Registry](https://registry.platformio.org/libraries/sparkfun/SparkFun%20LPS25HB%20Pressure%20Sensor%20Library), library description (repo: https://github.com/sparkfun/SparkFun_LPS25HB_Arduino_Library)
- Caveat: by its own description it is **basic** and Qwiic-board-oriented ("Library for the SparkFun Qwiic Pressure Sensor - LPS25HB… Several basic functionality examples included") — not a full-featured driver.
  Source: same PlatformIO Registry page, library description

---

## 5. MS5611 (Embedded Adventures MOD-1009)

### 5.1 Libraries used

| Library | Language / platform | Use it for |
|---|---|---|
| **RobTillaart MS5611** | Arduino, **I2C only** | The MS5611 library in the Arduino Library Manager; most complete / maintained option found |
| RobTillaart **MS5611_SPI** | Arduino, **SPI only, experimental** | Only if you must use SPI — see the strong caveats below |
| jarzebski **Arduino-MS5611** | Arduino, I2C | Older, widely-copied alternative with a *different, incompatible API* — see Caveats |
| Embedded Adventures' own MOD-1009 code | C (`ms5611.c` / `ms5611.h`), not an Arduino library | Vendor-supplied source referenced in the MOD-1009 datasheet — see Caveats |

Vendor note: the MOD-1009 product page lists only "Datasheet: MOD-1009 (for v3)" as its document, with module facts "Sensor: MS5611 / Hackability: 100% (full datasheet, schematics available)" — no Arduino library is offered by Embedded Adventures for this module.
Source: [Embedded Adventures — MOD-1009 product page](http://www.embeddedadventures.com/barometric_pressure_sensor_module_mod-1009.html), product table

The MOD-1009 datasheet itself points at raw C source, not a library manager package: "Programming — See ms5611.c and ms5611.h available in the PicP…" (datasheet § Programming).
Source: [MOD-1009 HWv3 datasheet](https://www.embeddedadventures.com/datasheets/MOD-1009_hw_v3_doc_v2.pdf), *Programming* (text found via search of that PDF)

### 5.2 RobTillaart MS5611 — Arduino (I2C)

**Features**

- Purpose: "Arduino library (I2C) for MS5611 temperature and pressure sensor… The MS5611-01BA03 is a high resolution pressure and temperature sensor a.k.a GY-63. The high resolution is made possible by oversampling many times."
  Source: [GitHub — RobTillaart/MS5611](https://github.com/RobTillaart/MS5611), *README / Description*
- Read API with caching semantics: `read(bits)` / `read()`, `getTemperature()` (°C), `getPressure()` (mBar), `getPressurePascal()` — and "Subsequent calls will return the same value until **read()** is called again."
  Source: same GitHub README, *Interface MS5611 / Read*
- Oversampling is selectable, with a documented speed-vs-resolution trade-off table: OSR_ULTRA_LOW 256 → 0.065 mbar / 600 µs; OSR_LOW 512 → 0.042 / 1200 µs; OSR_STANDARD 1024 → 0.027 / 2300 µs; OSR_HIGH 2048 → 0.018 / 4600 µs; OSR_ULTRA_HIGH 4096 → 0.012 mbar / 9100 µs. "Default = backwards compatible" is OSR_ULTRA_LOW.
  Source: same GitHub README, *Interface MS5611 / Oversampling*
- Altitude helpers: `getAltitude(airPressure = 1013.25)` (metres), `getAltitudeFeet(...)`, and the inverse calibration helper `getSeaLevelPressure(pressure, altitude)`.
  Source: same GitHub README, *Interface MS5611 / Altitude*
- Runtime calibration offsets (runtime only): `setPressureOffset()` / `getPressureOffset()`, `setTemperatureOffset()` / `getTemperatureOffset()` — "The offset functions are added (0.3.6) to calibrate the sensor against e.g. a local weather station. This calibration can only be done runtime."
  Source: same GitHub README, *Interface MS5611 / Offset*
- 2nd-order compensation toggle: `setCompensation(bool flag = true)` / `getCompensation()` — default on; "Disabling the compensation will be slightly faster but you loose precision."
  Source: same GitHub README, *Interface MS5611 / 2nd order pressure compensation*
- Device identification from factory PROM: `getDeviceID()` — hashed calibration PROM values; "Note: this is not an official ID from the device / datasheet, it is made up from calibration data." Also `getManufacturer()`, `getSerialCode()`, `getProm()`, `getCRC()`, `getLastResult()`, `lastRead()`.
  Source: same GitHub README, *Interface MS5611 / DeviceID, getManufacturer, Develop functions, Misc*

**Caveats**

- **I2C only in this library:** "This library only implements the I2C interface. An experimental SPI version of the library can be found here — https://github.com/RobTillaart/MS5611_SPI"
  Source: same GitHub README, *Description*
- **Self-heating, and a hard 5 V warning (critical for airbrakes wiring):** "In some configurations especially when using SPI the sensor showed a self heating effect… Depending on the configuration self heating can be as low as 0.1 °C to as high as 10++ °C. **WARNING** One should **NOT** use 5V to control I2C address line, SPI select, or the protocol select line. This causes extreme heat build up > 10 °C. **One should only use 3V3 lines for these 'selection lines'.**" The README also advises a dedicated temperature sensor (e.g. DS18B20) for reliable ambient temperature, and notes the temperature offset compensation "is not expected to be linear over the full range".
  Source: same GitHub README, *Self heating problem*
- **Only two I2C addresses, and more sensors need a multiplexer (with a speed cost):** "The device address is 0x76 or 0x77 depending on the CSB/CSO pin." For more devices: "This is possible with an I2C multiplexer e.g. TCA9548… Drawback… it takes more administration in your code… This will slow down the access… switching between channels will slow down other devices too if they are behind the multiplexer."
  Source: same GitHub README, *Description / I2C / I2C multiplexing*
- **Some compatible devices read 50 % pressure unless you change the math mode:** "Note: Some device types will return only 50% of the pressure value. This is solved by calling **reset(1)** to select the math method used." Compatibility itself is hedged: "The library should be compatible with MS56XX, MS57xx and MS58xx devices (to be tested)."
  Source: same GitHub README, *Compatibility / 0.3.9 pressure math*
- **Breaking changes — old versions / old tutorials are obsolete:**
  - "0.3.0 breaking change — fixed math error so previous versions are **obsolete**. temperature is a float expressed in degrees Celsius. pressure is a float expressed in mBar."
  - "0.4.0 breaking change — refactored the Wire dependency… User has to call **Wire.begin()** (or equivalent)… before calling **ms5611.begin()**" (also repeated under *Base*: "One must call **Wire.begin()**… before calling **begin()**").
  - "0.5.0 Breaking change — As the getAltitude() function had a bug it is advised to use version 0.5.0 or up. Older versions are considered obsolete."
  Source: same GitHub README, *Release Notes (major)*
- **Error values are magic numbers, not exceptions / NaN:** "Return values of -9.99 or -999 indicate read error." (for temperature / pressure respectively, in context)
  Source: same GitHub README, *Interface MS5611 / Read*
- **Known platform quirk — Nano 33 BLE:** "The I2C/Wire library of the NANO 33 BLE does not see the device on the I2C bus… Adding a **wire->write(0x00)** in **isConnected()** fixes the problem, however more investigation is needed to understand the root cause."
  Source: same GitHub README, *Release Notes / 0.3.5 NANO 33 BLE*
- **Future / unfinished items are listed by the author himself:** "Must — update documentation, sync with MS5611_SPI library; Should — add / improve error handling."
  Source: same GitHub README, *Future*

### 5.3 RobTillaart MS5611_SPI — Arduino (SPI, experimental)

**Features**

- Same interface family as the I2C version, for SPI: "Arduino library (SPI) for MS5611 temperature and pressure sensor… This library only implements the SPI interface. It is based upon the 0.3.6 version of the I2C library" — with the same read / oversampling / offset / altitude / compensation API.
  Source: [GitHub — RobTillaart/MS5611_SPI](https://github.com/RobTillaart/MS5611_SPI), *Description / Interface MS5611*

**Caveats — do not use for flight data without testing**

- **Banner warning in the README:** "**WARNING: the MS5611 has problems with self heating when using SPI interface so use with care.**"
  Source: same GitHub README, *Description*
- **Explicitly experimental:** "## WARNING EXPERIMENTAL — Note: This library is under development. SPI communication works and reasonable values are read, at least for pressure. All SPI tests so far gave too high temperatures, some were rising slowly, others faster… the selection of SPI as protocol seems to cause internal heating."
  Source: same GitHub README, *WARNING EXPERIMENTAL*
- **Recorded test failures:** in the author's test table, "UNO SW SPI — fail — temperature is rising very fast (stopped) ==> DO NOT USE 5V" and "UNO HW SPI — fail — no data"; ESP32 variants are marked Y but with temperatures "too high (+3)" / "rising slowly"; Nano 33 rows are "not tested yet".
  Source: same GitHub README, *WARNING EXPERIMENTAL / test table, Note UNO, Note ESP32*
- **Same breaking-change pattern:** "0.4.0 — As the getAltitude() function had a bug it is advised to use version 0.4.0 or up. Older versions are considered obsolete." and "0.3.0… The user has to call **SPI.begin()** or equivalent before calling **AD.begin()**."
  Source: same GitHub README, *0.4.0 / 0.3.0 Breaking change*

### 5.4 jarzebski Arduino-MS5611 — older alternative

**Features**

- Minimal I2C driver: "MS5611 Barometric Pressure & Temperature Sensor Arduino Library. This library use I2C to communicate, 2 pins are required to interface."
  Source: [GitHub — jarzebski/Arduino-MS5611](https://github.com/jarzebski/Arduino-MS5611), *README*
- API used in its examples / tutorials is different from RobTillaart's: `ms5611.begin(MS5611_ULTRA_HIGH_RES)`, `readTemperature()`, `readPressure()` (Pa in the tutorial outputs), `getAltitude(realPressure)`, `getAltitude(realPressure, referencePressure)` for relative altitude, and `getSeaLevel(realPressure, altitude)`.
  Source: example code adapted from this library in [ESP8266 / MS5611 tutorial](http://www.esp8266learning.com/esp8266-and-ms5611-barometric-pressure-sensor-example.php), *example code* — "This example is adapted from the https://github.com/jarzebski/Arduino-MS5611 library"

**Caveats**

- **Same header name, incompatible API:** both this library and RobTillaart's are included as `#include <MS5611.h>` / `#include <Wire.h>` in examples, but their constructors and method names differ (`begin(ULTRA_HIGH_RES)` + `readPressure()` vs `begin()` + `read()` + `getPressure()`). Installing both, or pasting code from one into a project using the other, will not compile / will not behave the same. Pick one per project.
  Sources: [GitHub — jarzebski/Arduino-MS5611](https://github.com/jarzebski/Arduino-MS5611), *README*; [GitHub — RobTillaart/MS5611](https://github.com/RobTillaart/MS5611), *Interface MS5611*
- **Sparse README in the repo itself:** the GitHub README is only the title, the one-line I2C note, and a link to the author's (Polish) project page — there is no feature list, caveat list, or API reference in the repo README. Documentation risk is higher than with the RobTillaart library.
  Source: [GitHub — jarzebski/Arduino-MS5611](https://github.com/jarzebski/Arduino-MS5611), *README* (entire README, as fetched 2026-10-30)
- **Age / activity signal:** the repo metadata as fetched shows it was created 2014-05-14, with 8 commits and 1 release on its default `dev` branch — versus RobTillaart MS5611 (created 2020-06-21, 40 commits, 19 releases). Treat the jarzebski library as legacy.
  Source: [GitHub — jarzebski/Arduino-MS5611](https://github.com/jarzebski/Arduino-MS5611), *Repository details*; [GitHub — RobTillaart/MS5611](https://github.com/RobTillaart/MS5611), *Repository details*

---

## 6. Cross-cutting caveats for the airbrakes design (dissimilar redundancy)

These follow directly from the sourced caveats above — they are the items most likely to bite this specific project:

1. **Every family gives you only 2 I2C addresses per bus.**
   LSM6DS3TR-C: 0x6A / 0x6B — [guide](https://learn.adafruit.com/adafruit-lsm6ds3tr-c-6-dof-accel-gyro-imu?view=all), *Address Jumper*.
   ADXL343: 0x53 / 0x1D — [guide](https://learn.adafruit.com/adxl343-breakout-learning-guide?view=all), *SDO/ALT ADDR*.
   LPS25HB: 0x5D / 0x5C — [CircuitPython LPS2X API](https://docs.circuitpython.org/projects/lps2x/en/latest/api.html), *LPS25 parameters*.
   MS5611: 0x76 / 0x77 — [RobTillaart MS5611](https://github.com/RobTillaart/MS5611), *Description*.
   A third same-type sensor on the same bus needs a multiplexer (MS5611 README explicitly warns this slows access — see §5.2) or a second bus / SPI.
2. **CircuitPython drivers are I2C-only for all three Adafruit sensors + there is no Adafruit MS5611 CircuitPython driver in this set at all.** If the flight computer is Python, SPI is only available via the Arduino/C++ drivers (LSM6DS, ADXL343 constructors, LPS2X `begin_SPI`) — sources in §2.3, §3.3, §4.3.
3. **Interrupts are the whole point for the ADXL343 standby idea in the PDF, and they are the least level-shifted part of that breakout.** Only CS/SCL/SDA are 5 V-safe; INT pins need manual level shifting on 5 V systems — [ADXL343 guide](https://learn.adafruit.com/adxl343-breakout-learning-guide?view=all), *Pinout / Power Pins*.
4. **MS5611 + SPI is a known-bad combination in the only SPI library found.** Use the I2C RobTillaart library (≥ 0.5.0) for flight pressure data, keep selection lines at 3.3 V, and do not mix jarzebski code with RobTillaart code — sources in §5.2–§5.4.
5. **Adafruit Arduino libraries are deliberately simple; advanced chip features live elsewhere.** LSM6DS FIFO / tap → ST library (§2.2). LPS25HB FIFO modes exist on the chip (Pololu carrier page notes "several FIFO operating modes") but the Adafruit LPS2X API surface in §4.2 exposes data-rate / threshold-interrupt / power-down, not FIFO management.
   Source for the chip-level FIFO note: [Pololu LPS25HB carrier](https://mgsl.in/products/pololu-2867) (Pololu product text, via search result) — "several FIFO operating modes, and a programmable external interrupt signal"
6. **Name collisions are everywhere in this stack:** `Adafruit_LSM6DS` ≠ `Arduino_LSM6DS3` (§2.4); `Adafruit ADXL343` ≠ the guide's stray "Adafruit_ADXL345" heading (§3.2); LPS25 example ≠ LPS22 example (§4.2); RobTillaart `MS5611.h` ≠ jarzebski `MS5611.h` (§5.4). Pin the exact library name + version in the project, not just the sensor name.

---

## 7. Quick recommendation (from the sources above)

| Sensor | Use | Avoid / be careful |
|---|---|---|
| LSM6DS3TR-C | Adafruit LSM6DS (Arduino, `Adafruit_LSM6DS3TRC`) or Adafruit CircuitPython LSM6DS (`lsm6ds3trc` subclass) | Arduino_LSM6DS3 (built-in-board only, fixed 104 Hz / ±4 g / ±2000 dps); expecting FIFO/tap from Adafruit's library |
| ADXL343 | Adafruit ADXL343 (Arduino) for interrupt-driven wake-up; Adafruit CircuitPython ADXL34x for Python prototyping | SPI mode on the breakout (not actively supported, not fully 5 V-safe); 5 V use of INT pins without level shifting |
| LPS25HB | Adafruit LPS2X (Arduino, `Adafruit_LPS25`) or Adafruit CircuitPython LPS2X (`LPS25`) | Ordering/coding by the PDF's "4258" — actual breakout is 4530; LPS22 examples/rates (50/75 Hz are LPS22-only in Python) |
| MS5611 (MOD-1009) | RobTillaart MS5611 **I2C**, version ≥ 0.5.0, 3.3 V selection lines, call `Wire.begin()` first | MS5611_SPI (experimental, self-heating, recorded UNO failures); jarzebski library mixed into the same project; any version < 0.5.0 (`getAltitude` bug) |

---

## 8. Source index — exactly where everything above was found

Product / identification
- LSM6DS3TR-C guide: https://learn.adafruit.com/adafruit-lsm6ds3tr-c-6-dof-accel-gyro-imu?view=all
- LSM6DS3TR-C product: https://www.adafruit.com/product/4503
- ADXL343 guide: https://learn.adafruit.com/adxl343-breakout-learning-guide?view=all
- LPS25 guide: https://learn.adafruit.com/adafruit-lps25-pressure-sensor?view=all
- MOD-1009 product: http://www.embeddedadventures.com/barometric_pressure_sensor_module_mod-1009.html
- MOD-1009 datasheet: https://www.embeddedadventures.com/datasheets/MOD-1009_hw_v3_doc_v2.pdf

LSM6DS
- Arduino repo: https://github.com/adafruit/Adafruit_LSM6DS
- Arduino API: https://adafruit.github.io/Adafruit_LSM6DS/html/class_adafruit___l_s_m6_d_s3_t_r_c.html
- CircuitPython repo: https://github.com/adafruit/Adafruit_CircuitPython_LSM6DS
- CircuitPython API: https://docs.circuitpython.org/projects/lsm6dsox/en/latest/api.html
- Look-alike library repo: https://github.com/arduino-libraries/Arduino_LSM6DS3

ADXL343 / ADXL34x
- Arduino repo: https://github.com/adafruit/Adafruit_ADXL343
- Arduino API: https://adafruit.github.io/Adafruit_ADXL343/html/class_adafruit___a_d_x_l343.html
- Arduino library listing: https://docs.arduino.cc/libraries/adafruit-adxl343/
- CircuitPython repo: https://github.com/adafruit/Adafruit_CircuitPython_ADXL34x
- CircuitPython API: https://docs.circuitpython.org/projects/adxl34x/en/latest/api.html

LPS25HB / LPS2X
- Arduino repo: https://github.com/adafruit/Adafruit_LPS2X
- Arduino API: https://adafruit.github.io/Adafruit_LPS2X/html/class_adafruit___l_p_s25.html
- CircuitPython repo: https://github.com/adafruit/Adafruit_CircuitPython_LPS2X
- CircuitPython API: https://docs.circuitpython.org/projects/lps2x/en/latest/api.html
- STM32duino repo: https://github.com/stm32duino/LPS25HB
- Pololu repo: https://github.com/pololu/lps-arduino
- SparkFun repo: https://github.com/sparkfun/SparkFun_LPS25HB_Arduino_Library

MS5611
- RobTillaart I2C repo: https://github.com/RobTillaart/MS5611
- RobTillaart SPI repo: https://github.com/RobTillaart/MS5611_SPI
- jarzebski repo: https://github.com/jarzebski/Arduino-MS5611

*Checked 2026-10-30. Library APIs and READMEs change — re-check the repo / API page for the exact version you pin in the flight code.*
