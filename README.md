# AZ/EL Antenna Rotator Controller — Arduino Stepper Motor

Arduino firmware for a two-axis antenna rotator using stepper motors, the EasyComm II protocol, and manual control via rotary encoders.

Based on the original design by **Viorel Racoviteanu (YO3RAK)**  
https://www.youtube.com/@racov | https://racov.ro

---

## Features

- Azimuth (0–359°) and Elevation (0–90°) control
- EasyComm II serial protocol — compatible with **PstRotator**, Orbitron, GPredict, Ham Radio Deluxe
- TB6600 / A4988 bipolar stepper motor driver support (step/direction interface)
- Manual positioning via two rotary encoders (AZ / EL)
- End-stop safety switches for both axes
- Auto-home to zero at startup
- 16×2 I2C LCD display (real position + command position)
- **0.1° command resolution** via serial (EasyComm II)

---

## Hardware

| Component | Description |
|---|---|
| Arduino | Uno (ATmega328P) or Mega |
| Motor Driver | TB6600 (or A4988) — step/direction mode |
| Display | 16×2 I2C LCD, address `0x27` |
| Encoders | 2× rotary encoder with push button |
| End stops | 4× limit switch (AZ low/high, EL low/high) |

### Pin Assignment

| Signal | Pin |
|---|---|
| AZ Step | D8 |
| AZ Dir | D9 |
| AZ Enable | D12 |
| EL Step | D10 |
| EL Dir | D11 |
| EL Enable | D13 |
| AZ Encoder A | D2 (interrupt) |
| AZ Encoder B | D4 |
| AZ Encoder Button | D5 |
| EL Encoder A | D3 (interrupt) |
| EL Encoder B | D6 |
| EL Encoder Button | D7 |
| AZ End Stop Low | A3 |
| AZ End Stop High | A2 |
| EL End Stop Low | A1 |
| EL End Stop High | A0 |
| LCD SDA | A4 |
| LCD SCL | A5 |

---

## Configuration

Adjust these constants at the top of the sketch to match your motors and gearbox:

```cpp
const long AzPulsPerTurn = 8140;   // stepper pulses per 360° azimuth
const long ElPulsPerTurn = 2050;   // stepper pulses per 90° elevation
const int  AzPark        = 120;    // azimuth parking position
const int  ElPark        = 0;      // elevation parking position
const int  Speed         = 500;    // max speed (steps/s)
```

> The acceleration is automatically set to `0.8 × Speed`.

---

## Libraries Required

Install via Arduino IDE Library Manager:

- [AccelStepper](https://www.arduinolibraries.info/libraries/accel-stepper)
- [LiquidCrystal_I2C](https://www.arduinolibraries.info/libraries/liquid-crystal-i2-c)
- [elapsedMillis](https://www.arduinolibraries.info/libraries/elapsed-millis)
- `Wire.h` (built-in)

---

## Serial Protocol (EasyComm II)

Baud rate: **9600**, timeout: 50 ms

| Command | Description | Example |
|---|---|---|
| `AZxxx.x` | Set azimuth target | `AZ180.5` |
| `ELxxx.x` | Set elevation target | `EL45.2` |
| `AZ EL` | Query current position | → `+180.4 45.2` |

The controller accepts fractional degrees (one decimal place) in both commands and responses.

---

## Manual Control

- **AZ encoder button** — enters manual positioning mode (display shows `*`)
  - Rotate encoder → changes AZ command in 1° steps
  - Rotate EL encoder simultaneously → changes EL command in 1° steps
  - Press button again → exits manual mode and executes the move
- **EL encoder button** — immediately sets parking position (AzPark / ElPark)

---

## LCD Display

```
Azm.NNN°=Cd.NNN°
Elv. NN°=Cd. NN°
```

- `Azm` / `Elv` — real measured position
- `Cd.` — current command target
- Status column (position 8): `→` `←` `↑` `↓` moving, `=` reached, `*` manual mode, `%` parking

> The display always shows integer degrees. Internally, positions are stored as `float` for 0.1° EasyComm resolution.

---

## Changes vs. Original (0.1° Resolution Update)

The original firmware by YO3RAK used `int` for command positions, limiting resolution to 1°.
This version upgrades to `float` throughout to support 0.1° command resolution via EasyComm II.

### Summary of changes

| # | Location | Change |
|---|---|---|
| 1 | Variables (l. 106–109) | `ComAzim`, `ComElev`, `OldComAzim`, `OldComElev`: `int` → `float` |
| 2 | Variable (l. 129) | `increment`: `int` → `float` (reserved for future use) |
| 3 | Encoder logic (l. 375–388) | `+= increment` replaced by `floor()+1` / `ceil()-1` to snap to integer degrees, preventing float drift |
| 4 | Azimuth wrap (l. 377) | `% 360` → `fmod(..., 360.0)` — `%` is not defined for `float` in C++ |
| 5 | `constrain()` calls | Integer bounds `0, 359` / `0, 90` → float bounds `0.0, 359.0` / `0.0, 90.0` |
| 6 | Serial parser AZ (l. 467–475) | Added `bool azDotSeen` flag; parser now accepts one decimal point in azimuth input |
| 7 | Serial parser EL (l. 487–495) | Same for elevation input (`bool elDotSeen`) |
| 8 | Conversion (l. 502, 508) | `Azimuth.toInt()` / `Elevation.toInt()` → `.toFloat()` |
| 9 | `DisplValue()` calls | All calls with `ComAzim` / `ComElev` wrapped in `round()` — display function expects `int` |
| 10 | Position response (l. 517–519, 534–536) | Replaced `String(round(...))+".0"` with `dtostrf()` for true 1-decimal output (e.g. `+180.4 45.2`) — `sprintf("%f")` does not work on AVR without extra linker flags |

### Why `floor()`/`ceil()` for encoder increments?

When PstRotator sets a fractional target like `AZ180.4`, the internal `ComAzim` is `180.4`.
A naive `+= 1.0` would produce `181.4`, `182.4`, ... — non-integer values from manual control.

Using `floor(ComAzim) + 1.0` snaps the first step to the next whole degree:

```
180.4 → up  → floor(180.4) + 1.0 = 181.0 ✓
180.4 → dn  → ceil(180.4)  - 1.0 = 180.0 ✓
180.0 → up  → floor(180.0) + 1.0 = 181.0 ✓
180.0 → dn  → ceil(180.0)  - 1.0 = 179.0 ✓
```

### Why `dtostrf()` for the serial response?

On AVR (Arduino Uno / Mega), `sprintf()` with `%f` is disabled by default and produces `?` or empty output. `dtostrf()` is the AVR-native float-to-string function:

```cpp
char azBuf[8], elBuf[6];
dtostrf(round(TruAzim * 10.0) / 10.0, 5, 1, azBuf);  // "180.4"
dtostrf(round(TruElev * 10.0) / 10.0, 4, 1, elBuf);  // "45.2"
ComputerWrite = "+" + String(azBuf) + " " + String(elBuf);
```

---

## Memory (Arduino Uno)

The float conversion adds only **+8 bytes SRAM** (4 × `float` instead of 4 × `int`).  
The main memory risk is the `String` class (heap fragmentation on AVR), which was already present in the original firmware. No new `String` objects are added by these changes.

After compiling, check the RAM usage reported by the Arduino IDE. On Uno (2 KB SRAM), keep usage below ~70%.

---

## License

Original code by Viorel Racoviteanu (YO3RAK) — see source file header.  
Modifications (0.1° resolution) provided as-is, no warranty.
