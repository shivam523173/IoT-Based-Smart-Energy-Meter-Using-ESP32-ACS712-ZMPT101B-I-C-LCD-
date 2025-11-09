# IoT-Based Smart Energy Meter Using ESP32 (ACS712 + ZMPT101B + I²C LCD)

An ESP32-based meter that measures **Vrms, Irms, real power (W), power factor, and energy (Wh)** using an **ACS712 current sensor** and a **ZMPT101B voltage sensor**. Results are displayed on a **16×2 I²C LCD** and printed to Serial. The sketch includes DC-offset removal, a 1-second RMS window, and a small **phase trim** for accurate real power.

## 🧩 Hardware
- ESP32 DevKit
- ACS712 current sensor module → **GPIO35**
- ZMPT101B voltage sensor module → **GPIO34**
- 16×2 I²C LCD (PCF8574 backpack) → **SDA=21, SCL=22**
- Isolated mains interface (ZMPT module is required; **do not** wire mains directly)
- Jumper wires, 5 V USB power

> **Safety:** Use only isolated sensor modules (ACS712/ZMPT). Keep mains on the sensor side, never on the breadboard/ESP32.

## 🔌 Wiring (ESP32)
- **ACS712 AO → GPIO35**, Vcc → 5 V, GND → GND
- **ZMPT101B AO → GPIO34**, Vcc → 5 V (or 3.3–5 V per module), GND → GND
- **LCD I²C → SDA 21, SCL 22** (Address commonly `0x27` or `0x3F`)

## 📚 Libraries
- `LiquidCrystal_I2C` (by Frank de Brabander or compatible)
- `Wire` (built-in)

Install via Arduino IDE → Tools → Manage Libraries.

## ⚙️ Calibration
Edit these in the sketch:
```cpp
float I_CAL       = 0.89f;   // A per volt for ACS712 path
float V_CAL       = 190.0f;  // V per volt for ZMPT path
float PHASE_SHIFT = 0.015f;  // 0.00..0.05 small lag on current
