/****************************************************
  ESP32 Smart Energy Meter
  Sensors: ACS712 (current) on GPIO35, ZMPT101B (voltage) on GPIO34
  Display: 16x2 I2C LCD on SDA=21, SCL=22 (0x27 or 0x3F)
  Computes: Vrms, Irms, Real Power P (W), PF, Energy (Wh)

  SAFETY: Use ONLY isolated sensor modules (ACS712 / ZMPT101B).
  Never expose mains to the ESP32 or breadboard directly.

  Author: you (packaged by ChatGPT for GitHub)
****************************************************/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

// ---------- Pins ----------
#define PIN_I   35   // ACS712 analog
#define PIN_V   34   // ZMPT101B analog

// ---------- LCD ----------
#define LCD_ADDR 0x27     // change to 0x3F if needed
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

// ---------- ADC / Sampling ----------
static const float ADC_REF_V = 3.3f;
static const int   ADC_BITS  = 12;
static const int   ADC_MAX   = (1 << ADC_BITS) - 1; // 4095
static const int   FS_HZ     = 4000;   // 4 kHz sampling
static const int   WIN_MS    = 1000;   // 1 second RMS window
static const unsigned long TS_US = 1000000UL / FS_HZ;

// ---------- Calibration (tune these!) ----------
// Convert HPF output-volts -> real units (A / V).
// 1) Run, read Vrms/Irms, compare to multimeter/mains (≈230 Vrms).
// 2) Adjust V_CAL and I_CAL until readings match.
float I_CAL       = 0.89f;    // A per 1.0V (ACS712 module dependent)
float V_CAL       = 190.0f;   // V per 1.0V (ZMPT module/trim pot dependent)

// Phase tweak (shift current vs voltage to improve real power)
// 0.00..0.05 typical. Increase if PF looks low at resistive load.
float PHASE_SHIFT = 0.015f;

// ---------- Energy ----------
double energy_Wh = 0.0;

// ---------- Filters ----------
struct HPF {
  // One-pole high-pass: y[n] = a*(y[n-1] + x[n] - x[n-1])
  float a = 0.995f, y = 0.0f, x_prev = 0.0f;
  inline float step(float x) {
    y = a * (y + x - x_prev);
    x_prev = x;
    return y;
  }
};

struct FracDelay {
  // Simple fractional delay (first-order IIR blend)
  // y = (1-k)*x + k*y;  k~0..0.2 behaves like a tiny lag.
  float k = 0.0f, y = 0.0f;
  inline float step(float x) {
    y = (1.0f - k) * x + k * y;
    return y;
  }
};

HPF hpf_i, hpf_v;
FracDelay dly_i;

// ---------- UI helpers ----------
static inline void lcdPrint2d(int v) { if (v < 10) lcd.print('0'); lcd.print(v); }

// ---------- Display ----------
void showLCD(float Vrms, float Irms, float P, float PF, double E_Wh) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("V:"); lcd.print(Vrms, 0);
  lcd.print(" I:"); lcd.print(Irms, 2);

  lcd.setCursor(0, 1);
  lcd.print("P:"); lcd.print(P, 1);
  lcd.print(" PF:"); lcd.print(PF, 2);

  // Briefly flash energy every few cycles (optional):
  // Commented to keep a steady display; use Serial for energy log.
}

// ---------- Core measurement ----------
void measurePower() {
  const float count_to_volt = ADC_REF_V / ADC_MAX;

  double sum_v2 = 0.0, sum_i2 = 0.0, sum_vi = 0.0;
  unsigned long start_ms = millis();
  unsigned long t_prev   = micros();
  unsigned long n = 0;

  // Tie delay coefficient to PHASE_SHIFT
  dly_i.k = PHASE_SHIFT;

  while ((millis() - start_ms) < WIN_MS) {
    unsigned long now = micros();
    if (now - t_prev < TS_US) continue;
    t_prev += TS_US;

    int ai = analogRead(PIN_I);
    int av = analogRead(PIN_V);

    // Convert counts -> volts at ADC pin
    float v_adc_i = ai * count_to_volt;
    float v_adc_v = av * count_to_volt;

    // Remove mid-rail DC (ACS/ZMPT centers near ~1.65V)
    float i_hpf = hpf_i.step(v_adc_i - 1.65f);
    float v_hpf = hpf_v.step(v_adc_v - 1.65f);

    // Apply calibration to get real units
    float i_a = i_hpf * I_CAL;    // Amps
    float v_v = v_hpf * V_CAL;    // Volts

    // Phase tuning on current (small lag improves P accuracy)
    float i_p  = dly_i.step(i_a);

    // Accumulate
    sum_i2 += (double)i_p * (double)i_p;
    sum_v2 += (double)v_v * (double)v_v;
    sum_vi += (double)v_v * (double)i_p;
    n++;
  }

  if (n == 0) return;

  float Irms = sqrt(sum_i2 / n);
  float Vrms = sqrt(sum_v2 / n);
  float P    = sum_vi / n;                   // Real power (W), can be +- for direction
  float S    = Vrms * Irms + 1e-6f;         // Apparent power (VA)
  float PF   = P / S;                        // Power factor
  // Clamp PF into [-1,1] for display sanity
  if (PF > 1.0f) PF = 1.0f;
  if (PF < -1.0f) PF = -1.0f;

  // Accumulate energy (Wh): P[W] * (windowHours)
  energy_Wh += (double)P * (double)WIN_MS / 3600000.0;

  // -------- Output --------
  showLCD(Vrms, Irms, P, PF, energy_Wh);
  Serial.printf("Vrms=%.1f V  Irms=%.3f A  P=%.2f W  PF=%.3f  E=%.4f Wh\n",
                Vrms, Irms, P, PF, energy_Wh);
}

void setup() {
  Serial.begin(115200);
  delay(100);

  // ADC config
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_I, ADC_11db); // allows ~0-3.6V input range
  analogSetPinAttenuation(PIN_V, ADC_11db);

  // LCD
  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Smart Energy Meter");
  lcd.setCursor(0, 1); lcd.print("Init...");
  delay(1200);
  lcd.clear();

  // Initial filter states
  hpf_i.a = 0.995f;
  hpf_v.a = 0.995f;
  dly_i.k = PHASE_SHIFT;

  Serial.println("ESP32 Smart Energy Meter Started");
  Serial.println(">> Calibrate V_CAL to ~230 Vrms and I_CAL to clamp meter.");
}

void loop() {
  measurePower();   // ~1s window by default
}
