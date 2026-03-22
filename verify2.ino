#include <Arduino.h>
#include <math.h>

#if defined(ESP32)
  #include <esp32-hal-adc.h>
#endif

// ============================================================================
// Hardware / circuit assumptions
//   chargePin    -> capacitor through known charge resistor
//   dischargePin -> capacitor through known discharge resistor to GND
//   measurePin   -> capacitor node ADC input
// ============================================================================

static constexpr uint8_t chargePin    = 5;
static constexpr uint8_t dischargePin = 18;
static constexpr uint8_t measurePin   = 34;

// Supply / ADC
static constexpr float V_SUPPLY_V = 3.300f;   // If your rail differs, adjust or measure it.
static constexpr uint8_t ADC_BITS  = 12;
static constexpr uint16_t ADC_MAX   = (1u << ADC_BITS) - 1;

// Known external resistors in the test path
static constexpr float R_CHARGE_OHMS    = 10000.0f;
static constexpr float R_DISCHARGE_OHMS = 10000.0f;

// Thresholds for exponential fitting
static constexpr float CHARGE_STOP_FRACTION    = 0.95f;  // stop when near plateau
static constexpr float DISCHARGE_STOP_V        = 0.050f; // stop when near zero
static constexpr float MIN_FIT_SPAN_FRACTION   = 0.10f;  // exclude extreme early/late samples
static constexpr float MAX_FIT_SPAN_FRACTION   = 0.90f;

struct Sample {
  uint32_t t_us;
  float v;
};

struct FitResult {
  bool ok = false;
  float tau_us = NAN;
  float v0 = NAN;
  float vinf = NAN;
  uint16_t usedPoints = 0;
};

struct MeasurementResult {
  bool ok = false;
  float cCharge_uF = NAN;
  float cDischarge_uF = NAN;
  float leakOhms = NAN;
  const char* status = "UNKNOWN";
};

static constexpr size_t MAX_SAMPLES = 500;
static Sample samples_buf[MAX_SAMPLES];

// ---------------------------------------------------------------------------
// ADC helper
// ---------------------------------------------------------------------------

static float readVoltageOnce()
{
#if defined(ESP32)
  uint32_t mv = analogReadMilliVolts(measurePin);
  return mv / 1000.0f;
#else
  uint16_t raw = analogRead(measurePin);
  return (raw * V_SUPPLY_V) / float(ADC_MAX);
#endif
}

// ---------------------------------------------------------------------------
// Pin control
// ---------------------------------------------------------------------------

static void releaseAllPins()
{
  pinMode(chargePin, INPUT);
  pinMode(dischargePin, INPUT);
}

static void startCharge()
{
  pinMode(dischargePin, INPUT);
  pinMode(chargePin, OUTPUT);
  digitalWrite(chargePin, HIGH);
}

static void stopCharge()
{
  pinMode(chargePin, INPUT);
  digitalWrite(chargePin, LOW);
}

static void startDischarge()
{
  pinMode(chargePin, INPUT);
  pinMode(dischargePin, OUTPUT);
  digitalWrite(dischargePin, LOW);
}

static void stopDischarge()
{
  pinMode(dischargePin, INPUT);
}

// ---------------------------------------------------------------------------
// Auto-ranging sample capture
// ---------------------------------------------------------------------------

static bool captureCurve(bool charging, Sample *samples, size_t &count)
{
  count = 0;
  if (charging) startCharge(); else startDischarge();

  const uint32_t t0 = micros();
  uint32_t sampleIntervalUs = 10;

  // Fast check to decide initial interval
  if (charging) {
      delayMicroseconds(500);
      if (readVoltageOnce() < 0.1 * V_SUPPLY_V) {
          sampleIntervalUs = 1000;
      }
  }

  while (count < MAX_SAMPLES) {
    const uint32_t now = micros();
    const float v = readVoltageOnce();
    samples[count++] = { now - t0, v };

    if (charging) {
      if (v >= V_SUPPLY_V * CHARGE_STOP_FRACTION) break;
    } else {
      if (v <= DISCHARGE_STOP_V) break;
    }

    if ((now - t0) > 60000000) break; // 60s timeout

    // Adaptive interval expansion if we're running out of buffer space
    if (count > 400) {
        sampleIntervalUs *= 2;
    }

    uint32_t targetNext = micros() + sampleIntervalUs;
    while (micros() < targetNext) yield();
  }

  if (charging) stopCharge(); else stopDischarge();
  return count >= 10;
}

// ---------------------------------------------------------------------------
// Log-linear fitting for exponential curve
// ---------------------------------------------------------------------------

static FitResult fitCurve(const Sample *s, size_t n, bool isRise)
{
  FitResult r;
  if (n < 10) return r;

  const float v0 = s[0].v;
  float vinf = isRise ? V_SUPPLY_V : 0.0f;

  // Refine vinf from the plateau if it exists (indicates leakage)
  if (isRise && (s[n-1].v < 0.99f * V_SUPPLY_V)) {
       float plateau = 0;
       int k = (n > 5) ? 5 : (int)n;
       for (int i=0; i<k; i++) plateau += s[n-1-i].v;
       vinf = plateau / k;
  }

  const float span = isRise ? (vinf - v0) : (v0 - vinf);
  if (span < 0.05f) return r;

  double sumX = 0, sumY = 0, sumXX = 0, sumXY = 0;
  size_t used = 0;

  for (size_t i = 0; i < n; ++i) {
    const float frac = isRise ? (s[i].v - v0) / span : (s[i].v - vinf) / span;
    if (frac <= MIN_FIT_SPAN_FRACTION || frac >= MAX_FIT_SPAN_FRACTION) continue;

    const float ratio = isRise ? (vinf - s[i].v) / (vinf - v0) : (s[i].v - vinf) / (v0 - vinf);
    if (!(ratio > 0.0f)) continue;

    const double x = double(s[i].t_us);
    const double y = log(double(ratio));

    sumX  += x;
    sumY  += y;
    sumXX += x * x;
    sumXY += x * y;
    used++;
  }

  if (used < 5) return r;

  const double denom = used * sumXX - sumX * sumX;
  if (fabs(denom) < 1e-12) return r;

  const double slope = (used * sumXY - sumX * sumY) / denom;
  if (!(slope < 0.0)) return r;

  r.ok = true;
  r.tau_us = float(-1.0 / slope);
  r.v0 = v0;
  r.vinf = vinf;
  r.usedPoints = (uint16_t)used;
  return r;
}

// ---------------------------------------------------------------------------
// Main measurement routine
// ---------------------------------------------------------------------------

static MeasurementResult measureCapacitor()
{
  MeasurementResult out;
  size_t n;

  // Fully discharge first
  startDischarge();
  delay(2000);
  stopDischarge();

  // 1. Charge cycle
  if (captureCurve(true, samples_buf, n)) {
    FitResult res = fitCurve(samples_buf, n, true);
    if (res.ok) {
        out.cCharge_uF = (res.tau_us * 1e-6f / R_CHARGE_OHMS) * 1e6f;
        out.ok = true;
        out.status = "OK";

        // Leakage estimate from plateau voltage
        if (res.vinf < 0.995f * V_SUPPLY_V) {
            out.leakOhms = R_CHARGE_OHMS * res.vinf / (V_SUPPLY_V - res.vinf);
        } else {
            out.leakOhms = INFINITY;
        }
    } else {
        out.status = "CHARGE_FIT_FAIL";
    }
  } else {
      out.status = "CHARGE_CAPTURE_FAIL";
  }

  // 2. Discharge cycle (even if charge failed, attempt to measure what's there)
  if (captureCurve(false, samples_buf, n)) {
    FitResult res = fitCurve(samples_buf, n, false);
    if (res.ok) {
        out.cDischarge_uF = (res.tau_us * 1e-6f / R_DISCHARGE_OHMS) * 1e6f;
    }
  }

  return out;
}

void setup()
{
  Serial.begin(115200);
  releaseAllPins();
#if defined(ESP32)
  analogReadResolution(ADC_BITS);
  analogSetPinAttenuation(measurePin, ADC_11db);
#endif
  Serial.println("cap_uF_charge,cap_uF_discharge,leak_ohm,status");
}

void loop()
{
  const MeasurementResult m = measureCapacitor();
  Serial.print(m.cCharge_uF, 6); Serial.print(",");
  Serial.print(m.cDischarge_uF, 6); Serial.print(",");
  Serial.print(m.leakOhms, 2); Serial.print(",");
  Serial.println(m.status);

  delay(2000);
}
