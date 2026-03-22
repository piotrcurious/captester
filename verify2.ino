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

// Sampling / timing
static constexpr uint32_t SAMPLE_INTERVAL_US = 500;      // Lower for small capacitors, higher for large ones.
static constexpr uint32_t MAX_CHARGE_TIME_US  = 2000000;  // 2 s
static constexpr uint32_t MAX_DISCHARGE_TIME_US = 2000000;
static constexpr size_t   MAX_SAMPLES = 160;

// Thresholds
static constexpr float CHARGE_STOP_FRACTION    = 0.985f;  // stop when near plateau
static constexpr float DISCHARGE_STOP_V        = 0.010f;  // stop when near zero
static constexpr float MIN_FIT_SPAN_FRACTION   = 0.10f;   // exclude extreme early/late samples
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
  float r2 = NAN;
  float rmse_v = NAN;     // in volts, in the transformed fit domain converted back is not trivial; here residual in y-domain is converted approximately by exp scale
  uint16_t usedPoints = 0;
};

struct MeasurementResult {
  bool ok = false;

  float vSupply = V_SUPPLY_V;
  float vPlateau = NAN;

  float leakOhms = NAN;
  float leakCurrent_uA = NAN;

  float tauCharge_us = NAN;
  float tauDischarge_us = NAN;

  float cCharge_uF = NAN;
  float cDischarge_uF = NAN;
  float cFinal_uF = NAN;

  float esrProxyCharge_Ohm = NAN;     // proxy: total extra series resistance in charge path beyond R_CHARGE_OHMS
  float esrProxyDischarge_Ohm = NAN;   // proxy: total extra series resistance in discharge path beyond R_DISCHARGE_OHMS

  float linearityR2 = NAN;
  float chargeDischargeMismatchPct = NAN;
  float fitQuality = NAN;
};

static Sample chargeSamples[MAX_SAMPLES];
static Sample dischargeSamples[MAX_SAMPLES];

// ---------------------------------------------------------------------------
// ADC helper
// ---------------------------------------------------------------------------

static float readVoltageOnce()
{
#if defined(ESP32)
  // More accurate than raw counts scaling, if available in your core build.
  uint32_t mv = analogReadMilliVolts(measurePin);
  return mv / 1000.0f;
#else
  uint16_t raw = analogRead(measurePin);
  return (raw * V_SUPPLY_V) / float(ADC_MAX);
#endif
}

static float readVoltageAveraged(uint8_t n = 8)
{
  float sum = 0.0f;
  for (uint8_t i = 0; i < n; ++i) {
    sum += readVoltageOnce();
  }
  return sum / n;
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
// Sampling
// ---------------------------------------------------------------------------

static bool captureChargeCurve(Sample *samples, size_t &count, uint32_t maxTimeUs, uint32_t sampleIntervalUs)
{
  count = 0;
  startCharge();

  const uint32_t t0 = micros();
  uint32_t nextSample = t0;

  while (true) {
    const uint32_t now = micros();

    if ((uint32_t)(now - nextSample) < 0x80000000UL || now == nextSample) {
      if (count < MAX_SAMPLES) {
        samples[count++] = { now - t0, readVoltageAveraged() };
      }

      const float v = samples[count - 1].v;
      if (count >= MAX_SAMPLES) break;
      if (v >= V_SUPPLY_V * CHARGE_STOP_FRACTION) break;
      if ((now - t0) >= maxTimeUs) break;

      nextSample += sampleIntervalUs;
    }

    // Prevent a tight spin from starving other tasks.
    yield();
  }

  stopCharge();
  return count >= 6;
}

static bool captureDischargeCurve(Sample *samples, size_t &count, uint32_t maxTimeUs, uint32_t sampleIntervalUs)
{
  count = 0;
  startDischarge();

  const uint32_t t0 = micros();
  uint32_t nextSample = t0;

  while (true) {
    const uint32_t now = micros();

    if ((uint32_t)(now - nextSample) < 0x80000000UL || now == nextSample) {
      if (count < MAX_SAMPLES) {
        samples[count++] = { now - t0, readVoltageAveraged() };
      }

      const float v = samples[count - 1].v;
      if (count >= MAX_SAMPLES) break;
      if (v <= DISCHARGE_STOP_V) break;
      if ((now - t0) >= maxTimeUs) break;

      nextSample += sampleIntervalUs;
    }

    yield();
  }

  stopDischarge();
  return count >= 6;
}

// ---------------------------------------------------------------------------
// Exponential fit helpers
// Model for charge:
//   v(t) = vinf - (vinf - v0) * exp(-t / tau)
// Linearized:
//   ln((vinf - v(t)) / (vinf - v0)) = -t / tau
//
// Model for discharge:
//   v(t) = v0 * exp(-t / tau)
// Linearized:
//   ln(v(t) / v0) = -t / tau
// ---------------------------------------------------------------------------

static float averageLastV(const Sample *s, size_t n, size_t k = 4)
{
  if (n == 0) return NAN;
  if (k > n) k = n;

  float sum = 0.0f;
  for (size_t i = n - k; i < n; ++i) {
    sum += s[i].v;
  }
  return sum / float(k);
}

static FitResult fitRiseCurve(const Sample *s, size_t n)
{
  FitResult r;
  if (n < 6) return r;

  const float v0 = s[0].v;
  const float vinf = averageLastV(s, n, 4);
  const float span = vinf - v0;

  if (!isfinite(vinf) || !isfinite(v0) || span <= 0.01f) return r;

  // Select only the middle part of the curve for a stable linear fit.
  double sumX = 0, sumY = 0, sumXX = 0, sumXY = 0;
  size_t used = 0;

  for (size_t i = 1; i < n; ++i) {
    const float frac = (s[i].v - v0) / span;
    if (frac <= MIN_FIT_SPAN_FRACTION || frac >= MAX_FIT_SPAN_FRACTION) continue;

    const float ratio = (vinf - s[i].v) / (vinf - v0);
    if (!(ratio > 0.0f)) continue;

    const double x = double(s[i].t_us);
    const double y = log(double(ratio));

    sumX  += x;
    sumY  += y;
    sumXX += x * x;
    sumXY += x * y;
    used++;
  }

  if (used < 3) return r;

  const double denom = used * sumXX - sumX * sumX;
  if (fabs(denom) < 1e-12) return r;

  const double slope = (used * sumXY - sumX * sumY) / denom;
  const double intercept = (sumY - slope * sumX) / used;

  if (!(slope < 0.0)) return r;

  // R^2 and residuals in transformed space.
  const double yMean = sumY / used;
  double ssTot = 0.0, ssRes = 0.0;
  double rmseSum = 0.0;

  for (size_t i = 1; i < n; ++i) {
    const float frac = (s[i].v - v0) / span;
    if (frac <= MIN_FIT_SPAN_FRACTION || frac >= MAX_FIT_SPAN_FRACTION) continue;

    const float ratio = (vinf - s[i].v) / (vinf - v0);
    if (!(ratio > 0.0f)) continue;

    const double x = double(s[i].t_us);
    const double y = log(double(ratio));
    const double yHat = slope * x + intercept;

    ssTot += (y - yMean) * (y - yMean);
    ssRes += (y - yHat) * (y - yHat);
    rmseSum += (y - yHat) * (y - yHat);
  }

  r.ok = true;
  r.tau_us = float(-1.0 / slope);
  r.v0 = v0;
  r.vinf = vinf;
  r.usedPoints = used;
  r.r2 = (ssTot > 0.0) ? float(1.0 - ssRes / ssTot) : NAN;
  r.rmse_v = float(sqrt(rmseSum / used)); // residual in log-domain
  return r;
}

static FitResult fitDecayCurve(const Sample *s, size_t n)
{
  FitResult r;
  if (n < 6) return r;

  const float v0 = s[0].v;
  const float vEnd = averageLastV(s, n, 4);

  if (!isfinite(v0) || v0 <= 0.001f) return r;

  double sumX = 0, sumY = 0, sumXX = 0, sumXY = 0;
  size_t used = 0;

  for (size_t i = 1; i < n; ++i) {
    const float frac = s[i].v / v0;
    if (frac <= MIN_FIT_SPAN_FRACTION || frac >= MAX_FIT_SPAN_FRACTION) continue;
    if (!(s[i].v > 0.0f)) continue;

    const double x = double(s[i].t_us);
    const double y = log(double(s[i].v / v0));

    sumX  += x;
    sumY  += y;
    sumXX += x * x;
    sumXY += x * y;
    used++;
  }

  if (used < 3) return r;

  const double denom = used * sumXX - sumX * sumX;
  if (fabs(denom) < 1e-12) return r;

  const double slope = (used * sumXY - sumX * sumY) / denom;
  const double intercept = (sumY - slope * sumX) / used;

  if (!(slope < 0.0)) return r;

  const double yMean = sumY / used;
  double ssTot = 0.0, ssRes = 0.0;
  double rmseSum = 0.0;

  for (size_t i = 1; i < n; ++i) {
    const float frac = s[i].v / v0;
    if (frac <= MIN_FIT_SPAN_FRACTION || frac >= MAX_FIT_SPAN_FRACTION) continue;
    if (!(s[i].v > 0.0f)) continue;

    const double x = double(s[i].t_us);
    const double y = log(double(s[i].v / v0));
    const double yHat = slope * x + intercept;

    ssTot += (y - yMean) * (y - yMean);
    ssRes += (y - yHat) * (y - yHat);
    rmseSum += (y - yHat) * (y - yHat);
  }

  r.ok = true;
  r.tau_us = float(-1.0 / slope);
  r.v0 = v0;
  r.vinf = vEnd;
  r.usedPoints = used;
  r.r2 = (ssTot > 0.0) ? float(1.0 - ssRes / ssTot) : NAN;
  r.rmse_v = float(sqrt(rmseSum / used));
  return r;
}

// ---------------------------------------------------------------------------
// Leakage / capacitance / ESR proxy inference
// ---------------------------------------------------------------------------

static float estimateLeakResistance(float vPlateau, float vSupply, float rChargeOhms)
{
  if (!(vPlateau > 0.0f) || !(vSupply > 0.0f)) return NAN;
  if (vPlateau >= vSupply * 0.995f) return INFINITY;

  const float denom = (vSupply - vPlateau);
  if (denom <= 0.0f) return NAN;

  return rChargeOhms * vPlateau / denom;
}

static float capacitanceFromTau(float tau_us, float rSeriesOhms, float rLeakOhms)
{
  if (!(tau_us > 0.0f) || !(rSeriesOhms > 0.0f)) return NAN;

  const float tau_s = tau_us * 1e-6f;

  if (!isfinite(rLeakOhms) || rLeakOhms <= 0.0f) {
    return tau_s / rSeriesOhms;
  }

  // For charge/discharge with leakage to ground:
  // tau = C * (R || Rleak) = C * (R * Rleak)/(R + Rleak)
  // => C = tau * (R + Rleak)/(R * Rleak)
  return tau_s * (rSeriesOhms + rLeakOhms) / (rSeriesOhms * rLeakOhms);
}

// Estimate extra series resistance from the first part of the rise curve.
// This is a practical proxy for ESR + wiring/contact resistance.
// It assumes the charge curve is close to a single exponential.
static float estimateSeriesResistanceProxy(const Sample *s, size_t n, const FitResult &fit, float rKnownOhms)
{
  if (!fit.ok || n < 3 || !(fit.tau_us > 0.0f) || !(fit.vinf > 0.0f)) return NAN;

  // Use the earliest valid points. The earlier the samples, the better this estimate.
  const size_t kMax = min<size_t>(6, n);
  float aSum = 0.0f;
  size_t used = 0;

  for (size_t i = 0; i < kMax; ++i) {
    const float t = float(s[i].t_us);
    const float v = s[i].v;

    if (!(v >= 0.0f) || !(v < fit.vinf)) continue;

    // v(t) ≈ Vinf - A * exp(-t/tau), so A ≈ (Vinf - v) * exp(t/tau)
    const float A = (fit.vinf - v) * expf(t / fit.tau_us);
    if (!(A > 0.0f) || !(A < fit.vinf)) continue;

    aSum += A;
    used++;
  }

  if (used < 2) return NAN;

  const float A = aSum / float(used);

  // For a pure step into R + ESR:
  // A = Vinf * R / (R + ESR)
  // => ESR = R * (Vinf - A) / A
  const float esr = rKnownOhms * (fit.vinf - A) / A;
  if (!(esr >= 0.0f)) return NAN;
  return esr;
}

// ---------------------------------------------------------------------------
// Full measurement routine
// ---------------------------------------------------------------------------

static MeasurementResult measureCapacitor()
{
  MeasurementResult out;

  releaseAllPins();
#if defined(ESP32)
  analogReadResolution(ADC_BITS);
  analogSetPinAttenuation(measurePin, ADC_11db);
#endif

  // Fully discharge before measuring.
  startDischarge();
  delay(50);
  stopDischarge();
  delay(20);

  // Measure baseline near zero.
  float vStart = readVoltageAveraged();
  if (vStart > 0.050f) {
    // Give it a bit more time if the node is not near zero.
    startDischarge();
    delay(150);
    stopDischarge();
    delay(20);
    vStart = readVoltageAveraged();
  }

  // Charge curve
  size_t nCharge = 0;
  const bool chargeOk = captureChargeCurve(chargeSamples, nCharge, MAX_CHARGE_TIME_US, SAMPLE_INTERVAL_US);

  // Use plateau from the end of the curve.
  const float vPlateau = averageLastV(chargeSamples, nCharge, 4);

  // Charge fit.
  FitResult riseFit = fitRiseCurve(chargeSamples, nCharge);

  // Leakage estimate from the asymptotic plateau.
  const float rLeak = estimateLeakResistance(vPlateau, V_SUPPLY_V, R_CHARGE_OHMS);
  const bool leakFinite = isfinite(rLeak) && rLeak > 0.0f && rLeak < 1e12f;

  // Capacitance from charge fit.
  const float cCharge_uF = riseFit.ok ? capacitanceFromTau(riseFit.tau_us, R_CHARGE_OHMS, rLeak) * 1e6f : NAN;

  // ESR proxy from the earliest samples of the rise curve.
  const float esrCharge = estimateSeriesResistanceProxy(chargeSamples, nCharge, riseFit, R_CHARGE_OHMS);

  // Discharge curve
  // First charge the capacitor to a high value, then switch to discharge and sample.
  if (!chargeOk) {
    // Still attempt discharge capture if charge capture was weak.
  }

  size_t nDischarge = 0;
  const bool dischargeOk = captureDischargeCurve(dischargeSamples, nDischarge, MAX_DISCHARGE_TIME_US, SAMPLE_INTERVAL_US);
  FitResult fallFit = fitDecayCurve(dischargeSamples, nDischarge);

  const float cDischarge_uF = fallFit.ok ? capacitanceFromTau(fallFit.tau_us, R_DISCHARGE_OHMS, rLeak) * 1e6f : NAN;

  // Another ESR proxy for the discharge path.
  // Here it is "extra series resistance" in the discharge path beyond the known resistor.
  const float esrDischarge = estimateSeriesResistanceProxy(dischargeSamples, nDischarge, fallFit, R_DISCHARGE_OHMS);

  // Final capacitance: weighted average of charge/discharge estimates.
  float cFinal_uF = NAN;
  if (isfinite(cCharge_uF) && isfinite(cDischarge_uF)) {
    const float w1 = isfinite(riseFit.r2) ? max(0.0f, riseFit.r2) : 1.0f;
    const float w2 = isfinite(fallFit.r2) ? max(0.0f, fallFit.r2) : 1.0f;
    const float wSum = w1 + w2;

    if (wSum > 0.0f) cFinal_uF = (cCharge_uF * w1 + cDischarge_uF * w2) / wSum;
  } else if (isfinite(cCharge_uF)) {
    cFinal_uF = cCharge_uF;
  } else if (isfinite(cDischarge_uF)) {
    cFinal_uF = cDischarge_uF;
  }

  // Linearity / confidence:
  // - exponential fit quality on both curves
  // - agreement between charge and discharge capacitance
  float mismatchPct = NAN;
  if (isfinite(cCharge_uF) && isfinite(cDischarge_uF) && (cCharge_uF + cDischarge_uF) > 0.0f) {
    mismatchPct = fabs(cCharge_uF - cDischarge_uF) / ((cCharge_uF + cDischarge_uF) * 0.5f) * 100.0f;
  }

  float fitQuality = NAN;
  if (isfinite(riseFit.r2) && isfinite(fallFit.r2)) {
    fitQuality = min(riseFit.r2, fallFit.r2);
  } else if (isfinite(riseFit.r2)) {
    fitQuality = riseFit.r2;
  } else if (isfinite(fallFit.r2)) {
    fitQuality = fallFit.r2;
  }

  out.ok = chargeOk || dischargeOk;
  out.vSupply = V_SUPPLY_V;
  out.vPlateau = vPlateau;

  out.leakOhms = rLeak;
  out.leakCurrent_uA = (leakFinite && vPlateau > 0.0f) ? (vPlateau / rLeak) * 1e6f : NAN;

  out.tauCharge_us = riseFit.tau_us;
  out.tauDischarge_us = fallFit.tau_us;

  out.cCharge_uF = cCharge_uF;
  out.cDischarge_uF = cDischarge_uF;
  out.cFinal_uF = cFinal_uF;

  out.esrProxyCharge_Ohm = esrCharge;
  out.esrProxyDischarge_Ohm = esrDischarge;

  out.linearityR2 = fitQuality;
  out.chargeDischargeMismatchPct = mismatchPct;
  out.fitQuality = fitQuality;

  return out;
}

// ---------------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  delay(300);

  releaseAllPins();

#if defined(ESP32)
  analogReadResolution(ADC_BITS);
  analogSetPinAttenuation(measurePin, ADC_11db);
#endif

  Serial.println();
  Serial.println("cap_uF_charge,cap_uF_discharge,cap_uF_final,tau_us_charge,tau_us_discharge,leak_ohm,leak_uA,esr_proxy_charge_ohm,esr_proxy_discharge_ohm,linearity_r2,mismatch_pct,v_plateau_v,status");
}

void loop()
{
  const MeasurementResult m = measureCapacitor();

  const char *status =
      (!m.ok) ? "FAIL" :
      (!isfinite(m.cFinal_uF) || m.cFinal_uF <= 0.0f) ? "BAD_CAP" :
      (isfinite(m.linearityR2) && m.linearityR2 < 0.995f) ? "WARN_LINEARITY" :
      (isfinite(m.chargeDischargeMismatchPct) && m.chargeDischargeMismatchPct > 10.0f) ? "WARN_MISMATCH" :
      "OK";

  Serial.print(m.cCharge_uF, 6); Serial.print(',');
  Serial.print(m.cDischarge_uF, 6); Serial.print(',');
  Serial.print(m.cFinal_uF, 6); Serial.print(',');
  Serial.print(m.tauCharge_us, 2); Serial.print(',');
  Serial.print(m.tauDischarge_us, 2); Serial.print(',');
  Serial.print(m.leakOhms, 2); Serial.print(',');
  Serial.print(m.leakCurrent_uA, 3); Serial.print(',');
  Serial.print(m.esrProxyCharge_Ohm, 2); Serial.print(',');
  Serial.print(m.esrProxyDischarge_Ohm, 2); Serial.print(',');
  Serial.print(m.linearityR2, 5); Serial.print(',');
  Serial.print(m.chargeDischargeMismatchPct, 2); Serial.print(',');
  Serial.print(m.vPlateau, 4); Serial.print(',');
  Serial.println(status);

  delay(2000);
}
