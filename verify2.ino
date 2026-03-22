#include <Arduino.h>
#include <math.h>

#if defined(ESP32)
  #include <esp32-hal-adc.h>
#endif

// Hardware configuration
static constexpr uint8_t chargePin    = 5;
static constexpr uint8_t dischargePin = 18;
static constexpr uint8_t measurePin   = 34;

static constexpr float V_SUPPLY_V = 3.300f;
static constexpr float R_CHARGE_OHMS    = 10000.0f;
static constexpr float R_DISCHARGE_OHMS = 10000.0f;
static constexpr float R_GPIO           = 80.0f; // Typical ESP32 output resistance

struct Sample { uint32_t t_us; float v; };
static constexpr size_t MAX_SAMPLES = 500;
static Sample samples_buf[MAX_SAMPLES];

// Pin control & ADC
static float readVoltageOnce() {
#if defined(ESP32)
    return (float)analogReadMilliVolts(measurePin) / 1000.0f;
#else
    return (analogRead(measurePin) * V_SUPPLY_V) / 4095.0f;
#endif
}

static float readVoltage(uint8_t oversample = 8) {
    if (oversample < 1) oversample = 1;
    float sum = 0;
    for (uint8_t i = 0; i < oversample; i++) {
        sum += readVoltageOnce();
    }
    return sum / (float)oversample;
}

static void waitMicros(uint32_t target) {
    while (true) {
        uint32_t now = micros();
        int32_t diff = (int32_t)(target - now);
        if (diff <= 0) break;
        if (diff > 1000) {
            yield();
        } else {
            __asm__ __volatile__("nop");
        }
    }
}

static void releaseAllPins() { pinMode(chargePin, INPUT); pinMode(dischargePin, INPUT); }
static void startCharge() { pinMode(dischargePin, INPUT); digitalWrite(chargePin, HIGH); pinMode(chargePin, OUTPUT); }
static void stopCharge() { pinMode(chargePin, INPUT); }
static void startDischarge() { pinMode(chargePin, INPUT); digitalWrite(dischargePin, LOW); pinMode(dischargePin, OUTPUT); }
static void stopDischarge() { pinMode(dischargePin, INPUT); }

// Method 1: Exponential Fit
struct FitResult { bool ok = false; float tau_us = NAN; float vinf = NAN; float score = INFINITY; };

static FitResult fitWithFixedVinf(const Sample *s, size_t n, float v0, float vinf) {
    FitResult r;
    r.vinf = vinf;
    if (fabsf(vinf - v0) < 0.05f) return r;

    double sumW = 0, sumWX = 0, sumWY = 0, sumWXX = 0, sumWXY = 0;
    size_t used = 0;
    for (size_t i = 0; i < n; ++i) {
        float ratio = (vinf - s[i].v) / (vinf - v0);
        if (ratio < 1e-6f || ratio >= 1.0f) continue;

        float frac = (s[i].v - v0) / (vinf - v0);
        if (frac > 0.05f && frac < 0.95f) {
            float y = logf(ratio);
            float tRel = (float)s[i].t_us;
            float weight = (vinf - s[i].v) * (vinf - s[i].v);

            sumW   += weight;
            sumWX  += weight * tRel;
            sumWY  += weight * y;
            sumWXX += weight * tRel * tRel;
            sumWXY += weight * tRel * y;
            used++;
        }
    }
    if (used < 3) return r;
    double delta = sumW * sumWXX - sumWX * sumWX;
    if (fabs(delta) < 1e-12) return r;
    double slope = (sumW * sumWXY - sumWX * sumWY) / delta;
    if (slope >= 0) return r;

    r.ok = true;
    r.tau_us = -1.0f / (float)slope;

    double rss = 0;
    for (size_t i = 0; i < n; ++i) {
        float v_pred = vinf - (vinf - v0) * expf(-(float)s[i].t_us / r.tau_us);
        rss += (s[i].v - v_pred) * (s[i].v - v_pred);
    }
    r.score = (float)rss / used;
    return r;
}

static FitResult fitExponential(const Sample *s, size_t n) {
    FitResult best;
    if (n < 10) return best;

    const float v0 = s[0].v;
    float v_last = s[n-1].v;
    float vinf_min = fmaxf(v_last + 0.01f, v0 + 0.02f);
    float vinf_max = fmaxf(V_SUPPLY_V * 1.15f, vinf_min + 0.1f);

    // Multi-pass search for Vinf to minimize RSS
    float current_min = vinf_min;
    float current_max = vinf_max;

    for (int pass = 0; pass < 4; pass++) {
        float step = (current_max - current_min) / 10.0f;
        FitResult passBest;
        float bestVinf = current_min;

        for (int i = 0; i <= 10; i++) {
            float vinf_test = current_min + step * i;
            FitResult res = fitWithFixedVinf(s, n, v0, vinf_test);
            if (res.ok && res.score < passBest.score) {
                passBest = res;
                bestVinf = vinf_test;
            }
        }

        if (passBest.ok) {
            best = passBest;
            // Golden-section inspired range narrowing
            current_min = fmaxf(vinf_min, bestVinf - step);
            current_max = fminf(vinf_max, bestVinf + step);
        } else break;
    }

    // Final sanity checks on the model
    if (best.ok) {
        if (best.tau_us < 1.0f || best.tau_us > 100000000.0f) best.ok = false;
        // score is RSS / used points. RMSE is sqrt(score).
        // Standardize: fit is questionable if RMSE > 50mV
        if (sqrtf(best.score) > 0.050f) best.ok = false;
    }

    return best;
}

// Method 2: Leakage via Voltage Decay (Self-Discharge)
static float estimateLeakResistance(float cap_uF) {
    if (cap_uF < 0.001f) return 1e12f; // Too small to measure decay accurately

    startCharge();
    uint32_t deadline = millis() + 5000;
    while(readVoltage() < V_SUPPLY_V * 0.9f) {
        if (millis() > deadline) break;
        yield();
    }
    stopCharge();
    releaseAllPins();
    delay(100);

    float v1 = readVoltage(16);
    uint32_t t1 = micros();

    // Adaptive window: longer for smaller capacitors or higher expected resistance
    uint32_t duration_ms = 1000;
    if (cap_uF > 1000.0f) duration_ms = 2000;

    deadline = millis() + duration_ms;
    while(millis() < deadline) { yield(); }

    float v2 = readVoltage(16);
    uint32_t t2 = micros();

    // If drop is too small, double the window for better resolution
    if (v1 - v2 < 0.05f && duration_ms < 5000) {
        deadline = millis() + duration_ms;
        while(millis() < deadline) { yield(); }
        v2 = readVoltage(16);
        t2 = micros();
    }

    float dt = (float)(t2 - t1) / 1000000.0f;
    if (v1 <= v2 || v2 < 0.05f) return 1e12f;

    float tau_leak = dt / logf(v1 / v2);
    return tau_leak / (cap_uF * 1e-6f);
}

// Method 3: Relaxation Oscillator
static float measureCapOscillator(float leak) {
    float vL = V_SUPPLY_V * 0.333f, vH = V_SUPPLY_V * 0.666f;
    float vSteady = V_SUPPLY_V * leak / (R_CHARGE_OHMS + R_GPIO + leak);
    if (vSteady < vH * 1.05f) return NAN;

    // Phase 1: Pre-condition - charge above vL
    uint32_t deadline = millis() + 5000;
    startCharge();
    while(readVoltage() < vL * 1.1f) {
        if (millis() > deadline) return NAN;
        yield();
    }
    stopCharge();

    // Phase 2: Establish known state at vL with discharge
    deadline = millis() + 5000;
    startDischarge();
    while(readVoltage() > vL) {
        if (millis() > deadline) return NAN;
        yield();
    }
    stopDischarge();

    uint32_t tAccumUs = 0;
    const int cycles = 10;
    for(int i=0; i < cycles; i++) {
        uint32_t tCycleStart = micros();
        deadline = millis() + 10000;
        startCharge();
        while(readVoltage() < vH) {
            if (millis() > deadline) return NAN;
            yield();
        }
        stopCharge();
        deadline = millis() + 10000;
        startDischarge();
        while(readVoltage() > vL) {
            if (millis() > deadline) return NAN;
            yield();
        }
        stopDischarge();
        tAccumUs += (micros() - tCycleStart);
    }
    float tCycle = (float)tAccumUs / (float)cycles;
    float a = (vSteady - vL) / (vSteady - vH), b = vH / vL;
    if (a <= 0.0f || b <= 0.0f) return NAN;
    float tcN = logf(a), tdN = logf(b);
    float rEq = ((R_CHARGE_OHMS + R_GPIO) * leak) / (R_CHARGE_OHMS + R_GPIO + leak);
    float rEqD = ((R_DISCHARGE_OHMS + R_GPIO) * leak) / (R_DISCHARGE_OHMS + R_GPIO + leak);
    return (tCycle * 1e-6f / (rEq * tcN + rEqD * tdN)) * 1e6f;
}

// Method 4: Virtual Feedback Gain (dV/dt)
static float measureCapFeedbackGain(float leak) {
    if (!isfinite(leak) || leak < 1000.0f) return NAN;
    float vStart = 1.0f, vEnd = 1.2f;
    uint32_t deadline = millis() + 5000;
    if (readVoltage() > vStart) {
        startDischarge();
        while(readVoltage() > vStart * 0.95f) {
            if (millis() > deadline) break;
            yield();
        }
        stopDischarge();
    }
    deadline = millis() + 10000;
    startCharge();
    while(readVoltage() < vStart) {
        if (millis() > deadline) return NAN;
        yield();
    }
    uint32_t t1 = micros();
    deadline = millis() + 20000;
    while(readVoltage() < vEnd) {
        if (millis() > deadline) return NAN;
        yield();
    }
    uint32_t t2 = micros();
    stopCharge();
    float dt = (float)(t2 - t1) / 1000000.0f, dv = vEnd - vStart;
    float vAvg = (vStart + vEnd) / 2.0f;
    float i_cap = ((V_SUPPLY_V - vAvg) / (R_CHARGE_OHMS + R_GPIO)) - (vAvg / leak);
    if (i_cap <= 0.0f) return NAN;
    float cap = (i_cap * dt / dv) * 1e6f;
    return (cap > 0.0f) ? cap : NAN;
}

void setup() {
    Serial.begin(115200); releaseAllPins();
#if defined(ESP32)
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
#endif
    Serial.println("\n--- Capacitor Parameter Inference System ---");
    Serial.println("mode,cap_uF,leak_MOhm,fit_status,tau_us,vinf_v,rmse_v");
}

enum class MeasurementMode { NONE, FIT, OSC, GAIN };

const char* modeToString(MeasurementMode m) {
    switch(m) {
        case MeasurementMode::FIT: return "FIT";
        case MeasurementMode::OSC: return "OSC";
        case MeasurementMode::GAIN: return "GAIN";
        default: return "NONE";
    }
}

void loop() {
    size_t n = 0;
    startDischarge(); delay(1500); stopDischarge();

    float v0_baseline = readVoltage();

    startCharge();
    uint32_t t0 = micros();

    // Capture first point immediately with low oversampling to catch fast transients
    float vStart = readVoltage(2);
    samples_buf[n++] = { (uint32_t)(micros() - t0), vStart };

    uint32_t baseInterval = 5;
    if (readVoltage(2) < 0.01 * V_SUPPLY_V) baseInterval = 1000;

    while (n < MAX_SAMPLES) {
        // Dynamic oversampling: 2x early on, 16x near plateau
        uint8_t os = (n < 50) ? 2 : (n < 200 ? 8 : 16);
        float vSample = readVoltage(os);
        samples_buf[n++] = { (uint32_t)(micros() - t0), vSample };
        // Charge until very close to asymptote to ensure good Vinf estimation
        if (samples_buf[n-1].v >= V_SUPPLY_V * 0.99f || samples_buf[n-1].t_us > 15000000) break;

        // Balanced sampling strategy: high density early, sufficient density late
        uint32_t interval = baseInterval + (uint32_t)(n * 0.5f * (float)baseInterval);
        if (n > 200) interval += (uint32_t)((n - 200) * 5.0f * (float)baseInterval);

        uint32_t target = micros() + interval;
        waitMicros(target);
        yield();
    }
    stopCharge();

    FitResult fr = fitExponential(samples_buf, n);

    float leak = NAN;
    float capFit = NAN, capOsc = NAN, capGain = NAN;

    if (fr.ok) {
        // Multi-pass refinement of C and Leak
        capFit = fr.tau_us / (R_CHARGE_OHMS + R_GPIO); // 1st pass: assume no leak
        for(int i=0; i<2; i++) {
            float leak_candidate = estimateLeakResistance(capFit);
            // Self-check leak: if it's too low to be plausible for a "good" cap,
            // it might be a measurement error.
            if (isfinite(leak_candidate) && leak_candidate > 10.0f) {
                leak = leak_candidate;
            } else {
                leak = 1e12f; break;
            }
            capFit = (fr.tau_us * (R_CHARGE_OHMS + R_GPIO + leak)) / ((R_CHARGE_OHMS + R_GPIO) * leak);
        }
    }

    if (!fr.ok || capFit > 1.0f) {
        capOsc = measureCapOscillator(isfinite(leak) ? leak : 1e12f);
    }

    if (isnan(capOsc) || capOsc > 500.0f) {
        capGain = measureCapFeedbackGain(isfinite(leak) ? leak : 1e12f);
    }

    float finalCap = 0;
    MeasurementMode mode = MeasurementMode::NONE;

    if (isfinite(capFit) && capFit > 0) { finalCap = capFit; mode = MeasurementMode::FIT; }
    if (isfinite(capOsc) && capOsc > 0) {
        // OSC is generally better for medium caps if it agrees with FIT or if FIT failed
        bool agreeWithFit = (isfinite(capFit) && capFit > 0.001f && fabsf(capOsc - capFit) / capFit < 0.2f);
        if (mode == MeasurementMode::NONE || agreeWithFit || capFit > 100.0f) {
             finalCap = capOsc; mode = MeasurementMode::OSC;
        }
    }
    if (isfinite(capGain) && capGain > 0) {
        if (mode == MeasurementMode::NONE || (isfinite(capOsc) && capOsc > 1000.0f)) {
            finalCap = capGain; mode = MeasurementMode::GAIN;
        }
    }

    if (mode == MeasurementMode::NONE) {
        Serial.println("FAIL,0,0,NO_ESTIMATE,0,0,0");
    } else {
        Serial.print(modeToString(mode)); Serial.print(",");
        Serial.print(finalCap, 6); Serial.print(",");
        if (!isfinite(leak) || leak > 1e11) Serial.print("INF"); else Serial.print(leak / 1e6f, 2);
        Serial.print(",");
        Serial.print(fr.ok ? "OK" : "EST"); Serial.print(",");
        Serial.print(fr.tau_us); Serial.print(",");
        Serial.print(fr.vinf); Serial.print(",");
        Serial.println(fr.ok ? sqrtf(fr.score) : 0.0f, 4);
    }
    delay(3000);
}
