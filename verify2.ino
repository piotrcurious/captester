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

static float readVoltage() {
    float sum = 0;
    for (int i = 0; i < 8; i++) {
        sum += readVoltageOnce();
    }
    return sum / 8.0f;
}

static void waitMicros(uint32_t target) {
    while ((int32_t)(micros() - target) < 0) {
        if ((int32_t)(target - micros()) > 1000) {
            yield();
        } else {
            __asm__ __volatile__("nop");
        }
    }
}

static void releaseAllPins() { pinMode(chargePin, INPUT); pinMode(dischargePin, INPUT); }
static void startCharge() { pinMode(dischargePin, INPUT); pinMode(chargePin, OUTPUT); digitalWrite(chargePin, HIGH); }
static void stopCharge() { pinMode(chargePin, INPUT); digitalWrite(chargePin, LOW); }
static void startDischarge() { pinMode(chargePin, INPUT); pinMode(dischargePin, OUTPUT); digitalWrite(dischargePin, LOW); }
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

static FitResult fitExponential(const Sample *s, size_t n, float v0) {
    FitResult best;
    if (n < 10) return best;

    float v_last = s[n-1].v;
    float vinf_min = max(v_last + 0.05f, v0 + 0.1f);
    float vinf_max = V_SUPPLY_V;

    for (int i = 0; i <= 20; i++) {
        float vinf_test = vinf_min + (vinf_max - vinf_min) * (i / 20.0f);
        FitResult res = fitWithFixedVinf(s, n, v0, vinf_test);
        if (res.ok && res.score < best.score) {
            best = res;
        }
    }
    return best;
}

// Method 2: Virtual Feedback Leakage Test
static float measureLeakageFeedback(float targetV = 1.65f, uint32_t duration_ms = 1000) {
    uint32_t tS = millis();
    while(readVoltage() < targetV) {
        if (millis() - tS > 5000) return NAN;
        startCharge(); delay(2); stopCharge(); yield();
    }
    while(readVoltage() > targetV + 0.02f) {
        if (millis() - tS > 6000) break;
        startDischarge(); delay(2); stopDischarge(); yield();
    }

    uint32_t high_us = 0;
    uint32_t total_us = 0;
    uint32_t tDeadline = millis() + duration_ms;

    while(millis() < tDeadline) {
        uint32_t tStepStart = micros();
        if (readVoltage() < targetV) {
            startCharge();
            uint32_t tPulseStart = micros();
            delayMicroseconds(50);
            uint32_t tPulseEnd = micros();
            stopCharge();
            high_us += (tPulseEnd - tPulseStart);
        } else {
            stopCharge();
            delayMicroseconds(50);
        }
        total_us += (micros() - tStepStart);
        yield();
    }
    if (total_us == 0) return NAN;
    float duty = (float)high_us / (float)total_us;
    if (duty < 0.0001f) return 1e12f;
    return (targetV * (R_CHARGE_OHMS + R_GPIO)) / ((V_SUPPLY_V - targetV) * duty);
}

// Method 3: Relaxation Oscillator
static float measureCapOscillator(float leak) {
    float vL = V_SUPPLY_V * 0.333f, vH = V_SUPPLY_V * 0.666f;
    float vSteady = V_SUPPLY_V * leak / (R_CHARGE_OHMS + R_GPIO + leak);
    if (vSteady < vH * 1.05f) return NAN;

    uint32_t deadline = millis() + 5000;
    startCharge();
    while(readVoltage() < vL) {
        if (millis() > deadline) return NAN;
        yield();
    }
    stopCharge();

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
    return (i_cap * dt / dv) * 1e6f;
}

void setup() {
    Serial.begin(115200); releaseAllPins();
#if defined(ESP32)
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
#endif
    Serial.println("\n--- Capacitor Parameter Inference System ---");
    Serial.println("mode,cap_uF,leak_MOhm,status");
}

void loop() {
    size_t n = 0;
    startDischarge(); delay(1500); stopDischarge();

    float v0_baseline = readVoltage();

    startCharge();
    uint32_t t0 = micros();
    uint32_t baseInterval = 5;
    delayMicroseconds(50);
    if (readVoltage() < 0.01 * V_SUPPLY_V) baseInterval = 1000;

    while (n < MAX_SAMPLES) {
        samples_buf[n++] = { micros() - t0, readVoltage() };
        if (samples_buf[n-1].v >= V_SUPPLY_V * 0.95f || samples_buf[n-1].t_us > 15000000) break;
        uint32_t interval = baseInterval + (uint32_t)(n * n * 0.005f * (float)baseInterval);
        uint32_t target = micros() + interval;
        waitMicros(target);
        yield();
    }
    stopCharge();

    FitResult fr = fitExponential(samples_buf, n, v0_baseline);
    if (fr.ok) {
        float leak = measureLeakageFeedback();
        if (!isfinite(leak) || leak <= 0.0f) {
            Serial.println("FAIL,0,0,LEAK_INVALID");
            delay(3000); return;
        }
        float cap = (fr.tau_us * 1e-6f * (R_CHARGE_OHMS + R_GPIO + leak) / ((R_CHARGE_OHMS + R_GPIO) * leak)) * 1e6f;
        if (!isfinite(cap) || cap <= 0.0f) {
            Serial.println("FAIL,0,0,CAP_INVALID");
            delay(3000); return;
        }
        const char* mode = "FIT";
        if (cap > 10.0f) {
            float capOsc = measureCapOscillator(leak);
            if (isfinite(capOsc) && capOsc > 0.0f) { cap = capOsc; mode = "OSC"; }
        }
        if (cap > 1000.0f) {
            float capGain = measureCapFeedbackGain(leak);
            if (isfinite(capGain) && capGain > 0.0f) { cap = capGain; mode = "GAIN"; }
        }
        Serial.print(mode); Serial.print(",");
        Serial.print(cap, 6); Serial.print(",");
        if (leak > 1e11) Serial.print("INF"); else Serial.print(leak / 1e6f, 2);
        Serial.println(",OK");
    } else Serial.println("FAIL,0,0,FIT_FAIL");
    delay(3000);
}
