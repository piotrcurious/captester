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
static void releaseAllPins() { pinMode(chargePin, INPUT); pinMode(dischargePin, INPUT); }
static void startCharge() { pinMode(dischargePin, INPUT); pinMode(chargePin, OUTPUT); digitalWrite(chargePin, HIGH); }
static void stopCharge() { pinMode(chargePin, INPUT); digitalWrite(chargePin, LOW); }
static void startDischarge() { pinMode(chargePin, INPUT); pinMode(dischargePin, OUTPUT); digitalWrite(dischargePin, LOW); }
static void stopDischarge() { pinMode(dischargePin, INPUT); }

// Method 1: Exponential Fit
struct FitResult { bool ok = false; float tau_us = NAN; float vinf = NAN; };
static FitResult fitExponential(const Sample *s, size_t n) {
    FitResult r;
    if (n < 10) return r;
    const float v0 = s[0].v;
    float vinf = V_SUPPLY_V;
    if (n > 100) {
        float dv = s[n-1].v - s[n-21].v;
        uint32_t dt = s[n-1].t_us - s[n-21].t_us;
        if (dt > 0 && (dv/dt) < 0.000002f && s[n-1].v < 0.98f * V_SUPPLY_V) vinf = s[n-1].v;
    }
    if (abs(vinf - v0) < 0.05f) return r;
    double sumX = 0, sumY = 0, sumXX = 0, sumXY = 0;
    size_t used = 0;
    for (size_t i = 0; i < n; ++i) {
        float frac = (s[i].v - v0) / (vinf - v0);
        float ratio = (vinf - s[i].v) / (vinf - v0);
        if (frac > 0.1f && frac < 0.9f && ratio > 0.0f) {
            sumX += s[i].t_us; sumY += log(ratio); sumXX += (double)s[i].t_us * s[i].t_us; sumXY += (double)s[i].t_us * log(ratio);
            used++;
        }
    }
    if (used < 3) return r;
    double slope = (used * sumXY - sumX * sumY) / (used * sumXX - sumX * sumX);
    if (slope >= 0) return r;
    r.ok = true; r.tau_us = -1.0f / (float)slope; r.vinf = vinf;
    return r;
}

// Method 2: Virtual Feedback Leakage Test
static float measureLeakageFeedback(float targetV = 1.65f, uint32_t duration_ms = 300) {
    uint32_t tS = millis();
    while(readVoltageOnce() < targetV && (millis() - tS) < 5000) {
        startCharge(); delay(1); stopCharge(); yield();
    }
    while(readVoltageOnce() > targetV + 0.05f && (millis() - tS) < 6000) {
        startDischarge(); delay(1); stopDischarge(); yield();
    }
    uint32_t high_us = 0;
    uint32_t tLoopStart = micros();
    uint32_t tDeadline = millis() + duration_ms;
    while(millis() < tDeadline) {
        uint32_t t1 = micros();
        if (readVoltageOnce() < targetV) {
            startCharge(); delayMicroseconds(5);
            high_us += (micros() - t1);
        } else {
            stopCharge(); delayMicroseconds(5);
        }
    }
    uint32_t tLoopEnd = micros();
    float duty = (float)high_us / (tLoopEnd - tLoopStart);
    if (duty < 0.0001f) return 1e12f;
    return targetV * R_CHARGE_OHMS / ((V_SUPPLY_V - targetV) * duty);
}

// Method 3: Relaxation Oscillator
static float measureCapOscillator(float leak) {
    float vL = V_SUPPLY_V * 0.333f, vH = V_SUPPLY_V * 0.666f;
    float vSteady = V_SUPPLY_V * leak / (R_CHARGE_OHMS + leak);
    if (vSteady < vH * 1.05f) return NAN;
    startCharge(); while(readVoltageOnce() < vL) yield(); stopCharge();
    uint32_t tS = micros();
    for(int i=0; i<10; i++) {
        startCharge(); while(readVoltageOnce() < vH) yield(); stopCharge();
        startDischarge(); while(readVoltageOnce() > vL) yield(); stopDischarge();
    }
    float tCycle = (float)(micros() - tS) / 10.0f;
    float tcN = log((vSteady - vL) / (vSteady - vH)), tdN = log(vH / vL);
    float rEq = (R_CHARGE_OHMS * leak) / (R_CHARGE_OHMS + leak);
    return (tCycle * 1e-6f / (rEq * (tcN + tdN))) * 1e6f;
}

// Method 4: Virtual Feedback Gain (dV/dt)
static float measureCapFeedbackGain(float leak) {
    float vStart = 1.0f, vEnd = 1.2f;
    if (readVoltageOnce() > vStart) {
        startDischarge(); while(readVoltageOnce() > vStart * 0.95f) yield(); stopDischarge();
    }
    startCharge();
    while(readVoltageOnce() < vStart) yield();
    uint32_t t1 = micros();
    while(readVoltageOnce() < vEnd) yield();
    uint32_t t2 = micros();
    stopCharge();
    float dt = (float)(t2 - t1) / 1000000.0f, dv = vEnd - vStart;
    float vAvg = (vStart + vEnd) / 2.0f;
    float i_cap = ((V_SUPPLY_V - vAvg) / R_CHARGE_OHMS) - (vAvg / leak);
    return (i_cap * dt / dv) * 1e6f;
}

void setup() {
    Serial.begin(115200); releaseAllPins();
#if defined(ESP32)
    analogReadResolution(12);
#endif
    Serial.println("\n--- Capacitor Parameter Inference System ---");
    Serial.println("mode,cap_uF,leak_MOhm,status");
}

void loop() {
    size_t n = 0;
    startDischarge(); delay(1500); stopDischarge();
    startCharge();
    uint32_t t0 = micros(), interval = 5;
    delayMicroseconds(50);
    if (readVoltageOnce() < 0.01 * V_SUPPLY_V) interval = 1000;
    while (n < MAX_SAMPLES) {
        samples_buf[n++] = { micros() - t0, readVoltageOnce() };
        if (samples_buf[n-1].v >= V_SUPPLY_V * 0.95f || samples_buf[n-1].t_us > 10000000) break;
        if (n > 400) interval *= 2;
        uint32_t target = micros() + interval;
        while(micros() < target) yield();
    }
    stopCharge();

    FitResult fr = fitExponential(samples_buf, n);
    if (fr.ok) {
        float leak = measureLeakageFeedback();
        float cap = (fr.tau_us * 1e-6f * (R_CHARGE_OHMS + leak) / (R_CHARGE_OHMS * leak)) * 1e6f;
        const char* mode = "FIT";
        if (cap > 10.0f) {
            float capOsc = measureCapOscillator(leak);
            if (!isnan(capOsc)) { cap = capOsc; mode = "OSC"; }
        }
        if (cap > 1000.0f) {
            float capGain = measureCapFeedbackGain(leak);
            if (!isnan(capGain)) { cap = capGain; mode = "GAIN"; }
        }
        Serial.print(mode); Serial.print(",");
        Serial.print(cap, 6); Serial.print(",");
        if (leak > 1e11) Serial.print("INF"); else Serial.print(leak / 1e6f, 2);
        Serial.println(",OK");
    } else Serial.println("FAIL,0,0,TIMEOUT");
    delay(3000);
}
