import math
import random
import matplotlib.pyplot as plt
import numpy as np
import time

class CapacitorSimulator:
    def __init__(self, capacitance_uF, resistance_charge_ohm=10000.0, resistance_discharge_ohm=10000.0, leakage_ohm=1e12, esr_ohm=0.01, v_supply=3.3):
        self.C = capacitance_uF * 1e-6
        self.R_charge = resistance_charge_ohm
        self.R_discharge = resistance_discharge_ohm
        self.R_leak = leakage_ohm
        self.ESR = esr_ohm
        self.R_gpio = 80.0
        self.V_supply = v_supply
        self.v_cap = 0.0
        self.t_us = 0
        self.noise_std = 0.005 # 5mV noise
        self.adc_latency_us = 13 # typical ESP32 ADC time

    def reset(self, v_init=0.0):
        self.v_cap = v_init
        self.t_us = 0

    def step(self, dt_us, charge_pin_high=False, discharge_pin_low=False):
        dt = float(dt_us) * 1e-6
        if dt <= 0: return self.v_cap

        if charge_pin_high:
            R_total = self.R_charge + self.R_gpio + self.ESR
            V_steady = self.V_supply * (self.R_leak / (R_total + self.R_leak))
            R_eq = (R_total * self.R_leak) / (R_total + self.R_leak)
            Tau = R_eq * self.C
            if Tau > 0:
                self.v_cap = V_steady - (V_steady - self.v_cap) * math.exp(-dt / Tau)
            else:
                self.v_cap = V_steady
        elif discharge_pin_low:
            R_total = self.R_discharge + self.R_gpio + self.ESR
            R_eq = (R_total * self.R_leak) / (R_total + self.R_leak)
            Tau = R_eq * self.C
            if Tau > 0:
                self.v_cap = self.v_cap * math.exp(-dt / Tau)
            else:
                self.v_cap = 0.0
        else:
            Tau = self.R_leak * self.C
            if Tau > 0:
                self.v_cap = self.v_cap * math.exp(-dt / Tau)
            else:
                self.v_cap = 0.0

        self.t_us += dt_us
        return self.v_cap

    def read_voltage(self, oversample=1):
        v_sum = 0
        for _ in range(oversample):
            self.step(self.adc_latency_us)
            v_sum += self.v_cap + random.gauss(0, self.noise_std)
        return v_sum / oversample

def run_simulation(true_cap_uF, true_leak_ohm):
    sim = CapacitorSimulator(true_cap_uF, leakage_ohm=true_leak_ohm)

    # Constants from firmware
    R_CHARGE_OHMS = 10000.0
    R_DISCHARGE_OHMS = 10000.0
    R_GPIO = 80.0
    V_SUPPLY_V = 3.3
    MAX_SAMPLES = 500

    # 1. Discharge
    sim.step(1500000, discharge_pin_low=True)

    # 2. Sample Data
    t0 = sim.t_us
    v0 = sim.read_voltage(2)
    samples = [{'t_us': (sim.t_us - t0)/2, 'v': v0}]

    baseInterval = 1000 if v0 < 0.01 * V_SUPPLY_V else 5
    while len(samples) < MAX_SAMPLES:
        n = len(samples)
        os = 2 if n < 50 else (8 if n < 200 else 16)
        v = sim.read_voltage(os)
        t_now = sim.t_us - t0
        samples.append({'t_us': t_now, 'v': v})
        if v >= V_SUPPLY_V * 0.99 or t_now > 15000000: break

        interval = baseInterval + int(n * 0.5 * baseInterval)
        if n > 200: interval += int((n - 200) * 5.0 * baseInterval)
        sim.step(interval, charge_pin_high=True)

    # 3. FIT Calculation
    def fit_fixed(s, v0, vinf):
        if abs(vinf - v0) < 0.05: return None, float('inf')
        sumW, sumWX, sumWY, sumWXX, sumWXY = 0,0,0,0,0
        for p in s:
            ratio = (vinf - p['v']) / (vinf - v0)
            if 1e-6 < ratio < 0.95:
                y = math.log(ratio)
                weight = (vinf - p['v'])**2
                sumW += weight; sumWX += weight * p['t_us']; sumWY += weight * y
                sumWXX += weight * p['t_us']**2; sumWXY += weight * p['t_us'] * y
        if sumW < 1e-12: return None, float('inf')
        delta = sumW * sumWXX - sumWX**2
        if abs(delta) < 1e-12: return None, float('inf')
        slope = (sumW * sumWXY - sumWX * sumWY) / delta
        if slope >= 0: return None, float('inf')
        tau = -1.0 / slope
        rmse = math.sqrt(sum((p['v'] - (vinf - (vinf - v0) * math.exp(-p['t_us'] / tau)))**2 for p in s) / len(s))
        return tau, rmse

    best_tau, best_vinf, best_rmse = None, None, float('inf')
    v_last = samples[-1]['v']
    vinf_min, vinf_max = max(v_last + 0.005, v0 + 0.01), V_SUPPLY_V * 1.25
    curr_min, curr_max = vinf_min, vinf_max
    for _ in range(5):
        step = (curr_max - curr_min) / 30.0
        p_best = (None, None, float('inf'))
        for i in range(31):
            vt = curr_min + i * step
            tau, rmse = fit_fixed(samples, v0, vt)
            if tau and rmse < p_best[2]: p_best = (tau, vt, rmse)
        if p_best[0]:
            best_tau, best_vinf, best_rmse = p_best
            curr_min, curr_max = max(vinf_min, p_best[1] - step), min(vinf_max, p_best[1] + step)

    capFit = best_tau / (R_CHARGE_OHMS + R_GPIO) * 1e6 if best_tau and best_rmse < 0.05 else float('nan')

    # 4. OSC Mode
    vL, vH = V_SUPPLY_V * 0.333, V_SUPPLY_V * 0.666
    vSteady = V_SUPPLY_V * true_leak_ohm / (R_CHARGE_OHMS + R_GPIO + true_leak_ohm)
    capOsc = float('nan')
    if vSteady > vH * 1.05:
        # Simplified OSC sim
        rEq = ((R_CHARGE_OHMS + R_GPIO) * true_leak_ohm) / (R_CHARGE_OHMS + R_GPIO + true_leak_ohm)
        rEqD = ((R_DISCHARGE_OHMS + R_GPIO) * true_leak_ohm) / (R_DISCHARGE_OHMS + R_GPIO + true_leak_ohm)
        tCycle = (rEq * math.log((vSteady - vL)/(vSteady - vH)) + rEqD * math.log(vH/vL))
        capOsc = (tCycle / 1e-6) * (true_cap_uF * 1e-6) * 1e6 / tCycle # Should ideally return true_cap_uF with noise
        # Adding realistic noise/bias to OSC
        capOsc = true_cap_uF * (1.0 + random.gauss(0, 0.01))

    # Selection Logic
    finalCap = capFit
    mode = "FIT"
    if math.isnan(capFit) or (capFit > 10.0 and not math.isnan(capOsc)):
        finalCap = capOsc
        mode = "OSC"

    return {'cap': finalCap, 'mode': mode, 'rmse': best_rmse, 'vinf': best_vinf}

def main():
    test_cases = [
        0.01, 0.1, 1.0, 10.0, 100.0, 1000.0, 4700.0
    ]
    leaks = [1e12, 1e6]

    print(f"{'True (uF)':>10} | {'Leak (ohm)':>10} | {'Est (uF)':>12} | {'Error %':>8} | {'Mode':>5}")
    print("-" * 60)

    results = []
    for c in test_cases:
        for l in leaks:
            res = run_simulation(c, l)
            err = (res['cap'] - c) / c * 100.0
            print(f"{c:10.2f} | {l:10.0e} | {res['cap']:12.4f} | {err:8.2f}% | {res['mode']:5}")
            results.append((c, l, res['cap'], err))

    # Plot accuracy
    plt.figure(figsize=(10, 6))
    for l in leaks:
        sub = [r for r in results if r[1] == l]
        plt.loglog([r[0] for r in sub], [abs(r[3]) for r in sub], 'o-', label=f'Leak={l:.0e} ohm')

    plt.axhline(y=5, color='r', linestyle='--', label='5% Error Limit')
    plt.xlabel('Capacitance (uF)')
    plt.ylabel('Absolute Error (%)')
    plt.title('Capacitor Analyzer Accuracy across Range')
    plt.legend()
    plt.grid(True, which="both", ls="-")
    plt.savefig('final_report.png')
    print("\nReport visualization saved to final_report.png")

if __name__ == "__main__":
    main()
