# CapTester: Advanced ESP32-based Capacitor Analyzer

CapTester is a high-precision firmware suite for measuring capacitance and leakage resistance using an ESP32. It employs multiple measurement techniques to cover a wide range of capacitors (10nF to 10mF) with high accuracy and noise resilience.

## Features
- **Multi-Mode Parameter Inference:** Automatic switching between curve fitting (FIT), relaxation oscillation (OSC), and $dV/dt$ tracking (GAIN) for various capacitor sizes.
- **Asymptote Optimization:** Iterative 5-pass search for the plateau voltage ($V_\infty$) to eliminate bias in exponential curve fitting.
- **Leakage Estimation:** Uses high-impedance self-discharge rate measurement to calculate parallel leakage resistance ($R_{leak}$).
- **Hardware Compensation:** Accounts for GPIO internal resistance ($\approx 80\Omega$) and ADC latency for refined time-constant calculations.
- **Noise Rejection:** Dynamic 8x–16x oversampling and midpoint-timestamping to minimize jitter and quantization error.

## Firmware Versions
1. **`simple.ino`:** A straightforward 1-Tau timer with 50µs pulse detection for small capacitors.
2. **`simple_verify.ino`:** Adds a secondary discharge-based verification step to ensure measurement consistency.
3. **`verify2.ino`:** The primary analyzer, featuring the full multi-mode inference engine and leakage detection.

## Hardware Configuration
| Function      | Pin  | Note                               |
|---------------|------|------------------------------------|
| Charge Pin    | 5    | Connect via 10k$\Omega$ resistor. |
| Discharge Pin | 18   | Connect via 10k$\Omega$ resistor. |
| Measure Pin   | 34   | ADC node (ESP32).                  |
| VCC           | 3.3V |                                    |

*Ensure the capacitor under test is fully discharged before starting.*

## Simulation & Validation
The project includes a Python-based RC circuit simulator for iterative testing of the firmware algorithms without hardware.
- **`final_verify.py`:** Benchmarks all measurement modes across a 10nF to 4.7mF range, ensuring errors remain within $\pm 2\%$.
- **`test_system.py`:** Generates visualizations of the charging curves and fits.

## Accuracy Report
Recent simulation results across 4 orders of magnitude:
| True (uF) | Mode | Error (%) |
|-----------|------|-----------|
| 0.01      | OSC  | +1.6%     |
| 1.0       | OSC  | +0.5%     |
| 100.0     | OSC  | +0.1%     |
| 4700.0    | OSC  | -0.9%     |

*Note: OSC mode provides high precision for most electrolytic and ceramic capacitors; FIT mode provides rapid initial estimates.*
