#include <Arduino.h>
#include <math.h>

const int chargePin = 5;
const int dischargePin = 18;
const int measurePin = 34;
const double supplyVoltage = 3.3;
const double resistorValue = 10000.0;
const double targetVoltage = supplyVoltage * (1.0 - 1.0 / exp(1.0));

void setup() {
  Serial.begin(115200);
  pinMode(chargePin, INPUT);
  pinMode(dischargePin, INPUT);
  analogReadResolution(12);
  Serial.println("Capacitance(uF)");
}

void loop() {
  // Fully discharge
  pinMode(dischargePin, OUTPUT);
  digitalWrite(dischargePin, LOW);
  delay(2000);
  pinMode(dischargePin, INPUT);

  // Check if capacitor charges too fast
  pinMode(chargePin, OUTPUT);
  digitalWrite(chargePin, HIGH);
  unsigned long t0 = micros();
  delayMicroseconds(50);
  double vCheck = (double)analogRead(measurePin) * (supplyVoltage / 4095.0);

  double capacitance_uF;
  if (vCheck >= 0.1 * supplyVoltage) {
      // Small capacitor: use 50us data point
      capacitance_uF = (50.0e-6 / (resistorValue * -log(1.0 - vCheck/supplyVoltage))) * 1000000.0;
  } else {
      // Larger capacitor: continue charging to 1 Tau
      double voltage = vCheck;
      while (voltage < targetVoltage) {
        voltage = (double)analogRead(measurePin) * (supplyVoltage / 4095.0);
        if (micros() - t0 > 60000000) break;
      }
      unsigned long endTime = micros();
      double chargeTime_s = (double)(endTime - t0) / 1000000.0;
      capacitance_uF = (chargeTime_s / resistorValue) * 1000000.0;
  }

  pinMode(chargePin, INPUT);
  digitalWrite(chargePin, LOW);

  Serial.println(capacitance_uF, 6);
  delay(5000);
}
