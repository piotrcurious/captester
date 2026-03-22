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
  Serial.println("PulseLength(us),Capacitance(uF),Error(%)");
}

void loop() {
  // Fully discharge
  pinMode(dischargePin, OUTPUT);
  digitalWrite(dischargePin, LOW);
  delay(2000);
  pinMode(dischargePin, INPUT);

  // Continuous charging
  pinMode(chargePin, OUTPUT);
  digitalWrite(chargePin, HIGH);
  unsigned long startTime = micros();

  double voltage = 0;
  while (voltage < targetVoltage) {
    voltage = analogRead(measurePin) * (supplyVoltage / 4095.0);
    if (micros() - startTime > 60000000) break;
  }
  unsigned long endTime = micros();
  pinMode(chargePin, INPUT);
  digitalWrite(chargePin, LOW);

  double chargeTime_s = (double)(endTime - startTime) / 1000000.0;
  double calculatedCapacitance = (chargeTime_s / resistorValue) * 1000000.0;

  // Verification: use a secondary measurement (discharge time to 1/e)
  double verifiedCapacitance = finalVerificationTest(calculatedCapacitance);
  double errorPercentage = (verifiedCapacitance - calculatedCapacitance) / calculatedCapacitance * 100.0;

  Serial.print("N/A,");
  Serial.print(calculatedCapacitance, 6);
  Serial.print(",");
  Serial.println(errorPercentage, 2);

  delay(5000);
}

double finalVerificationTest(double lastMeasured_uF) {
  const double startV = analogRead(measurePin) * (supplyVoltage / 4095.0);
  const double thresholdV = startV / exp(1.0);

  pinMode(dischargePin, OUTPUT);
  digitalWrite(dischargePin, LOW);
  unsigned long startTime = micros();

  double voltage = startV;
  while (voltage > thresholdV) {
    voltage = analogRead(measurePin) * (supplyVoltage / 4095.0);
    if (micros() - startTime > 60000000) break;
  }
  unsigned long endTime = micros();
  pinMode(dischargePin, INPUT);

  double dischargeTime_s = (double)(endTime - startTime) / 1000000.0;
  double verifiedCap_uF = (dischargeTime_s / resistorValue) * 1000000.0;

  return verifiedCap_uF;
}
