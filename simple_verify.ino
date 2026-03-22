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

  // Check if capacitor charges too fast
  pinMode(chargePin, OUTPUT);
  digitalWrite(chargePin, HIGH);
  unsigned long t0 = micros();
  delayMicroseconds(50);
  double vCheck = (double)analogRead(measurePin) * (supplyVoltage / 4095.0);

  double calculatedCapacitance;
  if (vCheck >= 0.1 * supplyVoltage) {
      calculatedCapacitance = (50.0e-6 / (resistorValue * -log(1.0 - vCheck/supplyVoltage))) * 1000000.0;
  } else {
      double voltage = vCheck;
      while (voltage < targetVoltage) {
        uint32_t sum = 0;
        for(int i=0; i<8; i++) sum += analogRead(measurePin);
        voltage = ((double)sum / 8.0) * (supplyVoltage / 4095.0);
        if (micros() - t0 > 60000000) break;
        yield();
      }
      unsigned long endTime = micros();
      double chargeTime_s = (double)(endTime - t0) / 1000000.0;
      calculatedCapacitance = (chargeTime_s / resistorValue) * 1000000.0;
  }

  // Verification: use a secondary measurement (discharge time to 1/e)
  // Ensure it is fully charged before verifying
  while (analogRead(measurePin) * (supplyVoltage / 4095.0) < targetVoltage * 1.5) {
      digitalWrite(chargePin, HIGH);
      delay(1);
      if (millis() % 1000 == 0) break;
  }
  pinMode(chargePin, INPUT);
  digitalWrite(chargePin, LOW);

  double verifiedCapacitance = finalVerificationTest();
  double errorPercentage = (verifiedCapacitance - calculatedCapacitance) / calculatedCapacitance * 100.0;

  Serial.print("N/A,");
  Serial.print(calculatedCapacitance, 6);
  Serial.print(",");
  Serial.println(errorPercentage, 2);

  delay(5000);
}

double finalVerificationTest() {
  const double startV = analogRead(measurePin) * (supplyVoltage / 4095.0);
  const double thresholdV = startV / exp(1.0);

  pinMode(dischargePin, OUTPUT);
  digitalWrite(dischargePin, LOW);
  unsigned long startTime = micros();

  double voltage = startV;
  while (voltage > thresholdV) {
    uint32_t sum = 0;
    for(int i=0; i<8; i++) sum += analogRead(measurePin);
    voltage = ((double)sum / 8.0) * (supplyVoltage / 4095.0);
    if (micros() - startTime > 60000000) break;
    yield();
  }
  unsigned long endTime = micros();
  pinMode(dischargePin, INPUT);

  double dischargeTime_s = (double)(endTime - startTime) / 1000000.0;
  double verifiedCap_uF = (dischargeTime_s / resistorValue) * 1000000.0;

  return verifiedCap_uF;
}
