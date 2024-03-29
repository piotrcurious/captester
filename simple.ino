#include <Arduino.h>

const int chargePin = 5; // Pin used to charge the capacitor
const int dischargePin = 18; // Pin used to discharge the capacitor
const int measurePin = 34; // Analog pin used to measure the voltage across the capacitor
const double supplyVoltage = 3.3; // ESP32 supply voltage
const double resistorValue = 10000.0; // Resistor value in ohms
const double targetVoltage = supplyVoltage * 0.8; // 80% of supply voltage
const double calibrationVoltage = supplyVoltage * 0.5; // 50% of supply voltage for calibration

void setup() {
  Serial.begin(115200);
  pinMode(chargePin, INPUT);
  pinMode(dischargePin, INPUT);
  analogReadResolution(12); // Set the resolution to 12 bits for ESP32
  Serial.println("PulseLength(us),Capacitance(uF)");
}

void loop() {
  // Calibration to find the maximum pulse length
  int maxPulseLength = calibrateCapacitor();
  Serial.print("Maximum Pulse Length(us): ");
  Serial.println(maxPulseLength);

  for (int pulseLength = maxPulseLength; pulseLength > 0; pulseLength -= 100000) {
    int pulseCount = 0; // Reset pulse count for each pulse length
    double voltage = 0; // Reset voltage for each pulse length

    // Set charge pin to OUTPUT to start charging
    pinMode(chargePin, OUTPUT);
    // Charge the capacitor with pulses and count the number of pulses needed to reach 80% of the supply voltage
    while (voltage < targetVoltage) {
      digitalWrite(chargePin, HIGH);
      delayMicroseconds(pulseLength);
      digitalWrite(chargePin, LOW);
      pinMode(chargePin, INPUT); // Set charge pin to INPUT after charging

      voltage = analogRead(measurePin) * (supplyVoltage / 4095.0);
      pulseCount++;
    }

    // Discharge the capacitor
    pinMode(dischargePin, OUTPUT);
    digitalWrite(dischargePin, LOW);
    delay(1000); // Wait 1 second to ensure the capacitor is fully discharged
    pinMode(dischargePin, INPUT); // Set discharge pin to INPUT after discharging

    // Calculate the capacitance
    double capacitance = calculateCapacitance(pulseLength, pulseCount, resistorValue);

    // Output the results in CSV format
    Serial.print(pulseLength);
    Serial.print(",");
    Serial.println(capacitance, 6);

    delay(5000); // Wait for 5 seconds before the next measurement
  }
}

int calibrateCapacitor() {
  int pulseLength = 1000; // Start with a small pulse length
  double voltage = 0;

  // Increase pulse length until the capacitor charges to the calibration voltage
  while (voltage < calibrationVoltage) {
    pinMode(chargePin, OUTPUT);
    digitalWrite(chargePin, HIGH);
    delayMicroseconds(pulseLength);
    digitalWrite(chargePin, LOW);
    pinMode(chargePin, INPUT);

    voltage = analogRead(measurePin) * (supplyVoltage / 4095.0);
    if (voltage < calibrationVoltage) {
      pulseLength += 1000; // Increase pulse length
    }
  }
  return pulseLength;
}

double calculateCapacitance(int pulseLength, int pulseCount, double resistorValue) {
  // Calculate the charge time in microseconds
  double chargeTime = static_cast<double>(pulseLength) * pulseCount;

  // Calculate the capacitance using the formula Q = C * V
  // Rearrange to find C: C = Q / V
  // Where Q is the charge (I * t) and V is the target voltage (80% of supply voltage)
  double current = supplyVoltage / resistorValue;
  double capacitance = (current * chargeTime) / (targetVoltage * 1000000.0); // Convert charge time from us to s

  return capacitance * 1000000.0; // Convert F to uF
}
