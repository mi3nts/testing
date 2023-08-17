#include <Arduino.h>

const int analogPin = A1;   // Analog pin to output simulated voltage
const int analogPin2 = A2;   // Analog pin to output simulated voltage
const float targetVoltage = 3.3; // Desired output voltage in volts
int mapVoltage(float voltage) {
    // Map the desired voltage to a PWM value (0-255)
    // Assumes a 5V reference voltage for the PWM output
    return map(voltage, 0, 5, 0, 255);
}
void setup() {
    pinMode(analogPin, OUTPUT);
}

void loop() {
    int pwmValue = mapVoltage(targetVoltage); // Calculate the PWM value for the desired voltage
    analogWrite(analogPin, pwmValue);         // Output the PWM value
    delay(10);       
    analogWrite(analogPin, 0);         // Output the PWM value
    delay(5000); 
    analogWrite(analogPin2, pwmValue);         // Output the PWM value
    delay(10);       
    analogWrite(analogPin2, 0);         // Output the PWM value
    delay(5000);    
    }

// const int heartbeatPin = 2;  // Pin connected to the heartbeat signal
// const int heartbeatTimeout = 3000;  // Timeout value in milliseconds

// unsigned long lastHeartbeatTime = 0;
// bool lastHeartbeatState = LOW;

// void setup() {
//   pinMode(heartbeatPin, INPUT);
//   Serial.begin(9600);
  
//   // Initialize the watchdog timer with a timeout of 5 seconds
//   wdt_enable(WDTO_8S);
// }

// void loop() {
//   int currentHeartbeatState = digitalRead(heartbeatPin);
//   unsigned long currentTime = millis();
  
//   // Check if the heartbeat signal has toggled since the last read
//   if (currentHeartbeatState != lastHeartbeatState) {
//     lastHeartbeatState = currentHeartbeatState;
//     lastHeartbeatTime = currentTime;
//   }
  
//   // Check for missing heartbeat
//   if (currentTime - lastHeartbeatTime > heartbeatTimeout) {
//     // Heartbeat missing, take appropriate action (e.g., trigger alert)
//     Serial.println("Heartbeat missing! Taking action...");
//   }
  
//   // Reset the watchdog timer
//   wdt_reset();

//   // Your main program logic here
// }

