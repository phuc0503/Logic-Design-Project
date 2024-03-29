#include <Arduino.h>
#include <SimpleKalmanFilter.h>
#include <Wire.h>
#include <math.h>

float pi = 3.14159265358979323846;

float notchFrequency = (50.0F); // Adjust this to the frequency you want to notch out (e.g., 50.0 Hz for 50 Hz power line interference)

// Notch filter variables
float notchX1 = 0.0;
float notchX2 = 0.0;
float notchY1 = 0.0;
float notchY2 = 0.0;
const float notchQ = 0.7071; // Quality factor of the notch filter (adjust this as needed)

int applyNotchFilter(int input) {
  // Notch filter implementation
  float omega = ((2.0 * pi) * notchFrequency) / 1000.0; // Convert notch frequency from Hz to radians/ms
  float sn = sin(omega);
  float cs = cos(omega);
  float alpha = sn / (2.0 * notchQ);

  float a0 = 1.0 + alpha;
  float b0 = 1.0;
  float b1 = -2.0 * cs;
  float b2 = 1.0;
  float a1 = -2.0 * cs;
  float a2 = 1.0 - alpha;

  float output = (b0 / a0) * input + (b1 / a0) * notchX1 + (b2 / a0) * notchX2 - (a1 / a0) * notchY1 - (a2 / a0) * notchY2;

  notchX2 = notchX1;
  notchX1 = input;
  notchY2 = notchY1;
  notchY1 = output;

  return int(output + 0.5); // Convert back to integer
}

//  Variables
int PulseSensorPin = 12;        // Pulse Sensor PURPLE WIRE connected to ANALOG PIN 12

const float rThreshold = 0.7;
const float decayRate = 0.01;
const float thrRate = 0.05;
const int minDiff = 5;

// Current values
float maxValue = 0;
float minValue = 4095;
float threshold = 2020;

// Timestamp of the last heartbeat
long lastHeartbeat = 0;

// Last value to detect crossing the threshold
int lastValue = 4095;

int heartRate(int currentValue){
  maxValue = (maxValue > currentValue) ? maxValue : currentValue;
  minValue = (minValue < currentValue) ? minValue : currentValue;

  // Detect Heartbeat
  float nthreshold = (maxValue - minValue) * rThreshold + minValue;
  threshold = threshold * (1-thrRate) + nthreshold * thrRate;
  threshold = min(maxValue, max(minValue, threshold));
  
  if(currentValue >= threshold && lastValue < threshold && (maxValue-minValue) > minDiff && millis() - lastHeartbeat > 300){
          
      if(lastHeartbeat != 0) {
      // Show Results
      int bpm = 60000/(millis() - lastHeartbeat);
      if(bpm > 50 && bpm < 250) {
          Serial.print("Heart Rate (bpm): ");
          Serial.println(bpm);

          if(bpm >= 130){
          Serial.println("HR too high!!!");
          digitalWrite(18, HIGH);
          }else{
          digitalWrite(18, LOW);
          }
      }
      }
      lastHeartbeat = millis();
  }

  // Decay for max/min
  maxValue -= (maxValue-currentValue)*decayRate;
  minValue += (currentValue-minValue)*decayRate;

  lastValue = currentValue;
  return currentValue;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(18, OUTPUT);
  Serial.begin(9600);         // Set's up Serial Communication at certain speed.
}

void loop() {
  // put your main code here, to run repeatedly:
  int currentValue = heartRate(applyNotchFilter(analogRead(PulseSensorPin)));
  

  //Plot
  Serial.print(">No filter:");
  Serial.println(currentValue);

  delay(20);
  
}