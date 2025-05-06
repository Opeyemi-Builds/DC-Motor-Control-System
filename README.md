# DC-Motor-Control-System
# DC Motor Control System using Arduino and PID

This project demonstrates a simple closed-loop control system using a DC motor, Arduino, and PID (Proportional-Integral-Derivative) control algorithm. It’s ideal for beginner electrical/control engineers who want hands-on experience with automation and feedback systems.

## Features
- Speed control using PID algorithm
- Real-time feedback with encoder or potentiometer
- Serial monitoring of output
- Adjustable setpoint and gain values

## Components Used
- Arduino Uno (or compatible board)
- L298N Motor Driver
- DC Motor
- Potentiometer (or rotary encoder)
- Power supply (9V or 12V)
- Jumper wires
- Breadboard

## Circuit Diagram
Coming soon (You can draw this with Fritzing or upload an image later).

## Arduino Code (Snippet)
```cpp
int motorPin = 9;
int sensorPin = A0;
int setPoint = 512;
float Kp = 1.5;
float error, output;

void setup() {
  pinMode(motorPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  int sensorValue = analogRead(sensorPin);
  error = setPoint - sensorValue;
  output = Kp * error;
  output = constrain(output, 0, 255);
  analogWrite(motorPin, output);
  Serial.println(sensorValue);
  delay(100);
}
