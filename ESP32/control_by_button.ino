#include <ESP32Servo.h>

// Pin Definitions
#define MotorPin1 13        // ESP32 pin for BLDC control
#define BUTTON1_PIN 22      // First button pin (GPIO22)
#define BUTTON2_PIN 23      // Second button pin (GPIO23)
#define SerialMonitor 9600

// BLDC Speed Constants
#define CCW_MAX_BLDC_Speed 1860     // Counter-Clockwise maximum
#define CW_MAX_BLDC_Speed 1060      // Clockwise maximum
#define BLDC_Center 1500            // Stop position
#define SPEED_INCREMENT 50          // Speed change increment

volatile int currentSpeed = BLDC_Center;
unsigned long lastDebounceTime = 0;
#define DEBOUNCE_DELAY 50

bool button1Pressed = false;
bool button2Pressed = false;

class BLDC_Motor1 {
  Servo servo1;

public:
  BLDC_Motor1() {}
  
  void Attach(int pin) {
    servo1.attach(pin, 1000, 2000);
    servo1.write(BLDC_Center);
  }

  void setSpeed(int speed) {
    if (speed >= CW_MAX_BLDC_Speed && speed <= CCW_MAX_BLDC_Speed) {
      servo1.write(speed);
    }
  }
};

BLDC_Motor1 bldcMotor1;

void setup() {
  Serial.begin(SerialMonitor);
  
  // Pin Setup
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  
  // ESP32 timer allocation
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  bldcMotor1.Attach(MotorPin1);
  
  delay(2000);
  Serial.println("System Ready");
  Serial.print("Initial Speed: ");
  Serial.println(currentSpeed);
}

void loop() {
  // Debug button states
  static int lastB1 = HIGH;
  static int lastB2 = HIGH;
  int b1 = digitalRead(BUTTON1_PIN);
  int b2 = digitalRead(BUTTON2_PIN);
  
  // Print only on state change
  if(b1 != lastB1) {
    Serial.print("Button 1 state: ");
    Serial.println(b1);
    lastB1 = b1;
  }
  if(b2 != lastB2) {
    Serial.print("Button 2 state: ");
    Serial.println(b2);
    lastB2 = b2;
  }

  if (b1 == LOW && !button1Pressed) {
    if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
      if (currentSpeed < CCW_MAX_BLDC_Speed) {
        currentSpeed += SPEED_INCREMENT;
        Serial.print("Button 1 pressed. New speed: ");
        Serial.println(currentSpeed);
        bldcMotor1.setSpeed(currentSpeed);
      }
      button1Pressed = true;
      lastDebounceTime = millis();
    }
  }
  else if (b1 == HIGH) {
    button1Pressed = false;
  }
  
  if (b2 == LOW && !button2Pressed) {
    if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
      if (currentSpeed > CW_MAX_BLDC_Speed) {
        currentSpeed -= SPEED_INCREMENT;
        Serial.print("Button 2 pressed. New speed: ");
        Serial.println(currentSpeed);
        bldcMotor1.setSpeed(currentSpeed);
      }
      button2Pressed = true;
      lastDebounceTime = millis();
    }
  }
  else if (b2 == HIGH) {
    button2Pressed = false;
  }
}