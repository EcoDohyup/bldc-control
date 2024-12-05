#include <ESP32Servo.h>

#define MotorPin1 13    // ESP32 pin for BLDC control
#define SerialMonitor 9600
#define CCW_MAX_BLDC_Speed 1860     // Counter-Clockwise maximum
#define CW_MAX_BLDC_Speed 1060      // Clockwise maximum
#define BLDC_Center 1500            // Stop position

volatile unsigned int userSpeed1 = 0;

class BLDC_Motor1 {
  Servo servo1;
  int updateInterval;
  volatile int pos = BLDC_Center;
  volatile unsigned long lastUpdate;
  volatile int increment;

public:
  BLDC_Motor1(int interval) {
    updateInterval = interval;
    increment = 50;
  }
  
  void Attach(int pin) {
    servo1.attach(pin, 1000, 2000);
  }

  void reset() {
    pos = BLDC_Center;
    servo1.write(pos);
    increment = abs(increment);
  }

  void Update(unsigned long currentMillis) {
    if ((currentMillis - lastUpdate) > updateInterval) {
      lastUpdate = currentMillis;
      if (pos < userSpeed1) {
        pos += increment;
      } else if (pos > userSpeed1) {
        pos -= increment;
      }
      servo1.write(pos);
    }
  }
};

BLDC_Motor1 bldcMotor1(50);

void setup() {
  Serial.begin(SerialMonitor);
  
  // ESP32 timer allocation
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  bldcMotor1.Attach(MotorPin1);

  delay(2000);
  Serial.println("BLDC Motor Control Ready");
}

void loop() {
  if (Serial.available()) {
    String received_data = Serial.readStringUntil('\n');
    processCommand(received_data);
  }

  unsigned long currentMillis1 = millis();
  if (userSpeed1) {
    bldcMotor1.Update(currentMillis1);
  }
}

void processCommand(String cmd) {
  if (cmd.startsWith("MOTOR1")) {
    int speed = cmd.substring(7).toInt();
    userSpeed1 = constrain(speed, CW_MAX_BLDC_Speed, CCW_MAX_BLDC_Speed);
    Serial.print("Motor 1 Speed Set To: ");
    Serial.println(userSpeed1);
  }
}