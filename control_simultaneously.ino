#include <ESP32Servo.h>

#define MotorPin1 12    // First BLDC motor pin
#define MotorPin2 14    // Second BLDC motor pin
#define SerialMonitor 9600
#define CCW_MAX_BLDC_Speed 1860     // Counter-Clockwise maximum
#define CW_MAX_BLDC_Speed 1060      // Clockwise maximum
#define BLDC_Center 1500            // Stop position

volatile unsigned int userSpeed = 0;

class BLDC_Motor {
  Servo servo;
  int updateInterval;
  volatile int pos;
  volatile unsigned long lastUpdate;
  volatile int increment;

public:
  BLDC_Motor(int interval) {
    updateInterval = interval;
    increment = 50;
    pos = BLDC_Center;  // Initialize position
    lastUpdate = 0;     // Initialize timer
  }
  
  void Attach(int pin) {
    servo.attach(pin, 1000, 2000);
    servo.write(BLDC_Center);  // Set initial position
  }

  void reset() {
    pos = BLDC_Center;
    servo.write(pos);
    increment = abs(increment);
  }

  void setSpeed(int speed) {
    pos = speed;  // Directly set the position
    servo.write(pos);
  }

  void Update(unsigned long currentMillis, volatile unsigned int targetSpeed) {
    if ((currentMillis - lastUpdate) > updateInterval) {
      lastUpdate = currentMillis;
      if (pos < targetSpeed) {
        pos += increment;
      } else if (pos > targetSpeed) {
        pos -= increment;
      }
      servo.write(pos);
    }
  }
};

// Create motor objects
BLDC_Motor bldcMotor1(50);
BLDC_Motor bldcMotor2(50);

void setup() {
  Serial.begin(SerialMonitor);
  
  // ESP32 timer allocation
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  bldcMotor1.Attach(MotorPin1);
  bldcMotor2.Attach(MotorPin2);

  delay(2000);
  Serial.println("Synchronized BLDC Motor Control Ready");
  Serial.println("Use command: 'MOTOR speed'");
  Serial.println("Speed range: 1060-1860");
}

void loop() {
  if (Serial.available()) {
    String received_data = Serial.readStringUntil('\n');
    processCommand(received_data);
  }

  unsigned long currentMillis = millis();
  
  // Update all motors with the same speed
  if (userSpeed) {
    bldcMotor1.Update(currentMillis, userSpeed);
    bldcMotor2.Update(currentMillis, userSpeed);
  }
}

void processCommand(String cmd) {
  cmd.trim();  // Remove any whitespace
  
  if (cmd.startsWith("MOTOR")) {
    int speed = cmd.substring(6).toInt();
    userSpeed = constrain(speed, CW_MAX_BLDC_Speed, CCW_MAX_BLDC_Speed);
    
    // Directly set the initial speed
    bldcMotor1.setSpeed(userSpeed);
    bldcMotor2.setSpeed(userSpeed);
    
    Serial.print("All Motors Speed Set To: ");
    Serial.println(userSpeed);
  }
}