#include <ESP32Servo.h>

// Pin Definitions
#define MotorPin1 13    // First BLDC motor pin
#define MotorPin2 12    // Second BLDC motor pin
#define SerialMonitor 9600
#define CCW_MAX_BLDC_Speed 1860     // Counter-Clockwise maximum
#define CW_MAX_BLDC_Speed 1060      // Clockwise maximum
#define BLDC_Center 1500            // Stop position

// Motor speed variables
volatile int motor1Speed = 0;
volatile int motor2Speed = 0;

class BLDC_Motor {
  Servo servo;

public:
  BLDC_Motor() {}
  
  void Attach(int pin) {
    servo.attach(pin, 1000, 2000);
    servo.write(BLDC_Center);
  }

  void setSpeed(int speed) {
    if (speed >= CW_MAX_BLDC_Speed && speed <= CCW_MAX_BLDC_Speed) {
      servo.write(speed);
      Serial.print("Motor speed set to: ");
      Serial.println(speed);
    }
  }
};

// Create motor objects
BLDC_Motor bldcMotor1;
BLDC_Motor bldcMotor2;

void setup() {
  Serial.begin(SerialMonitor);
  while (!Serial) {
    ; // wait for serial port to connect
  }
  
  // ESP32 timer allocation
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  // Initialize motors
  bldcMotor1.Attach(MotorPin1);
  bldcMotor2.Attach(MotorPin2);

  delay(2000);
  Serial.println("\nBLDC Motor Control Ready");
  Serial.println("Commands: 'MOTOR1 speed' or 'MOTOR2 speed'");
  Serial.println("Speed range: 1060-1860");
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    Serial.print("Received command: ");
    Serial.println(command);
    
    command.trim();
    
    if (command.startsWith("MOTOR1")) {
      int speed = command.substring(6).toInt();
      Serial.print("Motor 1 speed parsed: ");
      Serial.println(speed);
      if (speed >= CW_MAX_BLDC_Speed && speed <= CCW_MAX_BLDC_Speed) {
        motor1Speed = speed;
        bldcMotor1.setSpeed(speed);
      }
    }
    else if (command.startsWith("MOTOR2")) {
      int speed = command.substring(6).toInt();
      Serial.print("Motor 2 speed parsed: ");
      Serial.println(speed);
      if (speed >= CW_MAX_BLDC_Speed && speed <= CCW_MAX_BLDC_Speed) {
        motor2Speed = speed;
        bldcMotor2.setSpeed(speed);
      }
    }
  }
}