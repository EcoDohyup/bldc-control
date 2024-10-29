#include <ESP32Servo.h>

// Pin Definitions
#define MotorPin1 13    // First BLDC motor pin
#define MotorPin2 12    // Second BLDC motor pin
#define SerialMonitor 9600
#define CCW_MAX_BLDC_Speed 1860     // Counter-Clockwise maximum
#define CW_MAX_BLDC_Speed 1060      // Clockwise maximum
#define BLDC_Center 1500            // Stop position

// Task handles
TaskHandle_t motor1TaskHandle = NULL;
TaskHandle_t motor2TaskHandle = NULL;
TaskHandle_t serialTaskHandle = NULL;

// Queue handles for motor commands
QueueHandle_t motor1Queue;
QueueHandle_t motor2Queue;

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
    }
  }
};

// Create motor objects
BLDC_Motor bldcMotor1;
BLDC_Motor bldcMotor2;

// Task function for Motor 1
void Motor1Task(void * parameter) {
  int speed = 0;
  while(true) {
    if (xQueueReceive(motor1Queue, &speed, 0) == pdTRUE) {
      Serial.print("Motor1 Task received speed: ");
      Serial.println(speed);
      bldcMotor1.setSpeed(speed);
    }
    vTaskDelay(pdMS_TO_TICKS(20));  // 20ms delay
  }
}

// Task function for Motor 2
void Motor2Task(void * parameter) {
  int speed = 0;
  while(true) {
    if (xQueueReceive(motor2Queue, &speed, 0) == pdTRUE) {
      Serial.print("Motor2 Task received speed: ");
      Serial.println(speed);
      bldcMotor2.setSpeed(speed);
    }
    vTaskDelay(pdMS_TO_TICKS(20));  // 20ms delay
  }
}

// Task function for Serial Communication
void SerialTask(void * parameter) {
  while(true) {
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
          xQueueSend(motor1Queue, &speed, portMAX_DELAY);
        }
      }
      else if (command.startsWith("MOTOR2")) {
        int speed = command.substring(6).toInt();
        Serial.print("Motor 2 speed parsed: ");
        Serial.println(speed);
        if (speed >= CW_MAX_BLDC_Speed && speed <= CCW_MAX_BLDC_Speed) {
          xQueueSend(motor2Queue, &speed, portMAX_DELAY);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));  // 20ms delay
  }
}

void setup() {
  Serial.begin(SerialMonitor);
  while (!Serial) {
    ; // wait for serial port to connect
  }
  
  // Create queues for motor commands
  motor1Queue = xQueueCreate(5, sizeof(int));
  motor2Queue = xQueueCreate(5, sizeof(int));
  
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

  // Create tasks
  xTaskCreatePinnedToCore(
    Motor1Task,
    "Motor1Task",
    10000,
    NULL,
    1,
    &motor1TaskHandle,
    0  // Core 0
  );

  xTaskCreatePinnedToCore(
    Motor2Task,
    "Motor2Task",
    10000,
    NULL,
    1,
    &motor2TaskHandle,
    0  // Core 0
  );

  xTaskCreatePinnedToCore(
    SerialTask,
    "SerialTask",
    10000,
    NULL,
    2,  // Higher priority for serial
    &serialTaskHandle,
    1  // Core 1
  );
}

void loop() {
  // Empty loop as all work is done in tasks
  vTaskDelay(portMAX_DELAY);
}