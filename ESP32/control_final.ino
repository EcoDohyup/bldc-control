// 2024. 11.11 작성
// 모터 4개 개별제어 + 동시제어 가능
// MOTOR1 1600 이런 식으로 개별 제어
// MOTORS 1700 이런 식으로 동시 제어

#include <ESP32Servo.h>

// Pin Definitions
#define MotorPin1 13    // First BLDC motor pin
#define MotorPin2 12    // Second BLDC motor pin
#define MotorPin3 21    // Third BLDC motor pin
#define MotorPin4 22    // Fourth BLDC motor pin
#define SerialMonitor 9600
#define CCW_MAX_BLDC_Speed 1860     // Counter-Clockwise maximum
#define CW_MAX_BLDC_Speed 1060      // Clockwise maximum
#define BLDC_Center 1500            // Stop position

// Task handles
TaskHandle_t motor1TaskHandle = NULL;
TaskHandle_t motor2TaskHandle = NULL;
TaskHandle_t motor3TaskHandle = NULL;
TaskHandle_t motor4TaskHandle = NULL;
TaskHandle_t serialTaskHandle = NULL;

// Queue handles for motor commands
QueueHandle_t motor1Queue;
QueueHandle_t motor2Queue;
QueueHandle_t motor3Queue;
QueueHandle_t motor4Queue;

class BLDC_Motor {
  Servo servo;

public:
  BLDC_Motor() {}
  
  void Attach(int pin) {
    servo.attach(pin, 1000, 2000);
    // Initialization sequence
    servo.write(BLDC_Center);
    delay(100);  // Small delay to ensure the command is processed
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
BLDC_Motor bldcMotor3;
BLDC_Motor bldcMotor4;

// Helper function to set speed for all motors
void setAllMotorsSpeed(int speed) {
    if (speed >= CW_MAX_BLDC_Speed && speed <= CCW_MAX_BLDC_Speed) {
        xQueueSend(motor1Queue, &speed, portMAX_DELAY);
        xQueueSend(motor2Queue, &speed, portMAX_DELAY);
        xQueueSend(motor3Queue, &speed, portMAX_DELAY);
        xQueueSend(motor4Queue, &speed, portMAX_DELAY);
        Serial.print("All Motors speed set to: ");
        Serial.println(speed);
    } else {
        Serial.println("Invalid speed for motors");
    }
}

// Task functions for each motor (Same as before...)
void Motor1Task(void * parameter) {
  int speed = BLDC_Center;
  xQueueSend(motor1Queue, &speed, 0);
  
  while(true) {
    if (xQueueReceive(motor1Queue, &speed, 0) == pdTRUE) {
      Serial.print("Motor1 setting speed: ");
      Serial.println(speed);
      bldcMotor1.setSpeed(speed);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void Motor2Task(void * parameter) {
  int speed = BLDC_Center;
  xQueueSend(motor2Queue, &speed, 0);
  
  while(true) {
    if (xQueueReceive(motor2Queue, &speed, 0) == pdTRUE) {
      Serial.print("Motor2 setting speed: ");
      Serial.println(speed);
      bldcMotor2.setSpeed(speed);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void Motor3Task(void * parameter) {
  int speed = BLDC_Center;
  xQueueSend(motor3Queue, &speed, 0);
  
  while(true) {
    if (xQueueReceive(motor3Queue, &speed, 0) == pdTRUE) {
      Serial.print("Motor3 setting speed: ");
      Serial.println(speed);
      bldcMotor3.setSpeed(speed);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void Motor4Task(void * parameter) {
  int speed = BLDC_Center;
  xQueueSend(motor4Queue, &speed, 0);
  
  while(true) {
    if (xQueueReceive(motor4Queue, &speed, 0) == pdTRUE) {
      Serial.print("Motor4 setting speed: ");
      Serial.println(speed);
      bldcMotor4.setSpeed(speed);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// Modified SerialTask with new MOTORS command
void SerialTask(void * parameter) {
  while(true) {
    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      Serial.print("Received command: ");
      Serial.println(command);
      
      command.trim();
      
      // New command for all motors
      if (command.startsWith("MOTORS")) {
        int speed = command.substring(7).toInt();
        setAllMotorsSpeed(speed);
      }
      // Individual motor commands
      else if (command.startsWith("MOTOR1")) {
        int speed = command.substring(6).toInt();
        if (speed >= CW_MAX_BLDC_Speed && speed <= CCW_MAX_BLDC_Speed) {
          xQueueSend(motor1Queue, &speed, portMAX_DELAY);
        }
      }
      else if (command.startsWith("MOTOR2")) {
        int speed = command.substring(6).toInt();
        if (speed >= CW_MAX_BLDC_Speed && speed <= CCW_MAX_BLDC_Speed) {
          xQueueSend(motor2Queue, &speed, portMAX_DELAY);
        }
      }
      else if (command.startsWith("MOTOR3")) {
        int speed = command.substring(6).toInt();
        if (speed >= CW_MAX_BLDC_Speed && speed <= CCW_MAX_BLDC_Speed) {
          xQueueSend(motor3Queue, &speed, portMAX_DELAY);
        }
      }
      else if (command.startsWith("MOTOR4")) {
        int speed = command.substring(6).toInt();
        if (speed >= CW_MAX_BLDC_Speed && speed <= CCW_MAX_BLDC_Speed) {
          xQueueSend(motor4Queue, &speed, portMAX_DELAY);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void initializeMotors() {
  // Initialize all motors with a proper sequence
  Serial.println("Initializing motors...");
  
  // First attach all motors
  bldcMotor1.Attach(MotorPin1);
  delay(500);  // Delay between each motor initialization
  bldcMotor2.Attach(MotorPin2);
  delay(500);
  bldcMotor3.Attach(MotorPin3);
  delay(500);
  bldcMotor4.Attach(MotorPin4);
  delay(500);

  Serial.println("All motors initialized!");
}

void setup() {
  Serial.begin(SerialMonitor);
  
  // Create queues for motor commands
  motor1Queue = xQueueCreate(5, sizeof(int));
  motor2Queue = xQueueCreate(5, sizeof(int));
  motor3Queue = xQueueCreate(5, sizeof(int));
  motor4Queue = xQueueCreate(5, sizeof(int));
  
  // ESP32 timer allocation
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  // Initialize motors with proper sequence
  initializeMotors();

  delay(1000);
  Serial.println("\nQuad BLDC Motor Control Ready");
  Serial.println("Commands:");
  Serial.println("'MOTORn speed' - control individual motor (n: 1-4)");
  Serial.println("'MOTORS speed' - control all motors simultaneously");
  Serial.println("Speed range: 1060-1860");
  Serial.println("Examples:");
  Serial.println("MOTOR1 1600");
  Serial.println("MOTORS 1500");

  // Create tasks (Same as before...)
  xTaskCreatePinnedToCore(
    Motor1Task, "Motor1Task", 10000, NULL, 1, &motor1TaskHandle, 0
  );
  xTaskCreatePinnedToCore(
    Motor2Task, "Motor2Task", 10000, NULL, 1, &motor2TaskHandle, 0
  );
  xTaskCreatePinnedToCore(
    Motor3Task, "Motor3Task", 10000, NULL, 1, &motor3TaskHandle, 0
  );
  xTaskCreatePinnedToCore(
    Motor4Task, "Motor4Task", 10000, NULL, 1, &motor4TaskHandle, 0
  );
  xTaskCreatePinnedToCore(
    SerialTask, "SerialTask", 10000, NULL, 2, &serialTaskHandle, 1
  );
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}