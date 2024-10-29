#include <ESP32Servo.h>

// Pin Definitions
#define MotorPin1 13    // First BLDC motor pin
#define MotorPin2 12    // Second BLDC motor pin
#define BUTTON1_PIN 5   // Button 1 pin
#define BUTTON2_PIN 18  // Button 2 pin
#define MOTOR_SPEED 1580  // Fixed speed for both motors
#define BLDC_Center 1500  // Stop position

// Task handles
TaskHandle_t motor1TaskHandle = NULL;
TaskHandle_t motor2TaskHandle = NULL;
TaskHandle_t buttonTaskHandle = NULL;

// Queue handles for motor commands
QueueHandle_t motor1Queue;
QueueHandle_t motor2Queue;

// Button states
bool motor1Running = false;
bool motor2Running = false;
bool lastButton1State = true;  // Using pullup, so default is HIGH
bool lastButton2State = true;

class BLDC_Motor {
  Servo servo;

public:
  BLDC_Motor() {}
  
  void Attach(int pin) {
    servo.attach(pin, 1000, 2000);
    servo.write(BLDC_Center);  // Initialize at stop position
  }

  void setSpeed(int speed) {
    servo.write(speed);
  }

  void stop() {
    servo.write(BLDC_Center);
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
      if (speed == 0) {
        bldcMotor1.stop();
      } else {
        bldcMotor1.setSpeed(speed);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// Task function for Motor 2
void Motor2Task(void * parameter) {
  int speed = 0;
  while(true) {
    if (xQueueReceive(motor2Queue, &speed, 0) == pdTRUE) {
      if (speed == 0) {
        bldcMotor2.stop();
      } else {
        bldcMotor2.setSpeed(speed);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// Task function for Button handling
void ButtonTask(void * parameter) {
  // Configure button pins with pullup
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  
  while(true) {
    // Read current button states
    bool button1State = digitalRead(BUTTON1_PIN);
    bool button2State = digitalRead(BUTTON2_PIN);
    
    // Button 1 handling (Motor 1)
    if (button1State != lastButton1State) {
      if (button1State == LOW) {  // Button pressed (LOW due to pullup)
        motor1Running = !motor1Running;  // Toggle state
        int speed = motor1Running ? MOTOR_SPEED : 0;
        xQueueSend(motor1Queue, &speed, portMAX_DELAY);
        
        Serial.print("Motor 1 ");
        Serial.println(motor1Running ? "Started" : "Stopped");
      }
      lastButton1State = button1State;
      vTaskDelay(pdMS_TO_TICKS(50));  // Debounce delay
    }
    
    // Button 2 handling (Motor 2)
    if (button2State != lastButton2State) {
      if (button2State == LOW) {  // Button pressed (LOW due to pullup)
        motor2Running = !motor2Running;  // Toggle state
        int speed = motor2Running ? MOTOR_SPEED : 0;
        xQueueSend(motor2Queue, &speed, portMAX_DELAY);
        
        Serial.print("Motor 2 ");
        Serial.println(motor2Running ? "Started" : "Stopped");
      }
      lastButton2State = button2State;
      vTaskDelay(pdMS_TO_TICKS(50));  // Debounce delay
    }
    
    vTaskDelay(pdMS_TO_TICKS(20));  // Task delay
  }
}

void setup() {
  Serial.begin(115200);
  
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
  Serial.println("Press Button 1 to toggle Motor 1");
  Serial.println("Press Button 2 to toggle Motor 2");

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
    ButtonTask,
    "ButtonTask",
    10000,
    NULL,
    2,  // Higher priority for button handling
    &buttonTaskHandle,
    1  // Core 1
  );
}

void loop() {
  // Empty loop as all work is done in tasks
  vTaskDelay(portMAX_DELAY);
}