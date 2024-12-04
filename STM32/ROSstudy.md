# Materials for ROS2 and micro-ROS






## about micro-ROS
### micro-ROS란?
- ROS2와 임베디드 시스템 사이 통신 구현 "라이브러리"
- XRCE-DDS 프로토콜 사용

### ROS2 - micro-ROS 연결 구성도

``` mermaid
graph TD
    subgraph Ubuntu_ROS2_System
        ROS2_Node[ROS2 Node]
        Agent[micro-ROS Agent]
    end
    
    subgraph STM32_System
        micro_ROS_Client[micro-ROS Client]
        Sensor[Sensor]
        Actuator[Actuator]
    end

    Sensor -- Sensor Data --> micro_ROS_Client
    micro_ROS_Client -- Publish Data --> Agent
    Agent -- Send to ROS2 Node --> ROS2_Node
    
    ROS2_Node -- Publish Command --> Agent
    Agent -- Send Command --> micro_ROS_Client
    micro_ROS_Client -- Actuate --> Actuator
```
### Feature List
#### [Nodes](https://micro.ros.org/docs/tutorials/programming_rcl_rclc/node/)
#### [Pub/Sub](https://micro.ros.org/docs/tutorials/programming_rcl_rclc/pub_sub/)
#### [Services/Clients](https://micro.ros.org/docs/tutorials/programming_rcl_rclc/service/)
#### Graph
#### Executer
#### Lifecycle
#### Parameter

---

### micro-ROS - Ubuntu 22.04 lts setup
#### 개발환경
- Ubuntu 22.04 lts
- ROS2 Humble

#### 빌드 시스템 및 개발 환경
1. micro_ros_setup
- [micro-ros build system](https://micro.ros.org/docs/concepts/build_system/)


#### standalone build system 설치 절차
> 1. ros2 run micro_ros_setup create_firmware_ws.sh [RTOS] [HARDWARE_BOARD]
> 2. ros2 run micro_ros_setup configure_firmware.sh [APP] [OPTIONS]
> 3. ros2 run micro_ros_setup build_firmware.sh
> 4. ros2 run micro_ros_setup flash_firmware.sh

1. Install micro-ROS tools in Ubuntu:
```bash
# 작업공간 생성
mkdir -p ~/microros_ws/src
cd ~/microros_ws

# 관련 패키지 다운로드
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Install dependencies
rosdep update && rosdep install --from-paths src --ignore-src -y

# Build
colcon build # 이거 하면 /microros_ws 아래 log, install, build 디렉토리 생성됨

# 현재 사용중인 쉘에 패키지 로드
source install/local_setup.bash 

# 참고
source /opt/ros/humble/setup.bash # 현재 사용중인 쉘에 ROS2 패키지 로드
```

2. Create firmware workspace:
```bash
# 개발보드 펌웨어 생성, firmware 디렉토리 생성됨
ros2 run micro_ros_setup create_firmware_ws.sh freertos [$TARGETTING_SYSTEM]
# 기본 ws에 firmware 디렉토리 생김
# create_firmware_ws.sh 는 /ws/src/micro_ros_setup/scripts 에 존재


# Configure for Nucleo-F767ZI
ros2 run micro_ros_setup configure_firmware.sh [APP] [OPTIONS]
# 어떻게 타겟 시스템이랑 연결 할지 세팅, serial 로 하면 되는데 serial-usb 는 지가 된다면서 안 해줌 갯새긱가
# serial 로 하면 됨
# app 부분은 ping_ping 이나 int32_publisher
# options 는 --transport serial
```

3. Build, flash the firmware
```bash
# 빌드
ros2 run micro_ros_setup build_firmware.sh

# 펌웨어 플래시
ros2 run micro_ros_setup flash_firmware.sh
# serial 로 해도, 보드에 바로 붙은 디버거로 플래시 하니 되기는 함
```

### 플래시 이후 테스트 방법
- led_toggle 을 직접 생성, int32_subscriber 응용하여 만듦

```c
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <stdio.h>

#include "stm32f7xx_hal.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;

void subscription_callback(const void * msgin)
{
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    printf("Received: %ld\n", msg->data);
    
    // Control LEDs based on received value
    switch(msg->data) {
        case 1:
            HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);    // Green LED on
            HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);  // Blue LED off
            HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);  // Red LED off
            break;
        case 2:
            HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);  // Green LED off
            HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);    // Blue LED on
            HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);  // Red LED off
            break;
        case 3:
            HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);  // Green LED off
            HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);  // Blue LED off
            HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);    // Red LED on
            break;
        default:
            // Turn all LEDs off
            HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD2_Pin|LD3_Pin, GPIO_PIN_RESET);
    }
}

void appMain(void * arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "int32_subscriber_rclc", "", &support));

    // create subscriber
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/microROS/int32_subscriber"));

    // create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

    while(1){
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            vTaskDelay(pdMS_TO_TICKS(100));
    }

    // free resources
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_node_fini(&node));
    
    vTaskDelete(NULL);
}
```

#### Flash 이후 확인 작업
1. 시리얼통신
```bash 
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 baudrate=115200 -v6
```
2. 




### 이하 CubeIDE 내 micro-ROS 작업공간 마련 하는 방법 정리중
3. In STM32CubeIDE:
- Create new STM32 project:
  * Project Type: STM32Cube
  * Target: STM32F767ZI
  * Language: C

4. Configure in .ioc file:
- FreeRTOS settings:
  * Enable FREERTOS with CMSIS_V1
  * Create microROSTask:
    - Task Name: microROSTask
    - Priority: osPriorityNormal
    - Stack Size: 3000 words
    - Entry Function: MX_microROSTask
    - Allocation: Dynamic
  * Disable USE_NEWLIB_REENTRANT
  * Disable Use FW pack heap file

- USB settings:
  * USB_OTG_FS: Device Only
  * Enable Activate_SOF and Activate_VBUS
  * USB_DEVICE: CDC (Virtual COM Port)

- ESC Motor Control:
  * PA0 pin: TIM2_CH1
  * Configure TIM2 for 50Hz PWM

5. After generating code:
- Copy these folders from:
```bash
~/microros_ws/firmware/freertos_apps/microros_nucleo_f767zi_extensions/
```
to your CubeIDE project:
- `Middlewares/Third_Party/micro_ROS/` → to project's `Middlewares/Third_Party/`
- `micro_ros_stm32cubemx_utils/` → to project's root directory

6. In CubeIDE project settings:
- Add include paths:
  * `../Middlewares/Third_Party/micro_ROS/include`
  * `../micro_ros_stm32cubemx_utils`

Would you like me to:
1. Proceed with any specific part of this setup?
2. Explain any step in more detail?
3. Help with calculating the timer values for ESC control?


### micro-ROS - CubeIDE 통합
[CubeIDE Setup](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils?tab=readme-ov-file)
- 도커 파일 사용하여 통합해야

## Know

## links
- [micro-ROS on FreeRTOS](https://www.freertos.org/Community/Blogs/2020/micro-ros-on-freertos)
- [micro-ros와 esp32](https://www.robotstory.co.kr/king/?vid=901)
- [esp32로 micro-ROS led 예제](https://www.robotstory.co.kr/king/?vid=902)
- [f767보드와 micro-ROS 개발 환경](https://hexagon-emile.hatenablog.com/entry/2023/11/30/235049)
- [XRCE-DDS](https://huroint.tistory.com/entry/Micro-ROS-Micro-XRCE-DDS-%ED%8A%B9%EC%A7%95)