# bldc-control



## 소스코드 목록
### 최종 코드 : thread_and_button.ino
#### ESP32
1. **by_button**
    - 버튼 2개로 모터 하나의 속도 제어

2. **by_value**
    - 속도 입력하여 모터 속도 조절

3. **sep_thread**
    - 스레드 사용하여, 2개 모터 각각 속도 입력하여 조절

4. **separately_noThread**
    - 스레드 사용하지 않고, 2개 모터 각각 속도 입력하여 조절

5. **simultaneously**
    - 모터 두개 속도 입력하여 동시, 일시 제어

6. **thread_and_button**
    - 스레드 사용하여, 2개 모터를 각각 연동된 버튼으로 제어
    - 버튼 1 토글시 모터 1 시동, 다시 토글시 모터1 정지
    - 버튼 2 토글시 모터 2 시동, 다시 토글시 모터2 정지

#### STM32

--- 

## 프로젝트 진행 요망 사항
1. 4개 BLDC 모터 일시 제어 -> 3개 성공 했으므로 추가 구매 후 무리 없는 진행 예상
2. ST 보드로 진행중(2024.10.31)
    - ioc 파일 configuration -> micro-ROS 관련 library 옮겨 세팅 진행중
    - PWM 조정 등을 위한 타이머 세팅이 복잡
    - micro-ROS 관련 디렉토리 및 라이브러리 요소 파악중

## 프로젝트 완료
1. 1개 모터 제어 완료
2. 2개 모터 일시, 개별 제어 완료
3. 3개 모터 일시, 개별 제어 완료, ST 보드로 진행


## about micro-ROS

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

### micro-ROS - Ubuntu 22.04lts setup

1. Install micro-ROS tools in Ubuntu:
```bash
# Create workspace
mkdir -p ~/microros_ws/src
cd ~/microros_ws

# Clone micro-ROS setup package
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Install dependencies
rosdep update && rosdep install --from-paths src --ignore-src -y

# Build
colcon build # 이거 하면 /microros_ws 아래 log, install, build 디렉토리 생성됨
source install/local_setup.bash # 현재 사용중인 쉘에 패키지 로드

# 참고
source /opt/ros/humble/setup.bash # 현재 사용중인 쉘에 ROS2 패키지 로드
```

2. Create firmware workspace:
```bash
# Create firmware for FreeRTOS + STM32
ros2 run micro_ros_setup create_firmware_ws.sh freertos [$TARGETTING_SYSTEM]
# ros2 run: Execute a ROS2 package
# micro_ros_setup: Package name
# create_firmware_ws.sh: Script to create firmware workspace
# freertos: Specify FreeRTOS as RTOS
# nucleo_f767zi: Specify your exact board (instead of generic stm32cubemx)
# firmware 디렉토리 생김

# Configure for Nucleo-F767ZI
ros2 run micro_ros_setup configure_firmware.sh [APP] [OPTIONS]
# 어떻게 타겟 시스템이랑 연결 할지 세팅, serial 로 하면 되는데 serial-usb 는 된다면서 안 해줌 갯새긱가
```

3. Build, flash the firmware
```bash
ros2 run micro_ros_setup build_firmware.sh

ros2 run micro_ros_setup flash_firmware.sh
# serial 로 해도, 보드에 바로 붙은 디버거로 플래시 하니 되기는 함
```





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

