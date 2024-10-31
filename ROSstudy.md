# ROS2 structure


## ROS2 - micro-ROS 연결 구성도

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