# ROS2 structure

``` mermaid
graph LR
    A[Your PC/Ubuntu] <-->|USB/UART| B[STM32 Board]
    A -->|micro-ROS Agent| C[ROS 2 Network]
    B -->|micro-ROS Client| C
```

``` mermaid
graph LR
    subgraph STM32[STM32 Board]
        C[micro-ROS Client] -->|Publishes Data| T1((Topic))
    end
    subgraph PC[Your Computer]
        A[micro-ROS Agent] -->|Bridges Communication| N[ROS2 Network]
    end
    STM32 <-->|USB/UART| PC
```

``` mermaid
graph LR
    subgraph ROS2 Network
        N1[Node 1] -->|Publishes| T1((Topic A))
        T1 -->|Subscribes| N2[Node 2]
        N2 -->|Publishes| T2((Topic B))
        T2 -->|Subscribes| N1
    end
```