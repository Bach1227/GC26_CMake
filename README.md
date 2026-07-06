# GC26 底盘控制项目

STM32H723 平台，FreeRTOS 实时系统，控制 ZDT 步进电机底盘 + DM 编码器电机。

## 目录结构

```
App/                  # 应用层代码
├── ChassisControl.c/h   底盘控制 (队列驱动移动 + DM 电机 PID)
├── statemachine.c/h     查表法状态机 (事件队列驱动)
├── protocol.c/h         上位机通讯协议解析
├── PID.c/h              浮点/定点 PID 控制器
├── comm_manager.c/h     串口 DMA 收发管理
├── interrupthandle.c    中断处理 (UART + CAN)
├── MoveControl.c        移动控制
└── DataStructure.h      通用数据结构和工具

Bsp/                  # 板级支持包
├── bsp_can.c/h          FDCAN 底层收发
├── bsp_zdt.c/h          ZDT 步进电机驱动 (CAN)
├── bsp_dm.c/h           DM 编码器电机驱动 (CAN MIT 协议)

Core/                 # HAL + RTOS 初始化
├── Src/main.c           主函数
├── Src/freertos.c       FreeRTOS 初始化 + 任务创建
└── Inc/fdcan.h          FDCAN 句柄声明
```

## 核心架构

```
上位机 (串口)
    │  AA 55 ... 协议帧
    ↓
comm_manager (DMA 双缓冲)
    → protocol.c Protocol_ParseBuffer → Protocol_Dispatch
        │
        ├── CMD_CAR_MOVE  → Chassis_OnCarMove()   (旧式直接移动)
        ├── CMD_MOVE_ADJUST → SM_SetAdjustOffset() (视觉偏差)
        └── CMD_GRASP/EMERGENCY → ...其他命令

状态机 (statemachine)
    │  事件队列 (xQueue)
    │  SM_SendEvent → SM_Task → SM_ProcessEvent
    │                              → Action_xxx → Chassis_SendMoveCmd()
    ↓
Chassis_Task (队列驱动)
    │  xQueueReceive 阻塞等命令
    │  → 逆运动学 Chassis_IK → 四轮 wheel_position → ZDT_SyncTrigger
    │  → 软件定时器轮询到位标志
    ↓
CAN 总线
    ├── ZDT 步进电机 (位置模式, 回传 FD+9F 到位)
    └── DM 编码器电机 (MIT 协议, 速度模式)
```

### 状态机流转

```
IDLE ─START→ MOVE_TO_QR ─ARRIVED_QR→ READ_QR
  ↑                                        │
  │                                   QR_PARSED
  │                                        ↓
RETURN ←ALL_DONE─ TRANSFER_TEMP ←ARRIVED_TEMP─ MOVE_TO_TEMP
  ↑                    │                          ↑
  │              BATCH1_NEED_NEXT                 │
  │                    ↓                          │
  │              MOVE_TO_RAW ─ARRIVED_RAW→ CHECK_RAW
  │                    ↑                    │
  │                    │              ADJUSTED / MISALIGN
  │                    │                    │
  │                    │               FETCH_RAW ← 修正循环
  │                    │                    │
  │                    │              FETCH_DONE
  │                    │                    ↓
  │                    │              MOVE_TO_ROUGH ─ARRIVED_ROUGH→ CHECK_ROUGH
  │                    │                                         │
  │                    │                                    ADJUSTED / MISALIGN
  │                    │                                         │
  │                    │                                    PLACE_ROUGH ← 修正循环
  │                    │                                         │
  │                    │                                   PLACE_ROUGH_DONE
  │                    └───────────────────────────────────────┘ (循环第二批)
  │
  └───────────────────────────────────────────────────────────────┘ (返航)
```

## TODO List

### 状态机动作函数
- [ ] `Action_ParseQR` — 实现二维码读取与解析, 完成后 `SM_SendEvent(EVENT_QR_PARSED)`
- [ ] `Action_FetchRaw` — 实现原料区抓取, 完成后 `SM_SendEvent(EVENT_FETCH_DONE)`
- [ ] `Action_PlaceRough` — 实现粗加工区放置, 完成后 `SM_SendEvent(EVENT_PLACE_ROUGH_DONE)`
- [ ] `Action_TransferTemp` — 实现暂存区放置/码垛, 根据批次决定发 `BATCH1_DONE_NEED_NEXT` 或 `ALL_BATCHES_DONE`

### 移动参数调整
- [ ] 标定各目标点实际坐标 (QR/原料区/粗加工区/暂存区) → 修改 `Chassis_SendMoveCmd` 参数
- [ ] 标定 `WHEEL_BASE`、`PULSES_PER_REV`、`WHEEL_RADIUS` 等运动学参数
- [ ] 调整 `POS_SPEED_RPM` / `POS_ACCEL` 使移动速度和加速度符合要求

### 视觉反馈闭环
- [ ] 上位机视觉程序开发: 检测底盘到位偏差 → 发 `CMD_MOVE_ADJUST(x, y)`
- [ ] 本地传感器/视觉对接: 替代 `SM_SetAdjustOffset` 的桩代码
- [ ] 校准 `ALIGN_THRESHOLD_MM` (当前 5mm)

### DM 编码器电机
- [ ] 确认 DM 电机 CAN ID 和 MIT 协议参数匹配实际硬件
- [ ] 调整 `DM_PID_Kp/Ki/Kd` 使角度闭环响应满足要求
- [ ] 陀螺仪闭环替代开环旋转 (当前 `Chassis_IK` 旋转分量已置 0)

### CAN 总线
- [ ] `CAN_FilterInit()` 当前未调用, 如需接收需在初始化中加入
- [ ] ZDT 到位检测依赖电机回传 `FD+9F`, 验证此协议是否与实际硬件匹配
- [ ] 确认 FDCAN 波特率与上位机 CAN 工具一致

### 协议
- [ ] `CMD_MOVE_ADJUST` 由上机位在每次到位后发送, 带实时偏差值
- [ ] 确认 `CMD_GRASP` / `CMD_EMERGENCY` 命令处理与硬件匹配

### 调试与测试
- [ ] 开环单体测试: Chassis_SendMoveCmd 正常驱动四轮
- [ ] 到位检测测试: ZDT 电机回传 FD+9F 被正确识别
- [ ] 状态机流程测试: 完整跑一遍 IDLE → ... → IDLE
- [ ] 异常处理: ZDT_WaitMoveDone 超时 (5s) 的容错逻辑

## API 速查

```c
// 底盘移动
Chassis_SendMoveCmd(x_mm, y_mm, EVENT_ARRIVED_xxx);  // 下发移动 (非阻塞)
Chassis_OnCarMove(&car_move);                          // 旧式直接移动

// 状态机
SM_Init();                                              // 初始化 (默认表 + STATE_IDLE)
SM_SendEvent(EVENT_xxx);                                // 从任务发事件
SM_SendEventFromISR(EVENT_xxx);                         // 从中断发事件
SM_SetAdjustOffset(x_mm, y_mm);                         // 写入视觉偏差
SM_GetState();                                          // 获取当前状态

// DM 电机
Chassis_Init_DM(&hfdcan1);                              // 初始化
Chassis_Set_DM_Target(angle_rad);                       // 设置目标角度

// 到位检测
ZDT_WaitMoveDone(addr, timeout_ms);                     // 等待电机到位
ZDT_OnRxMessage(ext_id, data, dlc);                     // CAN 回调 (检测 FD+9F)
```
