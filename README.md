# 上下位机任务通信协议

`task/` 实现上位机侧的组帧、解析和串口流程；`embedded/protocol.c/.h` 是 STM32
工程的接入样例。嵌入式样例依赖下位机工程提供的 `ChassisControl.h`、`config.h`
和 `statemachine.h`，因此不会由本仓库的顶层 CMake 单独构建。

## 串口参数

- 接口：UART7
- 波特率：115200
- 数据格式：8N1
- 数据方向：双向；上位机发送任务帧，STM32 接收
- 物料顺序使用“原帧回显”确认；视觉反馈不使用 ACK

## 帧格式

```text
AA 55 CMD LEN PAYLOAD... CHECKSUM
```

| 字段 | 长度 | 说明 |
|---|---:|---|
| `AA 55` | 2字节 | 固定帧头 |
| `CMD` | 1字节 | 命令 |
| `LEN` | 1字节 | PAYLOAD 字节数，最大 6 |
| `PAYLOAD` | 0～6字节 | 命令数据 |
| `CHECKSUM` | 1字节 | `(CMD + LEN + PAYLOAD所有字节) & 0xFF` |

颜色编号统一为：

```text
0 = 未识别
1 = 红色
2 = 绿色
3 = 蓝色
```

## 命令

### 0x01 设置两轮物料顺序

```text
LEN = 6
PAYLOAD = ROUND_1_COLOR_1 ROUND_1_COLOR_2 ROUND_1_COLOR_3
          ROUND_2_COLOR_1 ROUND_2_COLOR_2 ROUND_2_COLOR_3
```

二维码文本格式为 `123+321`。加号两侧必须分别是 `1、2、3` 的不重复排列。
状态机到达二维码位置后保存两轮物料顺序。

示例：第一轮红绿蓝、第二轮蓝绿红（`123+321`）：

```text
AA 55 01 06 01 02 03 03 02 01 13
```

STM32 收到帧头、长度、校验和及两组排列都合法的 `0x01` 后，必须通过 UART
逐字节发回完全相同的 11 字节帧。上位机每 500 ms 重发一次，只有收到完全一致的
回显才启动物料识别。重复收到合法帧时仍应回显，以处理上一次回显在链路中丢失的情况。

### 0x02 视觉反馈

```text
LEN = 4
PAYLOAD = COLOR OFFSET_X OFFSET_Y IS_STATIC
```

| 字段 | 类型 | 说明 |
|---|---|---|
| `COLOR` | `uint8` | `0`=未识别，`1`=红，`2`=绿，`3`=蓝 |
| `OFFSET_X` | `int8` | X 归一化偏差，右为正，范围 -127～127 |
| `OFFSET_Y` | `int8` | Y 归一化偏差，下为正，范围 -127～127 |
| `IS_STATIC` | `uint8` | `0`=物体运动中，`1`=物体静止 |

上位机先在当前工作画面中计算像素偏差：

```text
DX_PIXEL = TARGET_X - FRAME_WIDTH / 2
DY_PIXEL = TARGET_Y - FRAME_HEIGHT / 2
```

再分别归一化并四舍五入：

```text
OFFSET_X = clamp(round(DX_PIXEL / (FRAME_WIDTH / 2) * 127), -127, 127)
OFFSET_Y = clamp(round(DY_PIXEL / (FRAME_HEIGHT / 2) * 127), -127, 127)
```

因此线上偏差是无量纲的归一化控制量，不是毫米，也不是原始像素。`COLOR=0`
同时承担“未识别”标志，此时 X、Y 和 `IS_STATIC` 都发送 0。

上位机只从已经跟踪且完成跨帧投票的圆中选择目标，并发送距离画面中心最近的圆。
同一目标连续 5 帧的相邻圆心位移不超过 2 像素时，`IS_STATIC=1`。

STM32 始终保存合法颜色。在 `STATE_ADJUST_*` 状态中：

- `IS_STATIC=0`：按照 `OFFSET_X/OFFSET_Y` 执行一次微调。
- `IS_STATIC=1`：不执行偏差移动，产生 `EVENT_ADJUST_DONE` 并进入下一动作。

示例：识别为绿色，归一化 X=-10、Y=5，物体仍在运动：

```text
AA 55 02 04 02 F6 05 00 03
```

## 异常处理

- 帧头、长度、校验或命令参数非法时直接丢弃。
- 解析器支持半帧、连续多帧和帧前噪声，并自动重新搜索 `AA 55`。
- 物料任务开始后，两轮物料顺序不再允许修改；重复帧仍需回显。
- 非视觉调整状态收到视觉反馈时只保存合法颜色，不执行偏差移动或状态切换。

## 下位机接入契约

- 状态机提供 `SM_SetMaterialSequences(first, second)`，一次保存两组各 3 字节顺序。
- UART 驱动在初始化时通过 `Protocol_SetTransmitCallback()` 注册发送函数。
- 发送回调必须在返回前复制或同步发送传入帧；协议层缓冲区仅在调用期间有效。
- 下位机将 `OFFSET_X/OFFSET_Y` 作为归一化控制误差使用，不再按毫米解释。
