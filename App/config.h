#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 启用状态机中的云台动作。 */
#define CONFIG_USE_GIMBAL              0

/* 是否执行物料取放：1=执行车体/地图取放与码垛，0=跳过并继续状态机。 */
#define CONFIG_ENABLE_MATERIAL_PLACEMENT  1

/* 底盘控制：1=正常执行移动，0=状态机直接模拟到达并跳过移动。 */
#define CONFIG_USE_CHASSIS                1

/*
 * 纯视觉微调测试模式：
 * 1=不启动正常路线，直接根据合法 0x02 视觉帧微调底盘；
 * 0=仅在状态机的 STATE_ADJUST_* 阶段接受偏差移动。
 */
#define CONFIG_VISION_ADJUST_ONLY         0

/* 跳过取料时的颜色确认，并按颜色匹配成功继续动作。 */
#define CONFIG_SKIP_COLOR_CONFIRM      1

/* 跳过二维码物料顺序：1=使用默认顺序 {2, 3, 1} 直接继续，0=等待上位机下发。 */
#define CONFIG_SKIP_QR                 1

/* 跳过视觉微调：1=到位后直接继续，0=等待 X/Y 偏差均稳定进入阈值。 */
#define CONFIG_SKIP_ADJUST             0

/*
 * UART7 TX/RX hardware loopback test.
 * Short PE8 (TX) to PE7 (RX); a non-echoing protocol test frame is sent
 * periodically so the valid 0x01 echo command cannot create a feedback loop.
 */
#define CONFIG_COMM_UART_LOOPBACK_TEST            0
#define CONFIG_COMM_UART_LOOPBACK_TEST_PERIOD_MS  1000U

/* DM 电机低速匀速旋转测试；正常运行时保持为 0。 */
#define CONFIG_DM_SLOW_ROTATE_TEST     0
#define CONFIG_DM_SLOW_ROTATE_SPEED    0.3f

/* DM 电机上电置零：1=上电重置零点，0=保留电机已有零点。 */
#define CONFIG_DM_RESET_ZERO_ON_BOOT   0

/* 1-6号步进通信：1=UART1(Emm_V5)，0=原FDCAN1驱动。 */
#ifndef CONFIG_STEPPER_USE_UART1
#define CONFIG_STEPPER_USE_UART1           1
#endif
#define CONFIG_STEPPER_UART_FRAME_GAP_MS    2U

/* 步进电机分组固定速度；5号伸缩低速运行，6号升降保持高速。 */
#define CONFIG_STEPPER_CHASSIS_SPEED_RPM   200U  /* 1-4号 */
#define CONFIG_STEPPER_EXTEND_SPEED_RPM    100U  /* 5号伸缩 */
#define CONFIG_STEPPER_LIFT_SPEED_RPM      2000U  /* 6号 */

/* 步进电机分组加速度，Emm_V5 有效范围为 0-255。 */
#define CONFIG_STEPPER_CHASSIS_ACCEL       128U  /* 1-4号 */
#define CONFIG_STEPPER_EXTEND_ACCEL        0U   /* 5号伸缩，低加速度 */
#define CONFIG_STEPPER_LIFT_ACCEL          0U  /* 6号 */

/* Timed chassis velocity control. Calibrate X/Y speeds on the real chassis. */
#define CONFIG_CHASSIS_X_SPEED_MM_S         500.0f
#define CONFIG_CHASSIS_Y_SPEED_MM_S         500.0f
#define CONFIG_CHASSIS_X_DIRECTION          (-1.0f)
#define CONFIG_CHASSIS_Y_DIRECTION          (-1.0f)
#define CONFIG_CHASSIS_TRANSLATE_RPM        120.0f
#define CONFIG_CHASSIS_YAW_RPM_LIMIT        40.0f
#define CONFIG_CHASSIS_YAW_OUTPUT_DIRECTION (-1.0f)
#define CONFIG_CHASSIS_MOTOR_RPM_LIMIT      200
#define CONFIG_CHASSIS_MOTOR_1_POLARITY     (1.0f)
#define CONFIG_CHASSIS_MOTOR_2_POLARITY     (1.0f)
#define CONFIG_CHASSIS_MOTOR_3_POLARITY     (1.0f)
#define CONFIG_CHASSIS_MOTOR_4_POLARITY     (1.0f)
#define CONFIG_CHASSIS_CONTROL_PERIOD_MS    20U
#define CONFIG_CHASSIS_MIN_MOVE_MS          100U
#define CONFIG_CHASSIS_STOP_SETTLE_MS       50U
#define CONFIG_CHASSIS_GYRO_TIMEOUT_MS      1000U

#define CONFIG_CHASSIS_HEADING_KP           1.5f
#define CONFIG_CHASSIS_HEADING_KI           0.0f
#define CONFIG_CHASSIS_HEADING_KD           0.0f
#define CONFIG_CHASSIS_HEADING_DEAD_DEG     0.5f

/* 视觉微调：X/Y 使用 PID 框架的纯 P 闭环，输入为归一化偏差。 */
#define CONFIG_VISION_ADJUST_KP_X            1.0f
#define CONFIG_VISION_ADJUST_KP_Y            -0.5f
#define CONFIG_VISION_ADJUST_DEADZONE        5.0f
#define CONFIG_VISION_ADJUST_RPM_LIMIT       15.0f
#define CONFIG_VISION_ADJUST_YAW_RPM_LIMIT   10.0f
#define CONFIG_VISION_ADJUST_PERIOD_MS       20U
#define CONFIG_VISION_ADJUST_TIMEOUT_MS      200U
#define CONFIG_VISION_ADJUST_X_STABLE_FRAMES 3U
#define CONFIG_VISION_ADJUST_Y_STABLE_FRAMES 3U

/* 6号升降电机的四档取放高度（相对顶部安全位的下降脉冲）。 */
#define CONFIG_GIMBAL_LIFT_GROUND_PULSES     90000L
#define CONFIG_GIMBAL_LIFT_CAR_PULSES        90000L
#define CONFIG_GIMBAL_LIFT_TURNTABLE_PULSES  90000L
#define CONFIG_GIMBAL_LIFT_STACK_PULSES      90000L
#define CONFIG_GIMBAL_LIFT_GROUND_WAIT_MS     2000U
#define CONFIG_GIMBAL_LIFT_CAR_WAIT_MS        1500U
#define CONFIG_GIMBAL_LIFT_TURNTABLE_WAIT_MS  2000U
#define CONFIG_GIMBAL_LIFT_STACK_WAIT_MS      2000U
#define CONFIG_GIMBAL_ROTATE_WAIT_MS           4000U
#define CONFIG_GIMBAL_GRIPPER_WAIT_MS          200U

/*
 * 伸展电机到地图工位的相对脉冲。
 * 地面取放和码垛共用，每个位置可独立调整。
 */
#define CONFIG_GIMBAL_MAP_EXTEND_POS_1_PULSES  400L
#define CONFIG_GIMBAL_MAP_EXTEND_POS_2_PULSES  200L
#define CONFIG_GIMBAL_MAP_EXTEND_POS_3_PULSES  400L

/*
 * 夹爪舵机 PWM：TIM1 已配置为 1 MHz 计数、20 ms 周期，因此 CCR 数值
 * 与高电平脉宽（us）相同。当前舵机有效行程为 0.5-2.5 ms。
 */
#define CONFIG_GRIPPER_OPEN_PULSE_US        1400
#define CONFIG_GRIPPER_CLOSE_PULSE_US       1100

/*
 * 按键任务启动前的夹爪舵机测试：
 * 1=上电后先张开、再闭合；0=只初始化 PWM 并保持闭合。
 */
#define CONFIG_GRIPPER_STARTUP_TEST          0
#define CONFIG_GRIPPER_STARTUP_TEST_HOLD_MS  2000U

/* 车体物料放置位置对应的云台角度（度）。 */
#define CONFIG_CAR_MATERIAL_POS_1_DEG  -8.0f
#define CONFIG_CAR_MATERIAL_POS_2_DEG  CONFIG_CAR_MATERIAL_POS_1_DEG-30.0f
#define CONFIG_CAR_MATERIAL_POS_3_DEG  CONFIG_CAR_MATERIAL_POS_2_DEG-35.0f

/* 地图物料放置位置对应的云台角度（度）。 */
#define CONFIG_MAP_MATERIAL_POS_1_DEG  150
#define CONFIG_MAP_MATERIAL_POS_2_DEG  135
#define CONFIG_MAP_MATERIAL_POS_3_DEG  90

/* 云台线缆保护：以当前电机零点为中心，只允许在单圈内运动。 */
#define CONFIG_GIMBAL_SINGLE_TURN_ENABLE  1
#define CONFIG_GIMBAL_TURN_MIN_DEG        (-180.0f)
#define CONFIG_GIMBAL_TURN_MAX_DEG        180.0f

/* 兼容现有的条件编译名称。 */
#if CONFIG_USE_GIMBAL
#define USE_GIMBAL
#endif

#endif /* APP_CONFIG_H */
