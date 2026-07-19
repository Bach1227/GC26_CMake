#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 启用状态机中的云台动作。 */
#define CONFIG_USE_GIMBAL              0

/* 跳过取料时的颜色确认，并按颜色匹配成功继续动作。 */
#define CONFIG_SKIP_COLOR_CONFIRM      1

/* DM 电机低速匀速旋转测试；正常运行时保持为 0。 */
#define CONFIG_DM_SLOW_ROTATE_TEST     0
#define CONFIG_DM_SLOW_ROTATE_SPEED    0.3f

/* DM 电机上电置零：1=上电重置零点，0=保留电机已有零点。 */
#define CONFIG_DM_RESET_ZERO_ON_BOOT   0

/* 底盘控制：1=正常执行移动，0=状态机直接模拟到达并跳过移动。 */
#define CONFIG_USE_CHASSIS                1

/* 1-6号步进通信：1=UART1(Emm_V5)，0=原FDCAN1驱动。 */
#ifndef CONFIG_STEPPER_USE_UART1
#define CONFIG_STEPPER_USE_UART1           1
#endif
#define CONFIG_STEPPER_UART_FRAME_GAP_MS    2U

/* 步进电机分组固定速度；6号升降电机速度最高。 */
#define CONFIG_STEPPER_CHASSIS_SPEED_RPM   200U  /* 1-4号 */
#define CONFIG_STEPPER_EXTEND_SPEED_RPM    300U  /* 5号 */
#define CONFIG_STEPPER_LIFT_SPEED_RPM      1500U  /* 6号 */

/* 步进电机分组加速度，Emm_V5 有效范围为 0-255。 */
#define CONFIG_STEPPER_CHASSIS_ACCEL       128U  /* 1-4号 */
#define CONFIG_STEPPER_EXTEND_ACCEL        160U  /* 5号 */
#define CONFIG_STEPPER_LIFT_ACCEL          0U  /* 6号 */

/* Timed chassis velocity control. Calibrate X/Y speeds on the real chassis. */
#define CONFIG_CHASSIS_X_SPEED_MM_S         500.0f
#define CONFIG_CHASSIS_Y_SPEED_MM_S         500.0f
#define CONFIG_CHASSIS_X_DIRECTION          (-1.0f)
#define CONFIG_CHASSIS_Y_DIRECTION          (-1.0f)
#define CONFIG_CHASSIS_TRANSLATE_RPM        160.0f
#define CONFIG_CHASSIS_YAW_RPM_LIMIT        40.0f
#define CONFIG_CHASSIS_MOTOR_RPM_LIMIT      200
#define CONFIG_CHASSIS_CONTROL_PERIOD_MS    20U
#define CONFIG_CHASSIS_MIN_MOVE_MS          100U
#define CONFIG_CHASSIS_STOP_SETTLE_MS       50U
#define CONFIG_CHASSIS_GYRO_TIMEOUT_MS      1000U

#define CONFIG_CHASSIS_HEADING_KP           1.5f
#define CONFIG_CHASSIS_HEADING_KI           0.0f
#define CONFIG_CHASSIS_HEADING_KD           0.0f
#define CONFIG_CHASSIS_HEADING_DEAD_DEG     0.5f

/* 6号升降电机行程（脉冲）：车体高度和底面/地图工位高度分别配置。 */
#define CONFIG_GIMBAL_LIFT_CAR_PULSES      20000L
#define CONFIG_GIMBAL_LIFT_GROUND_PULSES   20000L

/* 车体物料放置位置对应的云台角度（度）。 */
#define CONFIG_CAR_MATERIAL_POS_1_DEG  0.0f
#define CONFIG_CAR_MATERIAL_POS_2_DEG  30.0f
#define CONFIG_CAR_MATERIAL_POS_3_DEG  60.0f

/* 地图物料放置位置对应的云台角度（度）。 */
#define CONFIG_MAP_MATERIAL_POS_1_DEG  150
#define CONFIG_MAP_MATERIAL_POS_2_DEG  -180
#define CONFIG_MAP_MATERIAL_POS_3_DEG  (-150.0f)

/* 云台线缆保护：以当前电机零点为中心，只允许在单圈内运动。 */
#define CONFIG_GIMBAL_SINGLE_TURN_ENABLE  1
#define CONFIG_GIMBAL_TURN_MIN_DEG        (-180.0f)
#define CONFIG_GIMBAL_TURN_MAX_DEG        180.0f

/* 兼容现有的条件编译名称。 */
#if CONFIG_USE_GIMBAL
#define USE_GIMBAL
#endif

#endif /* APP_CONFIG_H */
