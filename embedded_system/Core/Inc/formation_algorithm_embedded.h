/**
 * @file formation_algorithm_embedded.h
 * @brief 嵌入式多车编队控制算法头文件
 * @author Formation Control Team
 * @version 1.0
 * @date 2024
 */

#ifndef FORMATION_ALGORITHM_EMBEDDED_H
#define FORMATION_ALGORITHM_EMBEDDED_H

#ifdef __cplusplus
extern "C" {
#endif

/* 包含标准库 */
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

/* 包含硬件相关头文件 */
#include "stm32f4xx_hal.h"
#include "hardware_interface.h"
#include "communication.h"

/* 系统参数定义 */
#define MAX_VEHICLES                8       // 最大车辆数量
#define MAX_OBSTACLES              16       // 最大障碍物数量
#define CONTROL_FREQUENCY          50       // 控制频率 (Hz)
#define COMMUNICATION_FREQUENCY    10       // 通信频率 (Hz)

/* 数学常数 */
#ifndef M_PI
#define M_PI                       3.14159265358979323846f
#endif
#define M_PI_2                     (M_PI / 2.0f)
#define DEG_TO_RAD                 (M_PI / 180.0f)
#define RAD_TO_DEG                 (180.0f / M_PI)

/* 系统状态定义 */
typedef enum {
    FORMATION_STATE_IDLE = 0,       // 空闲状态
    FORMATION_STATE_INIT,           // 初始化状态
    FORMATION_STATE_FORMING,        // 编队形成中
    FORMATION_STATE_FORMED,         // 编队已形成
    FORMATION_STATE_TRANSITIONING,  // 队形变换中
    FORMATION_STATE_FOLLOWING,      // 跟随状态
    FORMATION_STATE_EMERGENCY       // 紧急状态
} FormationState_t;

/* 编队类型定义 */
typedef enum {
    FORMATION_TYPE_LINE = 0,        // 直线编队
    FORMATION_TYPE_V_SHAPE,         // V形编队
    FORMATION_TYPE_DIAMOND,         // 菱形编队
    FORMATION_TYPE_CIRCLE,          // 圆形编队
    FORMATION_TYPE_RECTANGLE        // 矩形编队
} FormationType_t;

/* 二维向量结构体 */
typedef struct {
    float x;                        // X坐标
    float y;                        // Y坐标
} Vector2D_t;

/* 车辆状态结构体 */
typedef struct {
    uint8_t id;                     // 车辆ID
    Vector2D_t position;            // 位置 (m)
    Vector2D_t velocity;            // 速度 (m/s)
    float heading;                  // 航向角 (rad)
    float angular_velocity;         // 角速度 (rad/s)
    uint32_t timestamp;             // 时间戳 (ms)
    bool is_valid;                  // 状态是否有效
    bool is_leader;                 // 是否为领导者
} VehicleState_t;

/* 障碍物结构体 */
typedef struct {
    Vector2D_t position;            // 位置 (m)
    float radius;                   // 半径 (m)
    bool is_dynamic;                // 是否为动态障碍物
    Vector2D_t velocity;            // 速度 (m/s，仅动态障碍物)
} Obstacle_t;

/* 控制参数结构体 */
typedef struct {
    /* 人工势场参数 */
    float k_att;                    // 吸引力增益
    float k_rep;                    // 排斥力增益
    float k_form;                   // 编队力增益
    float k_obstacle;               // 障碍物排斥力增益
    
    /* 影响范围参数 */
    float influence_radius;         // 势场影响半径 (m)
    float obstacle_radius;          // 障碍物影响半径 (m)
    float safety_distance;          // 安全距离 (m)
    
    /* 运动限制参数 */
    float max_velocity;             // 最大速度 (m/s)
    float max_angular_velocity;     // 最大角速度 (rad/s)
    float max_acceleration;         // 最大加速度 (m/s²)
    
    /* 共识算法参数 */
    float consensus_gain;           // 共识增益
    float damping_factor;           // 阻尼系数
    float convergence_threshold;    // 收敛阈值 (m)
} ControlParams_t;

/* 编队配置结构体 */
typedef struct {
    FormationType_t type;           // 编队类型
    uint8_t num_vehicles;           // 车辆数量
    float scale;                    // 编队尺度
    uint8_t leader_id;              // 领导者ID
    Vector2D_t center;              // 编队中心
    float heading;                  // 编队航向
    Vector2D_t desired_positions[MAX_VEHICLES]; // 期望位置
} FormationConfig_t;

/* 控制输出结构体 */
typedef struct {
    float linear_velocity;          // 线速度 (m/s)
    float angular_velocity;         // 角速度 (rad/s)
    bool emergency_brake;           // 紧急制动标志
} ControlOutput_t;

/* 编队控制器主结构体 */
typedef struct {
    /* 系统状态 */
    FormationState_t state;         // 当前状态
    uint8_t vehicle_id;             // 本车ID
    uint32_t system_time;           // 系统时间 (ms)
    
    /* 配置参数 */
    ControlParams_t params;         // 控制参数
    FormationConfig_t config;       // 编队配置
    
    /* 车辆状态信息 */
    VehicleState_t vehicles[MAX_VEHICLES];      // 所有车辆状态
    VehicleState_t self_state;                  // 本车状态
    uint8_t num_active_vehicles;                // 活跃车辆数量
    
    /* 障碍物信息 */
    Obstacle_t obstacles[MAX_OBSTACLES];        // 障碍物列表
    uint8_t num_obstacles;                      // 障碍物数量
    
    /* 控制输出 */
    ControlOutput_t control_output;             // 控制输出
    Vector2D_t force_total;                     // 总合力
    
    /* 性能监控 */
    float formation_error;                      // 编队误差
    bool is_converged;                          // 是否收敛
    uint32_t last_update_time;                  // 上次更新时间
    
    /* 通信状态 */
    uint32_t last_comm_time[MAX_VEHICLES];      // 各车辆上次通信时间
    bool comm_status[MAX_VEHICLES];             // 通信状态
} FormationController_t;

/* 全局变量声明 */
extern FormationController_t g_formation_controller;

/* ===== 核心函数接口 ===== */

/**
 * @brief 初始化编队控制器
 * @param controller 控制器指针
 * @param vehicle_id 本车ID
 * @param params 控制参数
 * @return 0表示成功，其他值表示错误
 */
int8_t FormationController_Init(FormationController_t* controller, 
                               uint8_t vehicle_id, 
                               const ControlParams_t* params);

/**
 * @brief 设置编队配置
 * @param controller 控制器指针
 * @param config 编队配置
 * @return 0表示成功，其他值表示错误
 */
int8_t FormationController_SetConfig(FormationController_t* controller,
                                    const FormationConfig_t* config);

/**
 * @brief 更新本车状态
 * @param controller 控制器指针
 * @param position 当前位置
 * @param velocity 当前速度
 * @param heading 当前航向
 * @return 0表示成功，其他值表示错误
 */
int8_t FormationController_UpdateSelfState(FormationController_t* controller,
                                          const Vector2D_t* position,
                                          const Vector2D_t* velocity,
                                          float heading);

/**
 * @brief 更新邻车状态
 * @param controller 控制器指针
 * @param vehicle_id 车辆ID
 * @param state 车辆状态
 * @return 0表示成功，其他值表示错误
 */
int8_t FormationController_UpdateVehicleState(FormationController_t* controller,
                                             uint8_t vehicle_id,
                                             const VehicleState_t* state);

/**
 * @brief 更新障碍物信息
 * @param controller 控制器指针
 * @param obstacles 障碍物数组
 * @param num_obstacles 障碍物数量
 * @return 0表示成功，其他值表示错误
 */
int8_t FormationController_UpdateObstacles(FormationController_t* controller,
                                          const Obstacle_t* obstacles,
                                          uint8_t num_obstacles);

/**
 * @brief 计算控制输出
 * @param controller 控制器指针
 * @return 0表示成功，其他值表示错误
 */
int8_t FormationController_Calculate(FormationController_t* controller);

/**
 * @brief 获取控制输出
 * @param controller 控制器指针
 * @return 控制输出指针
 */
const ControlOutput_t* FormationController_GetOutput(const FormationController_t* controller);

/* ===== 辅助函数接口 ===== */

/**
 * @brief 生成标准编队队形
 * @param type 编队类型
 * @param num_vehicles 车辆数量
 * @param scale 编队尺度
 * @param positions 输出位置数组
 * @return 0表示成功，其他值表示错误
 */
int8_t Formation_Generate(FormationType_t type,
                         uint8_t num_vehicles,
                         float scale,
                         Vector2D_t* positions);

/**
 * @brief 计算两点间距离
 * @param p1 点1
 * @param p2 点2
 * @return 距离值
 */
float Vector2D_Distance(const Vector2D_t* p1, const Vector2D_t* p2);

/**
 * @brief 计算向量模长
 * @param v 向量
 * @return 模长
 */
float Vector2D_Magnitude(const Vector2D_t* v);

/**
 * @brief 向量归一化
 * @param v 向量
 * @return 归一化后的向量
 */
Vector2D_t Vector2D_Normalize(const Vector2D_t* v);

/**
 * @brief 向量加法
 * @param v1 向量1
 * @param v2 向量2
 * @return 和向量
 */
Vector2D_t Vector2D_Add(const Vector2D_t* v1, const Vector2D_t* v2);

/**
 * @brief 向量减法
 * @param v1 向量1
 * @param v2 向量2
 * @return 差向量
 */
Vector2D_t Vector2D_Subtract(const Vector2D_t* v1, const Vector2D_t* v2);

/**
 * @brief 向量数乘
 * @param v 向量
 * @param scalar 标量
 * @return 结果向量
 */
Vector2D_t Vector2D_Scale(const Vector2D_t* v, float scalar);

/**
 * @brief 限制值在指定范围内
 * @param value 输入值
 * @param min_val 最小值
 * @param max_val 最大值
 * @return 限制后的值
 */
float Constrain(float value, float min_val, float max_val);

/**
 * @brief 角度归一化到 [-π, π]
 * @param angle 输入角度
 * @return 归一化后的角度
 */
float NormalizeAngle(float angle);

/**
 * @brief 获取系统时间
 * @return 系统时间 (ms)
 */
uint32_t GetSystemTime(void);

/* ===== 状态机函数 ===== */

/**
 * @brief 编队状态机更新
 * @param controller 控制器指针
 */
void FormationStateMachine_Update(FormationController_t* controller);

/**
 * @brief 检查编队收敛性
 * @param controller 控制器指针
 * @return true表示已收敛，false表示未收敛
 */
bool FormationController_IsConverged(const FormationController_t* controller);

/**
 * @brief 计算编队误差
 * @param controller 控制器指针
 * @return 编队误差值
 */
float FormationController_CalculateError(const FormationController_t* controller);

/* ===== 通信相关函数 ===== */

/**
 * @brief 检查通信状态
 * @param controller 控制器指针
 * @param vehicle_id 车辆ID
 * @return true表示通信正常，false表示通信异常
 */
bool FormationController_CheckCommStatus(const FormationController_t* controller, 
                                        uint8_t vehicle_id);

/**
 * @brief 广播本车状态
 * @param controller 控制器指针
 * @return 0表示成功，其他值表示错误
 */
int8_t FormationController_BroadcastState(const FormationController_t* controller);

/* ===== 安全相关函数 ===== */

/**
 * @brief 检查碰撞风险
 * @param controller 控制器指针
 * @return true表示存在碰撞风险，false表示安全
 */
bool FormationController_CheckCollisionRisk(const FormationController_t* controller);

/**
 * @brief 触发紧急制动
 * @param controller 控制器指针
 */
void FormationController_EmergencyBrake(FormationController_t* controller);

#ifdef __cplusplus
}
#endif

#endif /* FORMATION_ALGORITHM_EMBEDDED_H */ 