#ifndef FORMATION_ALGORITHM_HPP
#define FORMATION_ALGORITHM_HPP

#include <vector>
#include <memory>
#include <unordered_map>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace multi_vehicle_formation {

/**
 * @brief 车辆状态结构体
 */
struct VehicleState {
    uint32_t id;                    // 车辆ID
    Eigen::Vector2d position;       // 位置 (x, y)
    Eigen::Vector2d velocity;       // 速度 (vx, vy)
    double heading;                 // 航向角 (rad)
    double angular_velocity;        // 角速度 (rad/s)
    uint64_t timestamp;             // 时间戳
    bool is_valid;                  // 状态有效性
    
    VehicleState() : id(0), position(0, 0), velocity(0, 0), 
                     heading(0), angular_velocity(0), timestamp(0), is_valid(false) {}
};

/**
 * @brief 编队配置结构体
 */
struct FormationConfig {
    std::string formation_type;              // 编队类型 ("line", "v_shape", "diamond", "circle")
    std::vector<Eigen::Vector2d> positions;  // 期望位置
    double formation_scale;                  // 编队尺度
    uint32_t leader_id;                      // 领导者ID
    Eigen::Vector2d formation_center;        // 编队中心
    double formation_heading;                // 编队航向
    
    FormationConfig() : formation_type("line"), formation_scale(1.0), 
                       leader_id(0), formation_center(0, 0), formation_heading(0) {}
};

/**
 * @brief 控制参数结构体
 */
struct ControlParams {
    // 人工势场参数
    double k_att;               // 吸引力增益
    double k_rep;               // 排斥力增益
    double k_form;              // 编队力增益
    double k_obstacle;          // 障碍物排斥力增益
    double influence_radius;    // 影响半径
    double obstacle_radius;     // 障碍物影响半径
    double safety_distance;     // 安全距离
    
    // 共识算法参数
    double consensus_gain;      // 共识增益
    double damping_factor;      // 阻尼系数
    double convergence_threshold; // 收敛阈值
    
    // 运动限制
    double max_velocity;        // 最大速度
    double max_acceleration;    // 最大加速度
    double max_angular_velocity; // 最大角速度
    
    ControlParams() : k_att(1.0), k_rep(2.0), k_form(1.5), k_obstacle(3.0),
                     influence_radius(3.0), obstacle_radius(2.0), safety_distance(1.0),
                     consensus_gain(0.5), damping_factor(0.8), convergence_threshold(0.1),
                     max_velocity(2.0), max_acceleration(1.0), max_angular_velocity(1.0) {}
};

/**
 * @brief 障碍物结构体
 */
struct Obstacle {
    Eigen::Vector2d position;   // 障碍物位置
    double radius;              // 障碍物半径
    bool is_dynamic;            // 是否为动态障碍物
    Eigen::Vector2d velocity;   // 动态障碍物速度
    
    Obstacle() : position(0, 0), radius(0.5), is_dynamic(false), velocity(0, 0) {}
};

/**
 * @brief 编队算法核心类
 */
class FormationAlgorithm {
public:
    /**
     * @brief 构造函数
     * @param params 控制参数
     */
    explicit FormationAlgorithm(const ControlParams& params = ControlParams());
    
    /**
     * @brief 析构函数
     */
    virtual ~FormationAlgorithm() = default;
    
    /**
     * @brief 设置控制参数
     * @param params 控制参数
     */
    void setControlParams(const ControlParams& params);
    
    /**
     * @brief 获取控制参数
     * @return 控制参数
     */
    const ControlParams& getControlParams() const { return params_; }
    
    /**
     * @brief 设置编队配置
     * @param config 编队配置
     */
    void setFormationConfig(const FormationConfig& config);
    
    /**
     * @brief 获取编队配置
     * @return 编队配置
     */
    const FormationConfig& getFormationConfig() const { return formation_config_; }
    
    /**
     * @brief 更新车辆状态
     * @param vehicle_id 车辆ID
     * @param state 车辆状态
     */
    void updateVehicleState(uint32_t vehicle_id, const VehicleState& state);
    
    /**
     * @brief 更新障碍物信息
     * @param obstacles 障碍物列表
     */
    void updateObstacles(const std::vector<Obstacle>& obstacles);
    
    /**
     * @brief 计算编队控制力
     * @param vehicle_id 目标车辆ID
     * @return 控制力向量
     */
    Eigen::Vector2d calculateFormationForce(uint32_t vehicle_id);
    
    /**
     * @brief 计算速度命令
     * @param vehicle_id 目标车辆ID
     * @return 速度命令 (线速度, 角速度)
     */
    std::pair<double, double> calculateVelocityCommand(uint32_t vehicle_id);
    
    /**
     * @brief 生成标准编队队形
     * @param formation_type 编队类型
     * @param num_vehicles 车辆数量
     * @param scale 编队尺度
     * @return 编队位置向量
     */
    static std::vector<Eigen::Vector2d> generateFormation(
        const std::string& formation_type, int num_vehicles, double scale = 1.0);
    
    /**
     * @brief 检查编队收敛性
     * @return 是否收敛
     */
    bool isFormationConverged();
    
    /**
     * @brief 计算编队误差
     * @return 平均编队误差
     */
    double calculateFormationError();
    
    /**
     * @brief 获取邻居车辆列表
     * @param vehicle_id 车辆ID
     * @param radius 搜索半径
     * @return 邻居车辆ID列表
     */
    std::vector<uint32_t> getNeighbors(uint32_t vehicle_id, double radius);

private:
    /**
     * @brief 计算吸引力
     * @param vehicle_id 车辆ID
     * @param target_position 目标位置
     * @return 吸引力向量
     */
    Eigen::Vector2d calculateAttractiveForce(uint32_t vehicle_id, 
                                            const Eigen::Vector2d& target_position);
    
    /**
     * @brief 计算排斥力
     * @param vehicle_id 车辆ID
     * @param repulsive_position 排斥源位置
     * @return 排斥力向量
     */
    Eigen::Vector2d calculateRepulsiveForce(uint32_t vehicle_id,
                                           const Eigen::Vector2d& repulsive_position);
    
    /**
     * @brief 计算编队约束力
     * @param vehicle_id 车辆ID
     * @return 编队约束力向量
     */
    Eigen::Vector2d calculateFormationConstraintForce(uint32_t vehicle_id);
    
    /**
     * @brief 计算障碍物排斥力
     * @param vehicle_id 车辆ID
     * @return 障碍物排斥力向量
     */
    Eigen::Vector2d calculateObstacleRepulsiveForce(uint32_t vehicle_id);
    
    /**
     * @brief 计算共识控制项
     * @param vehicle_id 车辆ID
     * @return 共识控制向量
     */
    Eigen::Vector2d calculateConsensusControl(uint32_t vehicle_id);
    
    /**
     * @brief 限制速度命令
     * @param linear_vel 线速度
     * @param angular_vel 角速度
     * @return 限制后的速度命令
     */
    std::pair<double, double> limitVelocityCommand(double linear_vel, double angular_vel);
    
    /**
     * @brief 获取车辆在编队中的期望位置
     * @param vehicle_id 车辆ID
     * @return 期望位置
     */
    Eigen::Vector2d getDesiredPosition(uint32_t vehicle_id);

private:
    ControlParams params_;                                      // 控制参数
    FormationConfig formation_config_;                          // 编队配置
    std::unordered_map<uint32_t, VehicleState> vehicle_states_; // 车辆状态映射
    std::vector<Obstacle> obstacles_;                           // 障碍物列表
    
    // 算法内部状态
    std::unordered_map<uint32_t, Eigen::Vector2d> force_history_;   // 力历史记录
    std::unordered_map<uint32_t, double> error_history_;            // 误差历史记录
    
    // 日志记录器
    rclcpp::Logger logger_;
};

/**
 * @brief 编队生成器类 - 用于生成各种标准编队
 */
class FormationGenerator {
public:
    /**
     * @brief 生成直线编队
     * @param num_vehicles 车辆数量
     * @param spacing 车辆间距
     * @param angle 编队角度
     * @return 编队位置向量
     */
    static std::vector<Eigen::Vector2d> generateLineFormation(
        int num_vehicles, double spacing = 2.0, double angle = 0.0);
    
    /**
     * @brief 生成V形编队
     * @param num_vehicles 车辆数量
     * @param spacing 车辆间距
     * @param angle V形角度
     * @return 编队位置向量
     */
    static std::vector<Eigen::Vector2d> generateVFormation(
        int num_vehicles, double spacing = 2.0, double angle = M_PI/4);
    
    /**
     * @brief 生成菱形编队
     * @param num_vehicles 车辆数量
     * @param size 菱形尺寸
     * @return 编队位置向量
     */
    static std::vector<Eigen::Vector2d> generateDiamondFormation(
        int num_vehicles, double size = 2.0);
    
    /**
     * @brief 生成圆形编队
     * @param num_vehicles 车辆数量
     * @param radius 圆形半径
     * @return 编队位置向量
     */
    static std::vector<Eigen::Vector2d> generateCircleFormation(
        int num_vehicles, double radius = 2.0);
    
    /**
     * @brief 生成矩形编队
     * @param num_vehicles 车辆数量
     * @param width 矩形宽度
     * @param height 矩形高度
     * @return 编队位置向量
     */
    static std::vector<Eigen::Vector2d> generateRectangleFormation(
        int num_vehicles, double width = 4.0, double height = 2.0);
};

} // namespace multi_vehicle_formation

#endif // FORMATION_ALGORITHM_HPP 