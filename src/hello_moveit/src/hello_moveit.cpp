#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp> // 修复头文件
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>
#include <thread>

// 辅助函数：角度转弧度
double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

class SoloFactoryPicker {
public:
    SoloFactoryPicker(const std::shared_ptr<rclcpp::Node>& node)
        : node_(node),
          arm_group_(node, "arm"),
          gripper_group_(node, "gripper")
    {
        // 1. 设置规划参数
        arm_group_.setPlanningTime(5.0);
        arm_group_.setMaxVelocityScalingFactor(0.5); // 速度慢点更安全
        arm_group_.setMaxAccelerationScalingFactor(0.5);
        arm_group_.setPoseReferenceFrame("base_link");
        arm_group_.setStartStateToCurrentState();

        // ==================== 【关键修正】 ====================
        // 2. 启动状态监视器并等待数据同步
        RCLCPP_INFO(node_->get_logger(), "正在同步机械臂状态...");
        bool state_valid = arm_group_.startStateMonitor(2.0);
        if (state_valid) {
            RCLCPP_INFO(node_->get_logger(), "✅ 机械臂状态同步成功！");
        } else {
            RCLCPP_WARN(node_->get_logger(), "⚠️ 状态同步超时，获取的可能是旧数据！");
        }
        // ======================================================
    }

    // 动作 1: 移动到目标位姿 (带降级策略)
    // 策略：先尝试严格的 Pose (位置+姿态)，如果失败，则降级为 Position Only (只保位置，不管朝向)
    bool moveToPose(const geometry_msgs::msg::Pose& target) {
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = false;

        // --- 尝试 1: 严格的位姿规划 (位置 + 姿态) ---
        RCLCPP_INFO(node_->get_logger(), "尝试完整位姿规划 (Position + Orientation)...");
        arm_group_.setPoseTarget(target);

        // 允许位置误差 1cm (默认是 0.0001m，太严格了)
        arm_group_.setGoalPositionTolerance(0.02); 

        // 允许角度误差 0.1弧度 (约 5.7度，默认也是极小)
        arm_group_.setGoalOrientationTolerance(0.1); 

        // 设置规划时间长一点，给 KDL 多一点运算时间
        arm_group_.setPlanningTime(5.0);
        
        if (arm_group_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(node_->get_logger(), "✅ 完整位姿规划成功，执行中...");
            return arm_group_.execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS;
        }
        
        RCLCPP_WARN(node_->get_logger(), "⚠️ 完整位姿不可达！尝试降级方案：忽略姿态，只对齐位置...");

        // --- 尝试 2: 降级为仅位置规划 (Position Only) ---
        // 1. 必须先清除之前的 Pose 目标
        arm_group_.clearPoseTargets(); 
        
        // 2. 设置仅位置目标 (x, y, z)
        arm_group_.setPositionTarget(target.position.x, target.position.y, target.position.z);
        
        // 3. (可选) 给位置求解稍微放宽一点公差，提高成功率
        arm_group_.setGoalPositionTolerance(0.03);       // 2cm
        arm_group_.setGoalOrientationTolerance(3.14);    // 基本不约束朝向
        arm_group_.setPlanningTime(10.0);
        arm_group_.setNumPlanningAttempts(30);
        arm_group_.allowReplanning(true);

        if (arm_group_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(node_->get_logger(), "✅ 仅位置规划成功，执行中...");
            return arm_group_.execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS;
        }

        RCLCPP_ERROR(node_->get_logger(), "❌ 所有规划尝试均失败！目标完全不可达。");
        return false;
    }

    // 动作 2: 直线移动 (笛卡尔路径)
    // 注意：SO-100 是 4/5 自由度，直线运动极易失败。如果失败，会自动降级为普通移动。
    bool moveCartesian(double dx, double dy, double dz) {
        std::vector<geometry_msgs::msg::Pose> waypoints;
        geometry_msgs::msg::Pose target_pose = arm_group_.getCurrentPose().pose;
        
        target_pose.position.x += dx;
        target_pose.position.y += dy;
        target_pose.position.z += dz;
        waypoints.push_back(target_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        
        // 计算笛卡尔路径
        double fraction = arm_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        
        if (fraction > 0.9) { // 如果路径规划成功率超过 90%
            RCLCPP_INFO(node_->get_logger(), "笛卡尔路径规划成功 (%.2f%%)", fraction * 100.0);
            return arm_group_.execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS;
        } else {
            RCLCPP_WARN(node_->get_logger(), "笛卡尔路径失败 (%.2f%%)，尝试使用关节空间规划...", fraction * 100.0);
            return moveToPose(target_pose); // 降级方案
        }
    }

    // 动作 3: 控制夹爪
    void controlGripper(double degrees) {
        // 【关键修正】将角度转换为弧度
        double rad = deg2rad(degrees);
        
        RCLCPP_INFO(node_->get_logger(), "夹爪目标: %.1f 度 (%.3f rad)", degrees, rad);
        gripper_group_.setJointValueTarget({rad});
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (gripper_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            gripper_group_.execute(plan);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "夹爪规划失败！");
        }
        // 给一点时间让物理夹爪完成动作
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    void runDemo() {

        // 显式启动监听，并等待 2 秒让数据传过来
        RCLCPP_INFO(node_->get_logger(), "正在同步机械臂状态...");
        
        // startStateMonitor(wait_time) 会自动处理订阅并等待数据
        bool state_valid = arm_group_.startStateMonitor(2.0);

        // 1. 获取当前作为预备点
        arm_group_.setPoseReferenceFrame("world");
        geometry_msgs::msg::Pose start_pose = arm_group_.getCurrentPose().pose;
        RCLCPP_INFO(node_->get_logger(), "当前 Start Pose: [x=%.3f, y=%.3f, z=%.3f]", 
            start_pose.position.x, start_pose.position.y, start_pose.position.z);

        // 2. 预备抓取位置 (在当前位置基础上微调，或者你可以硬编码一个位置)
        // geometry_msgs::msg::Pose pick_pose = start_pose;
        // 假设方块在前方
        // pick_pose.position.x = 0.2; 
        // pick_pose.position.y = 0.0;
        // pick_pose.position.z = 0.15;
        
        // 预抓取位置  
        geometry_msgs::msg::Pose pre_pick_pose;
        // 填充 Position
        pre_pick_pose.position.x = 0.18228;
        pre_pick_pose.position.y = -0.1176;
        pre_pick_pose.position.z = 0.07013;
        // 填充 Orientation (Quaternion)
        pre_pick_pose.orientation.x = -0.025595;
        pre_pick_pose.orientation.y =  0.69296;
        pre_pick_pose.orientation.z = 0.0077838;
        pre_pick_pose.orientation.w = 0.72048;
        
        // 抓取位置
        geometry_msgs::msg::Pose pick_pose;
        // 填充 Position
        pick_pose.position.x = 0.18523;
        pick_pose.position.y = -0.11938;
        pick_pose.position.z = 0.035587;
        // 填充 Orientation (Quaternion)
        pick_pose.orientation.x = -0.00096829;
        pick_pose.orientation.y = 0.73001;
        pick_pose.orientation.z = 0.031547;
        pick_pose.orientation.w = 0.6827;

        if (moveToPose(pre_pick_pose)) {
            RCLCPP_INFO(node_->get_logger(), "已到抓取点上方！"); 
            
            RCLCPP_INFO(node_->get_logger(), "Step 1: 打开夹爪...");
            controlGripper(50); // 50度

            if (moveToPose(pick_pose)){  // 

                RCLCPP_INFO(node_->get_logger(), "已到抓取点！！！"); 


                RCLCPP_INFO(node_->get_logger(), "Step 3: 闭合夹爪...");
                controlGripper(16); // 10度 (根据物体大小调整)

                RCLCPP_INFO(node_->get_logger(), "Step 4: 抬起...");
                moveCartesian(0, 0, 0.06);

                //移动到放置点 * 需要补充代码！
                geometry_msgs::msg::Pose place_pose;
                // 填充 Position
                place_pose.position.x = 0.123;
                place_pose.position.y = 0.191;
                place_pose.position.z = 0.037;
                
                // 填充 Orientation (Quaternion)
                place_pose.orientation.x = 0.053;
                place_pose.orientation.y = 0.702;
                place_pose.orientation.z = -0.042;
                place_pose.orientation.w = 0.709;

                geometry_msgs::msg::Pose pre_place_pose = place_pose;
                pre_place_pose.position.z +=0.06;

                if (moveToPose(pre_place_pose)) {
                    RCLCPP_INFO(node_->get_logger(), "已到放置点上方！");    
                    
                    RCLCPP_INFO(node_->get_logger(), "Step 2: 下降抓取...");
                    
                    if (moveCartesian(0, 0, -0.06)){

                        RCLCPP_INFO(node_->get_logger(), "已到放置点！");

                        RCLCPP_INFO(node_->get_logger(), "Step 6: 放下...");
                        controlGripper(40); // 打开

                        RCLCPP_INFO(node_->get_logger(), "Step 4: 抬起...");
                        moveCartesian(0, 0, 0.06); // 抬起 5cm

                        RCLCPP_INFO(node_->get_logger(), "演示结束！");
                    } else {
                        RCLCPP_ERROR(node_->get_logger(), "******无法到达放置点，跳过放置步骤！");
                    }

                } else {
                        RCLCPP_ERROR(node_->get_logger(), "******无法到达放置点上方，跳过预放置步骤！");
                    }               

            } else {
                RCLCPP_ERROR(node_->get_logger(), "******无法到达抓取点，跳过抓取步骤！");
            }           

        } else {
            RCLCPP_ERROR(node_->get_logger(), "******无法到达抓取点上方，跳过预抓取步骤！");
        }        
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    moveit::planning_interface::MoveGroupInterface arm_group_;
    moveit::planning_interface::MoveGroupInterface gripper_group_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<rclcpp::Node>(
        "hello_moveit_node",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // 开启后台线程处理 ROS 回调
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    SoloFactoryPicker picker(node);
    picker.runDemo();

    rclcpp::shutdown();
    return 0;
}
