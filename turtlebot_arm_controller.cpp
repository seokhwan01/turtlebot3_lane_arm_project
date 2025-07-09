/**
 * @file 
 * @brief 
 *
 * This program captures frames from a webcam using OpenCV,
 * retrieves the raw frame's width and height, compresses
 * the frame to JPEG format, and then decodes it to obtain
 * the compressed image's dimensions.
 *
 * Author: Rujin Kim
 * Date: 2025-05-17
 */

#include <rclcpp/rclcpp.hpp>
#include <turtlebot_cosmo_interface/srv/moveit_control.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_array.hpp>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Geometry>  
#include <vector>
#include <tuple>
#include <chrono>
#include <thread>


#define NEW  0

#define PI 3.14159265358979323846
class TurtlebotArmController : public rclcpp::Node {
public:
    TurtlebotArmController(const rclcpp::NodeOptions &options) : Node("turtlebot_arm_controller",options),
    node_(std::make_shared<rclcpp::Node>("move_group_interface")),           // Create an additional ROS node
    move_group_interface_(node_, "arm"),                                     // Initialize MoveGroupInterface for controlling the arm
    executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()) // Create a single-threaded executor
    {
        service_ = this->create_service<turtlebot_cosmo_interface::srv::MoveitControl>(
            "moveit_control", std::bind(&TurtlebotArmController::handleMoveitControl, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Ready to receive MoveitControl commands.");
        executor_->add_node(node_);
        executor_thread_ = std::thread([this]() {
            RCLCPP_INFO(node_->get_logger(), "Starting executor thread");    // Log message indicating the thread start
            executor_->spin();                                               // Run the executor to process callbacks
        });
    }

    // 추가
    ~TurtlebotArmController() {
        if (executor_) {
            executor_->cancel();
        }
        if (executor_thread_.joinable()) {
            executor_thread_.join();
        }
    }

    void printCurrentPose() {
        auto current_pose = move_group_interface_.getCurrentPose().pose;     // Get the current pose
        std::cout << "Current Pose:" << std::endl;

        // 쿼터니언 가져오기
        const auto& orientation = current_pose.orientation;

        // tf2::Quaternion 객체로 변환
        tf2::Quaternion quaternion(orientation.x, orientation.y, orientation.z, orientation.w);

        // RPY 계산
        double roll, pitch, yaw;
        tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

        // RPY 출력
        std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;

        std::cout<<"yaw : "<<yaw<< std::endl;  
        std::cout << "Position: (" << current_pose.position.x << ", "
                << current_pose.position.y << ", "
                << current_pose.position.z << ")" << std::endl;
        std::cout << "Orientation: (" << current_pose.orientation.x << ", "
                << current_pose.orientation.y << ", "
                << current_pose.orientation.z << ", "
                << current_pose.orientation.w << ")" << std::endl;
    }

    geometry_msgs::msg::Quaternion rpyToQuaternion(double roll, double pitch, double yaw) {
        Eigen::Quaterniond q;
        q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

        geometry_msgs::msg::Quaternion quaternion;
        quaternion.w = q.w();
        quaternion.x = q.x();
        quaternion.y = q.y();
        quaternion.z = q.z();
        return quaternion;
    }
#if NEW

// 수정
    bool planAndExecute(moveit::planning_interface::MoveGroupInterface& move_group_interface, moveit::planning_interface::MoveGroupInterface::Plan& plan) {
        bool success = static_cast<bool>(move_group_interface.plan(plan));
        if (!success) {
        RCLCPP_ERROR(this->get_logger(), "Planning failed.");
        return false;
        }
        auto exec_result = move_group_interface.execute(plan);
        if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Execution failed with error code %d", exec_result.val);
            return false;
        }
        return true;
        }
    
#else
// 이전
    // bool planAndExecute(moveit::planning_interface::MoveGroupInterface& move_group_interface, moveit::planning_interface::MoveGroupInterface::Plan& plan) {
       
       
    //     move_group_interface.clearPoseTargets();
    //     bool success = static_cast<bool>(move_group_interface.plan(plan));
        

    //    if (!success) {
    //     RCLCPP_ERROR(this->get_logger(), "Planning failed.");
    //     RCLCPP_ERROR(this->get_logger(), "retrying move.");
        
    //     return false;
    //     }
    //     auto exec_result = move_group_interface.execute(plan);
    //     if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
    //         RCLCPP_ERROR(this->get_logger(), "Execution failed with error code %d", exec_result.val);
    //         return false;
    //     }
    //     return true;
        
    //     }
    bool planAndExecute(moveit::planning_interface::MoveGroupInterface& move_group_interface,
                    moveit::planning_interface::MoveGroupInterface::Plan& plan,
                    rclcpp::Logger logger)
{
    auto result = move_group_interface.plan(plan);
    RCLCPP_INFO(logger, "[MoveIt] Plan result code: %d", result.val);
    switch (result.val) {
        case moveit::core::MoveItErrorCode::SUCCESS:
            RCLCPP_INFO(logger, "Planning succeeded.");
            break;
        case moveit::core::MoveItErrorCode::PLANNING_FAILED:
            RCLCPP_ERROR(logger, "Planning failed: Unable to find a valid path.");
            break;
        case moveit::core::MoveItErrorCode::NO_IK_SOLUTION:
            RCLCPP_ERROR(logger, "Planning failed: No inverse kinematics solution for target pose.");
            break;
        case moveit::core::MoveItErrorCode::INVALID_MOTION_PLAN:
            RCLCPP_ERROR(logger, "Planning failed: Motion plan invalid.");
            break;
        case moveit::core::MoveItErrorCode::FAILURE:
        default:
            RCLCPP_ERROR(logger, "Planning failed: Unknown or general failure.");
            break;
    }
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        auto exec_result = move_group_interface.execute(plan);
        if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(logger, "Execution succeeded.");
            return true;
        } else {
            RCLCPP_ERROR(logger, "Execution failed. Result code: %d", exec_result.val);
            return false;
        }
    }
    return false;
}
    
#endif

#if NEW
// 수정    
    geometry_msgs::msg::Quaternion multiply(const geometry_msgs::msg::Quaternion& q1, const geometry_msgs::msg::Quaternion& q2) {
        tf2::Quaternion tf_q1(q1.x, q1.y, q1.z, q1.w);
        tf2::Quaternion tf_q2(q2.x, q2.y, q2.z, q2.w);
        tf2::Quaternion tf_result = tf_q1 * tf_q2;
    
        geometry_msgs::msg::Quaternion result;
        result.x = tf_result.x();
        result.y = tf_result.y();
        result.z = tf_result.z();
        result.w = tf_result.w();
        return result;
    }
#else
// 이전
    geometry_msgs::msg::Quaternion multiply(const geometry_msgs::msg::Quaternion& q2, const geometry_msgs::msg::Quaternion& q1) {
       geometry_msgs::msg::Quaternion out;
       out.w = q2.w*q1.w - q2.x*q1.x - q2.y*q1.y - q2.z*q1.z;
       out.x = q2.w*q1.x + q2.x*q1.w + q2.y*q1.z - q2.z*q1.y;
       out.y = q2.w*q1.y - q2.x*q1.z + q2.y*q1.w + q2.z*q1.x;
       out.z = q2.w*q1.z + q2.x*q1.y - q2.y*q1.x + q2.z*q1.w;
       return out;
    }
#endif


private:
    void handleMoveitControl(const std::shared_ptr<turtlebot_cosmo_interface::srv::MoveitControl::Request> req,
                             std::shared_ptr<turtlebot_cosmo_interface::srv::MoveitControl::Response> res) {



        RCLCPP_INFO(this->get_logger(), "서비스 콜백 받음");
        RCLCPP_INFO(this->get_logger(), "요청 메시지 내용:");
        // RCLCPP_INFO(this->get_logger(), "Header: frame_id = %s", req->header.frame_id.c_str());
        // RCLCPP_INFO(this->get_logger(), "Header: stamp = %u.%u", req->header.stamp.sec, req->header.stamp.nanosec);
        RCLCPP_INFO(this->get_logger(), "  cmd: %d", req->cmd);
        RCLCPP_INFO(this->get_logger(), "  posename: %s", req->posename.c_str());
        RCLCPP_INFO(this->get_logger(), "  waypoints 개수: %ld", req->waypoints.poses.size());
        for (size_t i = 0; i < req->waypoints.poses.size(); ++i) {
            const auto& pose = req->waypoints.poses[i];
            RCLCPP_INFO(this->get_logger(), "  [%ld] Position: x=%.3f, y=%.3f, z=%.3f", i, pose.position.x, pose.position.y, pose.position.z);
            RCLCPP_INFO(this->get_logger(), "  [%ld] Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f", i,
                        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        }
            
        
        // moveit core 의 arm 그룹을 사용하기위한 선언
        using moveit::planning_interface::MoveGroupInterface;
        auto arm_interface = MoveGroupInterface(shared_from_this(), "arm");
        arm_interface.setPlanningPipelineId("move_group");
        // arm_interface.setPlanningTime(10.0); 
        arm_interface.setPlanningTime(30.0); 



        // arm_interface.setGoalPositionTolerance(0.01);
        // arm_interface.setGoalOrientationTolerance(0.05);
        arm_interface.setGoalPositionTolerance(0.02);
        arm_interface.setGoalOrientationTolerance(0.5); //0.2 x




        // arm_interface.setGoalPositionTolerance(0.005);
        // arm_interface.setGoalOrientationTolerance(0.025);
        
        // arm_interface.setGoalPositionTolerance(0.0025);
        // arm_interface.setGoalOrientationTolerance(0.0125);

        // moveit core 의 gripper 그룹을 사용하기위한 선언
        auto gripper_interface = MoveGroupInterface(shared_from_this(), "gripper");
        

        // 좌표를 통한 이동 
#if NEW
// 수정




        if (req->cmd == 0) {
            std::vector<geometry_msgs::msg::Pose> waypoints(req->waypoints.poses.begin(), req->waypoints.poses.end());
            if (waypoints.empty()) {
                RCLCPP_WARN(this->get_logger(), "No waypoints provided for cmd 0.");
                res->response = false;
                return;
            }
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto _pose = waypoints[0];
            auto current_pose = arm_interface.getCurrentPose().pose;
            ///
            RCLCPP_INFO(this->get_logger(), "Current pose: x: %f y: %f z: %f ox: %f oy: %f oz: %f ow: %f",
            current_pose.position.x,
            current_pose.position.y,
            current_pose.position.z,
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w
            );

            float target_z = _pose.position.z;
            float target_x = _pose.position.x;
            float target_y = _pose.position.y;
            //double yaw = atan2(target_y, target_x);
            // double yaw = atan2(target_y - current_pose.position.y, target_x - current_pose.position.x);
            double yaw = atan2(target_y, target_x+0.08);

            geometry_msgs::msg::Quaternion multiply_quaternion = multiply(rpyToQuaternion(0, 0, yaw), rpyToQuaternion(0, PI/2, 0));

            geometry_msgs::msg::Pose target_pose;
            target_pose.orientation = multiply_quaternion;
            target_pose.position.x = target_x;
            target_pose.position.y = target_y;
            target_pose.position.z = target_z;

            arm_interface.setPoseTarget(target_pose);
            res->response = planAndExecute(arm_interface, plan,this->get_logger());
        }

#else

   
        // }
       if (req->cmd == 0) {
        if (req->waypoints.poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "No pose received.");
            return;
        }
        auto current_pose = arm_interface.getCurrentPose().pose;
        auto target_pose = req->waypoints.poses[0];  // :흰색_확인_표시: base_link 기준의 마커 pose (x, y, z + orientation)
        // 디버깅용 출력
        //target_pose.position.z=0.02;
        // target_pose.position.z=0.15;

        target_pose.position.x = target_pose.position.x+0.2;

        RCLCPP_INFO(this->get_logger(), "Target Pose Received: x=%.3f, y=%.3f, z=%.3f",
            target_pose.position.x,
            target_pose.position.y,
            target_pose.position.z);
        RCLCPP_INFO(this->get_logger(), "current Pose: x=%.3f, y=%.3f , z=%.3f",
                    current_pose.position.x,
                    current_pose.position.y,
                    current_pose.position.z);


        target_pose.orientation.x=0;
        target_pose.orientation.y=0;
        target_pose.orientation.z=0;
        target_pose.orientation.w=1;

//add~~~~~~~~~~~~~~~~~~~~~~`
//         // RPY: roll=0, pitch=10도, yaw=atan2로 계산된 방향
// double yaw = atan2(target_y, target_x + 0.08);
// geometry_msgs::msg::Quaternion q_stable = multiply(
//     rpyToQuaternion(0, 0.17, yaw),  // pitch 10도
//     rpyToQuaternion(0, M_PI/2, 0)   // 기존 회전 보정 유지
// );
// target_pose.orientation = q_stable;

        RCLCPP_INFO(this->get_logger(), "Target Orientation: qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f",
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z,
            target_pose.orientation.w);




        // target_pose.position.x=target_pose.position.x-current_pose.position.x;
        //  target_pose.position.y  = target_pose.position.y-current_pose.position.y;
        //  target_pose.position.z = target_pose.position.z-current_pose.position.z;
        //  RCLCPP_INFO(this->get_logger(), "Target Pose substract: x=%.3f, y=%.3f, z=%.3f",
        //     target_pose.position.x,
        //     target_pose.position.y,
        //     target_pose.position.z);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        arm_interface.setStartStateToCurrentState();
        arm_interface.setPoseTarget(target_pose);
        std::cout << "[INFO] Calling planAndExecute..." << std::endl;
        // 1. Gripper open
        gripper_interface.setNamedTarget("open");
        if (gripper_interface.move() != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(this->get_logger(), "Gripper failed to open.");
            res->response = false;
            return;
        }
        planAndExecute(arm_interface, plan, this->get_logger());
        // 3. Gripper close
        std::this_thread::sleep_for(std::chrono::seconds(2));
        gripper_interface.setNamedTarget("close");
        if (gripper_interface.move() != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(this->get_logger(), "Gripper failed to close.");
            res->response = false;
            return;
        }

        arm_interface.setNamedTarget("put_state");
        res->response = (arm_interface.move() == moveit::core::MoveItErrorCode::SUCCESS);

        // 1. Gripper open
        gripper_interface.setNamedTarget("open");
        if (gripper_interface.move() != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(this->get_logger(), "Gripper failed to open.");
            res->response = false;
            return;
        }

        arm_interface.setNamedTarget("lane_tracking_01");
        res->response = (arm_interface.move() == moveit::core::MoveItErrorCode::SUCCESS);



    }   
#endif

        else if (req->cmd == 1) {
            arm_interface.setNamedTarget(req->posename);
            // res->response = (arm_interface.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            res->response = (arm_interface.move() == moveit::core::MoveItErrorCode::SUCCESS);

        } 

        else if (req->cmd == 2) {
            gripper_interface.setNamedTarget(req->posename);
            // res->response = (gripper_interface.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            res->response = (gripper_interface.move() == moveit::core::MoveItErrorCode::SUCCESS);

        } 

        else if (req->cmd == 3){
            std::vector<geometry_msgs::msg::Pose> waypoints;
            for (const auto &pose : req->waypoints.poses) {
                waypoints.push_back(pose);
            }
            auto _pose = waypoints[0];
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto current_pose = arm_interface.getCurrentPose().pose;
            float target_z = _pose.position.z;
            float target_x = _pose.position.x;
            float target_y = _pose.position.y;
            double yaw = atan2(target_x, target_y);  
            std::cout<<"yaw"<< yaw << std::endl;
            geometry_msgs::msg::Quaternion multiply_quaternion = multiply(rpyToQuaternion(0, 0, yaw),rpyToQuaternion(0, 0, PI/2));
            std::cout << "multiply_quaternion: " << multiply_quaternion.x << " " << multiply_quaternion.y << " " << multiply_quaternion.z << " " << multiply_quaternion.w << std::endl;

            current_pose = arm_interface.getCurrentPose().pose;
            auto target_pose = [multiply_quaternion, target_x, target_y, target_z]{
                geometry_msgs::msg::Pose msg;
                msg.orientation = multiply_quaternion;
                msg.position.x = target_x;
                msg.position.y = target_y;
                msg.position.z = target_z; 
                return msg;
            }();
            arm_interface.setPoseTarget(target_pose);
            planAndExecute(arm_interface, plan,this->get_logger());
        }


        else if (req->cmd == 4){
            std::vector<geometry_msgs::msg::Pose> waypoints;
            for (const auto &pose : req->waypoints.poses) {
                waypoints.push_back(pose);
            }
            auto _pose = waypoints[0];
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto current_pose = arm_interface.getCurrentPose().pose;
            float target_z = _pose.position.z;
            float target_x = _pose.position.x;
            float target_y = _pose.position.y;
            float quaternion_x = _pose.orientation.x;
            float quaternion_y = _pose.orientation.y;
            float quaternion_z = _pose.orientation.z;
            float quaternion_w = _pose.orientation.w;
            
            // double yaw = atan2(target_x, target_y);  
            // std::cout<<yaw;

            current_pose = arm_interface.getCurrentPose().pose;
            auto target_pose = [quaternion_x, quaternion_y, quaternion_z, quaternion_w, target_x, target_y, target_z]{
                geometry_msgs::msg::Pose msg;
                msg.orientation.x = quaternion_x;
                msg.orientation.y = quaternion_y;
                msg.orientation.z = quaternion_z;
                msg.orientation.w = quaternion_w;
                msg.position.x = target_x;
                msg.position.y = target_y;
                msg.position.z = target_z; 
                return msg;
            }();
            arm_interface.setPoseTarget(target_pose);
            planAndExecute(arm_interface, plan,this->get_logger());
        }

        else if (req->cmd==9){
            printCurrentPose();
        }

#if NEW
//수정
        else {
            RCLCPP_WARN(this->get_logger(), "Received unknown command: %d", req->cmd);
            res->response = false;
        }
#else
//이전
        else {
           res->response = false;
        }
#endif
    }

    rclcpp::Service<turtlebot_cosmo_interface::srv::MoveitControl>::SharedPtr service_;
    rclcpp::Node::SharedPtr node_;                                         // Additional ROS node pointer
    moveit::planning_interface::MoveGroupInterface move_group_interface_;  // MoveIt interface for controlling the arm
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;  // Single-threaded executor
    std::thread executor_thread_;                                          // Thread to run the executor
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);    // Allow automatic parameter declaration
    node_options.use_intra_process_comms(false);                           // Disable intra-process communication
    auto node = std::make_shared<TurtlebotArmController>(node_options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}