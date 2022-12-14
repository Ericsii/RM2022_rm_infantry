#include "rm_infantry/auto_aim_node.hpp"

namespace rm_infantry
{
    AutoAimNode::AutoAimNode(const rclcpp::NodeOptions &options)
    {
        node_ = std::make_shared<rclcpp::Node>("auto_aim", options);
        node_->declare_parameter("armor_is_red", false);
        node_->declare_parameter("camera_name", "mv_camera");
        node_->declare_parameter("imu_name", "imu");
        node_->declare_parameter("auto_start", false);
        node_->declare_parameter("gimbal_cmd_name", "cmd_gimbal");

        auto gimbal_cmd_name = node_->get_parameter("gimbal_cmd_name").as_string();
        std::string camera_name = node_->get_parameter("camera_name").as_string();
        std::string imu_name = node_->get_parameter("imu_name").as_string();
        bool is_red = node_->get_parameter("armor_is_red").as_bool();

        using namespace std::placeholders;
        RCLCPP_INFO(node_->get_logger(), "Creating rcl pub&sub&client.");
        rclcpp::QoS gimbal_cmd_sub_qos_profile(rclcpp::KeepLast(1), best_effort_qos_policy);
        gimbal_cmd_pub_ = node_->create_publisher<rm_interfaces::msg::GimbalCmd>(
            "cmd_gimbal", gimbal_cmd_sub_qos_profile);

        set_mode_srv_ = node_->create_service<rm_interfaces::srv::SetMode>(
            "auto_aim/set_mode", std::bind(&AutoAimNode::set_mode_cb, this, _1, _2));

        //获取本局颜色
        set_color_srv_ = node_->create_service<rm_interfaces::srv::SetColor>(
            "auto_aim/set_color", std::bind(&AutoAimNode::set_color_cb, this, _1, _2));

        wrapper_client_ = std::make_shared<rm_cam::WrapperClient>(
            node_, camera_name, imu_name, std::bind(&AutoAimNode::process_fn, this, _1, _2, _3));
        RCLCPP_INFO(node_->get_logger(), "Create success.");

        sensor_msgs::msg::CameraInfo cam_info;
        if (!wrapper_client_->get_camera_info(cam_info))
        {
            RCLCPP_FATAL(
                node_->get_logger(),
                "Get camera info failed!");
            return;
        }

        std::ostringstream oss;
        oss << "k:";
        for (auto &x : cam_info.k)
        {
            oss << x << " ";
        }
        oss << ",d:";
        for (auto &x : cam_info.d)
        {
            oss << x << " ";
        }
        RCLCPP_INFO(node_->get_logger(), "get camera info: %s", oss.str().c_str());

        // 初始化
        std::vector<double> camera_k(9, 0);
        std::copy_n(cam_info.k.begin(), 9, camera_k.begin());
        std::shared_ptr<rm_auto_aim::ArmorDetector> detector = std::make_shared<rm_auto_aim::ArmorDetectorSVM>(node_, is_red);
        auto_aim_algo_ = std::make_shared<rm_infantry::AutoAimAlgo>(node_, camera_k, cam_info.d, detector);
        auto_aim_algo_->set_target_color(is_red);
        transform_tool_ = std::make_shared<rm_util::CoordinateTranslation>();
        measure_tool_ = std::make_shared<rm_util::MonoMeasureTool>(camera_k, cam_info.d);

#ifdef RM_DEBUG_MODE
        bool auto_start = node_->get_parameter("auto_start").as_bool();
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
        if (auto_start)
        {
            aim_mode = 0x01;
            wrapper_client_->start();
        }
#endif
        RCLCPP_INFO(
            node_->get_logger(),
            "Init finished.");
    }

    void AutoAimNode::process_fn(std_msgs::msg::Header header, cv::Mat &img, geometry_msgs::msg::Quaternion q)
    {
        // 计算时间戳
        double time_stamp_ms = header.stamp.sec * 1e3 + header.stamp.nanosec * 1e-6;

#ifdef RM_DEBUG_MODE
        // tf2 发布当前姿态
        geometry_msgs::msg::TransformStamped pose_tf;
        pose_tf.header.stamp = header.stamp;
        pose_tf.header.frame_id = "world";
        pose_tf.child_frame_id = "camera";
        pose_tf.transform.translation.x = 1.0;
        pose_tf.transform.translation.y = 0.0;
        pose_tf.transform.translation.z = 0.0;
        pose_tf.transform.rotation = q;
        tf_broadcaster_->sendTransform(pose_tf);
        RCLCPP_INFO(
            node_->get_logger(),
            "Get message");
#endif
        // 姿态四元数
        curr_pose_ = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
        auto euler_angles = rm_util::CoordinateTranslation::quat2euler(curr_pose_);
        float c_pitch, c_yaw, c_roll;
        c_pitch = rm_util::rad_to_deg(euler_angles(0));
        c_yaw = rm_util::rad_to_deg(euler_angles(2));
        c_roll = rm_util::rad_to_deg(euler_angles(1));
        (void)c_roll;
        // 装甲板识别
        int ret = auto_aim_algo_->process(time_stamp_ms, img, curr_pose_);

        rm_interfaces::msg::GimbalCmd gimbal_cmd;
        gimbal_cmd.id = gimbal_cmd_id++;
        gimbal_cmd.position.pitch = c_pitch;
        gimbal_cmd.position.yaw = c_yaw;

        if (!ret)
        {
            float target_pitch, target_yaw;
            target_pitch = auto_aim_algo_->getTargetPitch();
            target_yaw = auto_aim_algo_->getTargetYaw();

#ifdef RM_DEBUG_MODE
            /* 仅从图像2D坐标进行目标跟踪
             auto target = auto_aim_algo_->getTarget();
             measure_tool_->calc_view_angle(target.armorDescriptor.centerPoint, target_pitch, target_yaw);
             target_pitch = rm_util::rad_to_deg(auto_aim_algo_->mTarget_pitch);
             target_yaw = rm_util::rad_to_deg(auto_aim_algo_->mTarget_yaw); */
            RCLCPP_INFO(
                node_->get_logger(),
                "c_pitch: %f, c_yaw: %f, c_roll: %f", c_pitch, c_yaw, c_roll);
            RCLCPP_INFO(
                node_->get_logger(),
                "target_pitch: %f, target_yaw: %f", target_pitch, target_yaw);
#endif

            if (this->gimbal_ctrl_flag_)
            {
                gimbal_cmd.position.pitch = target_pitch;
                gimbal_cmd.position.yaw = target_yaw;
                gimbal_cmd_pub_->publish(gimbal_cmd);
            }
        }
        else
        {
            if (this->gimbal_ctrl_flag_)
                gimbal_cmd_pub_->publish(gimbal_cmd);
#ifdef RM_DEBUG_MODE
            RCLCPP_INFO(
                node_->get_logger(),
                "No armors");
#endif
        }
    }

    bool AutoAimNode::set_color_cb(
        const std::shared_ptr<rm_interfaces::srv::SetColor::Request> request,
        std::shared_ptr<rm_interfaces::srv::SetColor::Response> response)
    {
        response->success = true;
        this->color = request->color;
        // wrapper_client_->stop();
        // color is our color
        if (this->color == 0xbb)
        {
            auto_aim_algo_->set_target_color(true);
            RCLCPP_INFO(node_->get_logger(), "set target Color【RED】!");
        }
        else
        {
            auto_aim_algo_->set_target_color(false);
            RCLCPP_INFO(node_->get_logger(), "set target Color【BLUE】!");
        }
        // wrapper_client_->start();
        return true;
    }

    bool AutoAimNode::set_mode_cb(
        const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
        std::shared_ptr<rm_interfaces::srv::SetMode::Response> response)
    {
        // 0x00,休眠模式，0x01:自动射击模式，0x02：自动瞄准模式（不发子弹）,0x03,测试模式,不控制.
        // 0xaa,击打哨兵模式（纳入label为0装甲板）
        response->success = true;
        auto_aim_algo_->set_aim_mode(int(request->mode));
        switch (request->mode)
        {
        case 0x00:
            wrapper_client_->stop();
            RCLCPP_INFO(node_->get_logger(), "【normal】!");
            break;
        case 0x01:
            gimbal_ctrl_flag_ = true;
            shoot_ctrl_flag_ = true;
            wrapper_client_->start();
            RCLCPP_INFO(node_->get_logger(), "【auto aim】!");
            break;
        case 0xaa:
            gimbal_ctrl_flag_ = true;
            shoot_ctrl_flag_ = true;
            wrapper_client_->start();
            RCLCPP_INFO(node_->get_logger(), "【sentry】!");
            break;
        case 0xbb:
            gimbal_ctrl_flag_ = false;
            shoot_ctrl_flag_ = false;
            wrapper_client_->stop();
            break;
        case 0xcc:
            gimbal_ctrl_flag_ = false;
            shoot_ctrl_flag_ = false;
            wrapper_client_->stop();
            break;
        default:
            response->success = false;
        }
        return true;
    }
} // namespace rm_infantry

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_infantry::AutoAimNode)
