#include "rm_infantry/auto_aim_algo.hpp"

namespace rm_infantry
{
    AutoAimAlgo::AutoAimAlgo(rclcpp::Node::SharedPtr node,
                             std::vector<double> camera_intrinsic,
                             std::vector<double> camera_distortion,
                             std::shared_ptr<rm_auto_aim::ArmorDetector> armor_detector) : node_(node), armor_detector_(armor_detector)
    {
        mono_loacation_tool_ = std::make_shared<rm_util::MonoMeasureTool>(camera_intrinsic, camera_distortion);

        float realWidth, realHeight, half_x, half_y;

        // 小装甲
        realHeight = 6.3;
        realWidth = 12.8;
        half_x = realWidth / 2;
        half_y = realHeight / 2;
        mSmallArmorPoints.emplace_back(-half_x, half_y, 0.);
        mSmallArmorPoints.emplace_back(-half_x, -half_y, 0);
        mSmallArmorPoints.emplace_back(half_x, -half_y, 0);
        mSmallArmorPoints.emplace_back(half_x, half_y, 0);

        // 大装甲
        realWidth = 22.2;
        half_x = realWidth / 2;
        half_y = realHeight / 2;
        mBigArmorPoints.emplace_back(-half_x, half_y, 0.);
        mBigArmorPoints.emplace_back(-half_x, -half_y, 0);
        mBigArmorPoints.emplace_back(half_x, -half_y, 0);
        mBigArmorPoints.emplace_back(half_x, half_y, 0);

        mono_loacation_tool_ = std::make_shared<rm_util::MonoMeasureTool>(camera_intrinsic, camera_distortion);

        point_pub_ = node_->create_publisher<geometry_msgs::msg::PointStamped>(
            "world/clicked_point", 10);
        cam_point_pub_ = node_->create_publisher<geometry_msgs::msg::PointStamped>(
            "camera/clicked_point", 10);

        node_->declare_parameter("yaw_offset", -2.);
        node_->get_parameter("yaw_offset", yaw_offset);
        node_->declare_parameter("pitch_offset", 0.);
        node_->get_parameter("pitch_offset", pitch_offset);

        double fliter_p = 5000., fliter_v = 5000., fliter_R = 0.01;
        node_->declare_parameter("fliter_p", fliter_p);
        node_->get_parameter("fliter_p", fliter_p);
        node_->declare_parameter("fliter_v", fliter_v);
        node_->get_parameter("fliter_v", fliter_v);
        node_->declare_parameter("fliter_R", fliter_R);
        node_->get_parameter("fliter_R", fliter_R);

        Eigen::MatrixXd Q(6, 6);
        double sigma_p = fliter_p, sigma_v = fliter_v;
        sigma_p = sigma_p * sigma_p;
        sigma_v = sigma_v * sigma_v;
        Q << sigma_p, 0., 0., 0., 0., 0.,
            0., sigma_p, 0., 0., 0., 0.,
            0., 0., sigma_p, 0., 0., 0.,
            0., 0., 0., sigma_v, 0., 0.,
            0., 0., 0., 0., sigma_v, 0.,
            0., 0., 0., 0., 0., sigma_v;

        Eigen::MatrixXd R(3, 3);
        R = Eigen::MatrixXd::Identity(3, 3) * fliter_R;

        std::stringstream ss;
        ss << "Kalman param: " << std::endl;
        ss << "Q = " << std::endl
           << Q << std::endl;
        ss << "R = " << std::endl
           << R << std::endl;
        RCLCPP_INFO(node_->get_logger(), "%s", ss.str().c_str());

        auto ekf = std::make_shared<rm_filters::ExKalmanFilter>(6, 3, 0, Q, R); // 构造扩展卡尔曼滤波器
        ekf->base_state = rm_filters::MState::const_acc;
        ekf->df_state = rm_filters::MState::df_const_acc;
        ekf->se_df_state = rm_filters::MState::se_df_const_acc;
        ekf->base_sensor = rm_filters::MState::const_acc_sensor;
        ekf->df_sensor = rm_filters::MState::df_const_acc_sensor;
        ekf->se_df_sensor = rm_filters::MState::se_df_const_acc_sensor;
        ekf_filter = ekf;
        Eigen::VectorXd x_0 = Eigen::VectorXd::Zero(6); // 初始状态
        ekf_filter->init(x_0);

        // 枪口与相机之间的偏移
        node_->declare_parameter("x_offset", x_offset);
        node_->get_parameter("x_offset", x_offset);
        node_->declare_parameter("y_offset", y_offset);
        node_->get_parameter("y_offset", y_offset);
        node_->declare_parameter("z_offset", z_offset);
        node_->get_parameter("z_offset", z_offset);

        // 相机到陀螺仪旋转 (根据陀螺仪安装位置进行修改)
        Eigen::Matrix3d rotation;
        rotation << 1, 0, 0, 0, 0, 1, 0, -1, 0;
        cam2imu_static_ = Eigen::Quaterniond(rotation);

        // 弹道解算器
        node_->declare_parameter("shoot_delay", 1.);
        node_->get_parameter("shoot_delay", shoot_delay);
        node_->declare_parameter("shoot_speed_name", "shoot_speed");
        auto shoot_speed_name = node_->get_parameter("shoot_speed_name").as_string();
        rclcpp::QoS shoot_speed_sub_qos_profile(rclcpp::KeepLast(1), best_effort_qos_policy);
        shoot_speed_sub_ = node_->create_subscription<rm_interfaces::msg::ShootSpeed>(
            shoot_speed_name,
            shoot_speed_sub_qos_profile,
            std::bind(&AutoAimAlgo::shoot_speed_cb, this, std::placeholders::_1));
        node_->declare_parameter("shoot_air_param", shoot_air_param);
        node_->get_parameter("shoot_air_param", shoot_air_param);
        // gimbal_solver_ = std::make_shared<rm_trajectory::DatabaseSolver>(30.);
        // gimbal_solver_ = std::make_shared<rm_trajectory::GravityNofrictionSolver>(30.);
        gimbal_solver_ = std::make_shared<rm_trajectory::GravitySolver>(30., shoot_air_param);
        trajectory_transform_tool_ = std::make_shared<rm_trajectory::TransformTool>(gimbal_solver_);
    }

    rm_interfaces::msg::ShootSpeed AutoAimAlgo::get_shoot_speed()
    {
        std::lock_guard<std::mutex> lock(shoot_speed_mutex_);
        return shoot_speed;
    }

    void AutoAimAlgo::set_shoot_speed(const rm_interfaces::msg::ShootSpeed::SharedPtr shoot_speed_temp)
    {
        std::lock_guard<std::mutex> lock(shoot_speed_mutex_);
        shoot_speed = *shoot_speed_temp;
    }

    void AutoAimAlgo::shoot_speed_cb(const rm_interfaces::msg::ShootSpeed::SharedPtr shoot_speed_msg_temp)
    {
        // set_shoot_speed(shoot_speed_msg_temp);
        gimbal_solver_->set_initial_vel(shoot_speed_msg_temp->shoot_speed);
        trajectory_transform_tool_ = std::make_shared<rm_trajectory::TransformTool>(gimbal_solver_);
        initial_vel = shoot_speed_msg_temp->shoot_speed;

#ifdef RM_DEBUG_MODE
        RCLCPP_INFO(node_->get_logger(), "initial_vel: %f", initial_vel);
#endif
    }

    int AutoAimAlgo::process(double time_stamp, cv::Mat &src, Eigen::Quaterniond pose)
    {
        mTarget_yaw = 0;
        mTarget_pitch = 0;
        shoot = false;

        // 自瞄预测参数
        bool same_armor = false, same_id = false;

        // 仅供测试使用，无弹道补偿和滤波跟踪
        using std::sort;
        using std::vector;

        (void)time_stamp;
        (void)pose;

        // 陀螺仪四元数、坐标系转换四元数
        Eigen::Quaterniond imu_q(pose.w(), pose.x(), pose.y(), pose.z());
        imu_q.normalize();
        // Eigen::Quaterniond q = imu_q.inverse();         //    imu_q四元数矩阵的逆矩阵

        auto euler_angles = rm_util::CoordinateTranslation::quat2euler(pose);
        float c_pitch, c_yaw, c_roll;

        c_pitch = rm_util::rad_to_deg(euler_angles(0));
        c_yaw = rm_util::rad_to_deg(euler_angles(2));
        c_roll = rm_util::rad_to_deg(euler_angles(1));
        (void)c_pitch;
        (void)c_yaw;
        (void)c_roll;
#ifdef RM_DEBUG_MODE
        RCLCPP_INFO(node_->get_logger(), "c_pitch: %f, c_yaw: %f, c_roll: %f", c_pitch, c_yaw, c_roll);
#endif
        auto img_center_point = cv::Point2f(src.cols, src.rows / 2);

        // step1.图像识别
        int ret;
        ret = armor_detector_->process(src);
        if (ret != 0)
        {
            // #ifdef RM_DEBUG_MODE
            cv::Mat debugImg = src;
            cv::imshow("target", debugImg);
            cv::waitKey(1);
            // #endif
            return 1; // 无目标
        }
        auto armor_descriptors = armor_detector_->getArmorVector();

#ifdef RM_DEBUG_MODE
        RCLCPP_INFO(
            node_->get_logger(),
            "detected armor size: %lu", armor_descriptors.size());
#endif

        // 装甲板按照图像下方中心点远近排序（物理距离最近）
        sort(armor_descriptors.begin(), armor_descriptors.end(), [img_center_point](const rm_auto_aim::ArmorDescriptor &a, const rm_auto_aim::ArmorDescriptor &b)
             {
            auto dis_a = cv::norm(img_center_point - a.centerPoint);
            auto dis_b = cv::norm(img_center_point - b.centerPoint);
            return dis_a < dis_b; });
        // 默认选择离准星最近的装甲板
        mTarget.armorDescriptor = armor_descriptors.front();

        // step2.坐标解算--装甲板筛选
        Eigen::Vector3d position3d_world;  //世界坐标系坐标
        Eigen::Vector3d position3d_camera; //相机坐标系坐标
        Eigen::Vector3d offset_shoot(x_offset, y_offset, z_offset);
        position3d_world << 0, 0, 0;
        // TODO: 筛选大装甲板
        for (size_t i = 0; i < armor_descriptors.size(); ++i)
        {
            rm_auto_aim::ArmorTarget armor_target;
            armor_target.armorDescriptor = armor_descriptors[i];
            auto label = armor_target.armorDescriptor.label;
            vector<cv::Point2f> points;
            points.push_back(armor_target.armorDescriptor.points[0]);
            points.push_back(armor_target.armorDescriptor.points[1]);
            points.push_back(armor_target.armorDescriptor.points[2]);
            points.push_back(armor_target.armorDescriptor.points[3]);

#ifdef RM_DEBUG_MODE
            RCLCPP_INFO(node_->get_logger(), "armor_label: %d", label);
#endif
            if (this->aim_mode != 0xaa && label == 0)
            {
                continue; //自瞄时label为0则直接返回
            }
            //【图像->相机】PnP解算：目标图像坐标系--->在相机坐标系下的坐标
            if (label == 1 || label == 0)
            {
                // 大装甲板匹配
                mono_loacation_tool_->solve_pnp(points, mBigArmorPoints,
                                                armor_target.postion, armor_target.rotation);
                armor_target.isBigArmor = true;
            }
            else
            {
                // 小装甲板匹配
                mono_loacation_tool_->solve_pnp(points, mSmallArmorPoints,
                                                armor_target.postion, armor_target.rotation);
            }

            // 【相机->世界】目标相机坐标系--->在世界坐标系下的坐标（陀螺仪的世界坐标系）
            Eigen::Vector3d position3d_camera_temp(armor_target.postion.x, armor_target.postion.y, armor_target.postion.z);
            Eigen::Vector3d position3d_world_temp = imu_q * (cam2imu_static_ * position3d_camera_temp + offset_shoot);
            if (i == 0)
            {
                //默认选择距离准星最近装甲板
                mTarget.armorDescriptor = armor_descriptors[i];
                position3d_camera = position3d_camera_temp;
                position3d_world = position3d_world_temp;
            }
            // step3.筛选出当前最重要目标
            // 1）判断是否为同一id的车
            if (last_label == label)
            {
                same_id = true;
            }
            else
            {
                if (time_lost == 0)
                    time_lost = time_stamp;
                same_id = false;
            }
            // 2）判断是否为上次击打同一装甲板(如果是，则直接选中并跳出筛选)
            if (same_id && is_same_armor(last_position3d_world, position3d_world_temp, 50))
            {
                same_armor = true;
                mTarget.armorDescriptor = armor_descriptors[i];
                position3d_camera = position3d_camera_temp;
                position3d_world = position3d_world_temp;
                break;
            }
            else
            {
                same_armor = false;
            }
        }
        if (!same_armor && (time_stamp < (time_lost + 30)))
            position3d_world = last_position3d_world;
        else
            time_lost = 0;

        double target_pitch = rm_util::rad_to_deg(atan2(position3d_world(2, 0), sqrt(pow(position3d_world(0, 0), 2) + pow(position3d_world(1, 0), 2))));
        double target_yaw = rm_util::rad_to_deg(atan2(position3d_world(0, 0), position3d_world(1, 0)));
        double target_distance = position3d_world.norm(); //水平距离
        double target_height = position3d_world(2, 0);

#ifdef RM_DEBUG_MODE
        RCLCPP_INFO(node_->get_logger(),
                    "\n------------滤波前：------------");
        RCLCPP_INFO(node_->get_logger(),
                    "[camera] target_x: %f, target_y: %f, target_z: %f",
                    position3d_camera(0, 0), position3d_camera(1, 0), position3d_camera(2, 0));
        RCLCPP_INFO(node_->get_logger(),
                    "[world] target_x: %f, target_y: %f, target_z: %f",
                    position3d_world(0, 0), position3d_world(1, 0), position3d_world(2, 0));
        RCLCPP_INFO(node_->get_logger(),
                    "real_target_pitch: %f, real_target_yaw: %f",
                    target_pitch, target_yaw);
        RCLCPP_INFO(node_->get_logger(),
                    "target_distance: %f, target_height: %f",
                    target_distance, target_height);
#endif
        double pre_target_pitch = target_pitch;
        double pre_target_yaw = target_yaw;
        double pre_target_distance = target_distance;
        double pre_target_height = target_height;
        (void)pre_target_pitch;
        (void)pre_target_height;

        //实际目标pitch、yaw
        mTarget_yaw = float(target_yaw);
        mTarget_pitch = float(target_pitch);

        //预测击打延时--子弹飞行时间与发弹延迟
        double time = (time_stamp - last_time) * 0.01;
        double predict_time = position3d_world.norm() / (initial_vel * 100) + shoot_delay;

        // step4.滤波预测补偿
        if (same_armor)
        {
            // 【正常滤波模式】 击打同一块装甲板，EKF滤波预测
            Eigen::VectorXd u(6);
            Eigen::VectorXd z_k(3);
            u << 0, 0, 0, 0, 0, 0;
            z_k << position3d_world(0, 0), position3d_world(1, 0), position3d_world(2, 0);
            //预测更新
            time = 0.05;
            ekf_filter->predict(u, time);

            auto x_k = ekf_filter->update(z_k);
            //滤波的装甲板中心点
            f_position3d_world(0, 0) = z_k(0, 0);
            f_position3d_world(1, 0) = z_k(1, 0);
            f_position3d_world(2, 0) = z_k(2, 0);
            //预测的击打点
            filter_position3d_world(0, 0) = z_k(0, 0) + x_k(3, 0) * predict_time;
            filter_position3d_world(1, 0) = z_k(1, 0) + x_k(4, 0) * predict_time;
            filter_position3d_world(2, 0) = z_k(2, 0) + x_k(5, 0) * predict_time;

#ifdef RM_DEBUG_MODE
            RCLCPP_INFO(node_->get_logger(), "【速度】 v_x: %f, v_y: %f, v_z: %f", x_k(3, 0), x_k(4, 0), x_k(5, 0));
#endif
        }
        else
        {
            // 不同ID初始化滤波器
            Eigen::VectorXd x_0(6); // 初始状态
            x_0 << position3d_world(0), position3d_world(1), position3d_world(2),
                0., 0., 0.;
            ekf_filter->init(x_0);
            filter_position3d_world(0, 0) = position3d_world(0, 0);
            filter_position3d_world(1, 0) = position3d_world(1, 0);
            filter_position3d_world(2, 0) = position3d_world(2, 0);
        }
        // TODO: step5.反小陀螺

        // step6.计算pitch、yaw
        /* pitch抬枪补偿 (单位：cm、角度)*/
        Eigen::Vector3d position3d_shoot(filter_position3d_world);
        trajectory_transform_tool_->solve(position3d_shoot, pre_target_pitch, pre_target_yaw); //【枪口弹道偏移】弹道模型拟合
        /* yaw预测补偿 (单位：cm、角度)*/
        pre_target_yaw = rm_util::rad_to_deg(atan2(filter_position3d_world(0, 0), filter_position3d_world(1, 0)));
        pre_target_distance = filter_position3d_world.norm();
        if (abs(pre_target_distance) > 1000 || pre_target_distance <= 0)
            return 3; //防止距离错误
        // 传输最终pitch、yaw数据到上一层（-180 ~ 180）
        if (abs(mTarget_pitch) <= 200 && abs(mTarget_yaw) <= 200)
        {
            mTarget_pitch = pre_target_pitch + pitch_offset;
            mTarget_yaw = -(pre_target_yaw + yaw_offset); //由于与电控的yaw方向相反
        }

        // 【世界->相机】滤波后目标世界坐标系--->在相机坐标系下的坐标
        Eigen::Vector3d pre_camera;
        pre_camera(0, 0) = filter_position3d_world(0, 0);
        pre_camera(1, 0) = filter_position3d_world(1, 0);
        pre_camera(2, 0) = filter_position3d_world(2, 0);
        pre_camera = cam2imu_static_.inverse() * imu_q.inverse() * (pre_camera - offset_shoot);

        // debug输出
#ifdef RM_DEBUG_MODE
        RCLCPP_INFO(node_->get_logger(), "\n------------滤波后：------------");
        RCLCPP_INFO(node_->get_logger(), "target_pitch: %f, target_yaw: %f", target_pitch, target_yaw);
        RCLCPP_INFO(node_->get_logger(), "world_pitch: %f, world_yaw: %f", world_pitch, world_yaw);
        RCLCPP_INFO(node_->get_logger(), "c_pitch: %f, c_yaw: %f", c_pitch, c_yaw);
        RCLCPP_INFO(node_->get_logger(), "pre_target_height: %f, pre_target_pitch: %f, pre_target_yaw: %f", pre_target_height, pre_target_pitch, pre_target_yaw);
        RCLCPP_INFO(node_->get_logger(), "error: %s", transform_tool_->error_message().c_str());
        RCLCPP_INFO(node_->get_logger(), "[shoot] target_x: %f, target_y: %f, target_z: %f", position3d_shoot(0, 0), position3d_shoot(1, 0), position3d_shoot(2, 0));

        RCLCPP_INFO(node_->get_logger(), "[world] target_x: %f, target_y: %f, target_z: %f", filter_position3d_world(0, 0), filter_position3d_world(1, 0), filter_position3d_world(2, 0));
        RCLCPP_INFO(node_->get_logger(), "[camera] target_x: %f, target_y: %f, target_z: %f", pre_camera(0, 0), pre_camera(1, 0), pre_camera(2, 0));

        RCLCPP_INFO(node_->get_logger(), "time: %f", time);
        time_bet = (time_bet + time) / 2;
        RCLCPP_INFO(node_->get_logger(), "平均时间间隔: %f ，fps: %f", time_bet, 1000.0 / time_bet);
        RCLCPP_INFO(node_->get_logger(), "旋转周期：%f", auto_aim_time[0]);
        RCLCPP_INFO(node_->get_logger(), "上一次装甲板面积最大时刻：%f", auto_aim_time[2]);

        cv::Mat debugImg = src;
        cv::putText(debugImg,
                    std::to_string(int(target_distance)),
                    mTarget.armorDescriptor.points[0],
                    cv::FONT_HERSHEY_SIMPLEX, 1,
                    rm_util::red, 3);
        cv::putText(debugImg,
                    std::to_string(mTarget.armorDescriptor.label),
                    mTarget.armorDescriptor.points[1],
                    cv::FONT_HERSHEY_SIMPLEX, 1,
                    rm_util::blue, 3);
        cv::putText(debugImg,
                    std::to_string(int(target_height)),
                    mTarget.armorDescriptor.points[2],
                    cv::FONT_HERSHEY_SIMPLEX, 1,
                    rm_util::green, 3);
        cv::circle(debugImg, mTarget.armorDescriptor.centerPoint, 5, {0, 255, 0}, 3);
        Eigen::Matrix3d F;
        cv::cv2eigen(mono_loacation_tool_->camera_intrinsic_, F);
        Eigen::Vector3d pre_img = F * pre_camera / pre_camera(2, 0);
        cv::circle(debugImg, {int(pre_img(0, 0)), int(pre_img(1, 0))}, 5, {255, 255, 0}, 3);
        cv::imshow("target", debugImg);
        cv::waitKey(1);
#endif
        // 发布滤波后的目标点的位置信息
        geometry_msgs::msg::PointStamped cam_target_point;
        cam_target_point.point.x = filter_position3d_world(0, 0) / 100;
        cam_target_point.point.y = filter_position3d_world(1, 0) / 100;
        cam_target_point.point.z = filter_position3d_world(2, 0) / 100;
        cam_target_point.header.frame_id = "imu_link";
        cam_point_pub_->publish(cam_target_point);

        // 发布目标点的位置信息
        geometry_msgs::msg::PointStamped target_point;
        target_point.point.x = position3d_world(0, 0) / 100;
        target_point.point.y = position3d_world(1, 0) / 100;
        target_point.point.z = position3d_world(2, 0) / 100;
        target_point.header.frame_id = "imu_link";
        point_pub_->publish(target_point);

        last_time = time_stamp;
        last_label = mTarget.armorDescriptor.label;
        last_position3d_world = position3d_world;
        return 0;
    }

    rm_auto_aim::ArmorTarget AutoAimAlgo::getTarget()
    {
        return this->mTarget;
    }

    float AutoAimAlgo::getTargetPitch()
    {
        return this->mTarget_pitch;
    }

    float AutoAimAlgo::getTargetYaw()
    {
        return this->mTarget_yaw;
    }

    void AutoAimAlgo::set_target_color(bool is_red)
    {
        armor_detector_->set_target_color(is_red);
    }

    void AutoAimAlgo::setTrack(bool is_track)
    {
        mIsTrack = is_track;
    }

    void AutoAimAlgo::set_aim_mode(int aim_mode)
    {
        this->aim_mode = aim_mode;
    }

    bool AutoAimAlgo::is_same_armor(Eigen::Vector3d old_position3d, Eigen::Vector3d now_position3d, double distance_threshold)
    {
        double distance = (now_position3d - old_position3d).norm();
        if (distance < distance_threshold)
            return true;
        else
            return false;

#ifdef RM_DEBUG_MODE
        if (1)
        {
            RCLCPP_INFO(node_->get_logger(), "distance_to_last_target: %f", distance);
        }
#endif
    }
}