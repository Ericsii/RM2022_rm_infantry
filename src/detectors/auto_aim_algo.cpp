#include "rm_auto_aim/auto_aim_algo.hpp"

#include <iostream>
#include <algorithm>
#include <Eigen/Core>

#include "rm_auto_aim/armor_detector_interface.hpp"                                                   

namespace rm_auto_aim
{
    AutoAimAlgo::AutoAimAlgo(rclcpp::Node::SharedPtr node,
                             std::vector<double> camera_intrinsic,
                             std::vector<double> camera_distortion,
                             std::shared_ptr<ArmorDetector> armor_detector) : node_(node), armor_detector_(armor_detector)
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

        Eigen::MatrixXd z_k(rm_filters::Matrix_y, 1);
        z_k(0, 0) = 0;
        z_k(1, 0) = 0;
        z_k(2, 0) = 0;

        ekf_filter = new rm_filters::ExKalman(
                rm_filters::MState::const_acc, rm_filters::MState::df_const_acc, 
                rm_filters::MState::se_df_const_acc, rm_filters::MState::const_acc_sensor, 
                rm_filters::MState::df_const_acc_sensor, rm_filters::MState::se_df_const_acc_sensor);
        ekf_filter->init(z_k);

        // 相机到陀螺仪旋转 (根据陀螺仪安装位置进行修改)
        Eigen::Matrix3d rotation;
        rotation << 0, 0, 1, -1, 0, 0, 0, -1, 0;
        cam2imu_static_ = Eigen::Quaterniond(rotation);
    }

    int AutoAimAlgo::process(double time_stamp, cv::Mat &src, Eigen::Quaterniond pose, int aim_mode)
    {
        mTarget_yaw = 0;
        mTarget_pitch = 0;
        shoot = false;

        // 自瞄预测参数
        bool same_armor = false, same_id = false;
        double initial_vel = 30., shoot_delay = 10;

        // 仅供测试使用，无弹道补偿和滤波跟踪
        using std::vector;
        using std::sort;

        (void) time_stamp;
        (void) pose;

        // 陀螺仪四元数、坐标系转换四元数
        Eigen::Quaterniond imu_q(pose.w(), pose.x(), pose.y(), pose.z());
        imu_q.normalize();
        // Eigen::Quaterniond q = imu_q.inverse();         //    imu_q四元数矩阵的逆矩阵

        auto euler_angles = rm_util::CoordinateTranslation::quat2euler(pose);
        float c_pitch, c_yaw;
        c_pitch = rm_util::rad_to_deg(euler_angles(1));
        c_yaw = rm_util::rad_to_deg(euler_angles(0));
#ifdef RM_DEBUG_MODE
        RCLCPP_INFO(node_->get_logger(), "c_pitch: %f, c_yaw: %f", c_pitch, c_yaw);
#endif
        auto img_center_point = cv::Point2f(src.cols / 2, src.rows / 2);

        // step1.图像识别
        int ret;
        ret = armor_detector_->process(src);
        if (ret != 0)
        {
#ifdef RM_DEBUG_MODE
            cv::Mat debugImg = src;
            cv::imshow("target", debugImg);
#endif
            return 1; // 无目标
        }
        auto armor_descriptors = armor_detector_->getArmorVector();

#ifdef RM_DEBUG_MODE
        RCLCPP_INFO(
            node_->get_logger(),
            "detected armor size: %lu", armor_descriptors.size());
#endif
        
        // 装甲板按照图像中心（准星）远近排序
        sort(armor_descriptors.begin(), armor_descriptors.end(), [img_center_point](const ArmorDescriptor &a, const ArmorDescriptor &b){
            auto dis_a = cv::norm(img_center_point - a.centerPoint);
            auto dis_b = cv::norm(img_center_point - b.centerPoint);
            return dis_a < dis_b;
        });
        // 默认选择离准星最近的装甲板
        mTarget.armorDescriptor = armor_descriptors.front();

        // step2.坐标解算--装甲板筛选
        Eigen::Vector3d position3d_world;   //世界坐标系坐标
        Eigen::Vector3d position3d_camera;  //相机坐标系坐标
        for (size_t i = 0; i < armor_descriptors.size(); ++i)
        {
            ArmorTarget armor_target;
            armor_target.armorDescriptor = armor_descriptors[i];
            auto label = armor_descriptors[i].label;
            vector<cv::Point2f> points;
            points.push_back(armor_descriptors[i].points[0]);
            points.push_back(armor_descriptors[i].points[1]);
            points.push_back(armor_descriptors[i].points[2]);
            points.push_back(armor_descriptors[i].points[3]);

            if (aim_mode == 0x11 && label == 0)
            {
#ifdef RM_DEBUG_MODE
                cv::Mat debugImg = src;
                cv::imshow("target", debugImg);
#endif
                return 2; //自瞄时label为0则直接返回
            }
            //【图像->相机】PnP解算：目标图像坐标系--->在相机坐标系下的坐标
            if (label == 1 || label == 0)
            {
                // 大装甲板匹配
                mono_loacation_tool_->solve_pnp(points, mBigArmorPoints,
                                                armor_target.postion, armor_target.rotation);   
                armor_target.isBigArmor = true;
#ifdef RM_DEBUG_MODE
                RCLCPP_INFO(node_->get_logger(),"big_armor: %d", label);
#endif
            }
            else
            {
                // 小装甲板匹配
                mono_loacation_tool_->solve_pnp(points, mSmallArmorPoints,
                                                armor_target.postion, armor_target.rotation);
            }

            // 【相机->世界】目标相机坐标系--->在世界坐标系下的坐标（陀螺仪的世界坐标系）
            Eigen::Vector3d position3d_camera_temp(armor_target.postion.x, armor_target.postion.y, armor_target.postion.z);
            Eigen::Vector3d position3d_world_temp = imu_q * cam2imu_static_ * position3d_camera_temp;
            if (i==0)
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
                same_id = false;
                // 不同ID初始化滤波器
                Eigen::MatrixXd z_k(rm_filters::Matrix_y, 1);
                z_k(0, 0) = 0;
                z_k(1, 0) = 0;
                z_k(2, 0) = 0;
                ekf_filter->init(z_k);
            }
            // 2）判断是否为上次击打同一装甲板(如果是，则直接选中并跳出筛选)
            if (same_id && is_same_armor(last_position3d_world, position3d_world_temp, 5))
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
                // 不同装甲板初始化滤波器
                Eigen::MatrixXd z_k(rm_filters::Matrix_y, 1);
                z_k(0, 0) = 0;
                z_k(1, 0) = 0;
                z_k(2, 0) = 0;
                ekf_filter->init(z_k);
            }
        }
        
        double target_pitch = rm_util::rad_to_deg(atan2(-position3d_camera(1, 0), sqrt(pow(position3d_camera(0,0),2)+pow(position3d_camera(2,0),2))));
        double target_yaw = rm_util::rad_to_deg(atan2(position3d_camera(0, 0), position3d_camera(2, 0)));
        double target_distance = sqrt(pow(position3d_world(0,0),2)+pow(position3d_world(1,0),2));
        double target_height = position3d_world(2, 0);
        
#ifdef RM_DEBUG_MODE
        RCLCPP_INFO(node_->get_logger(),
                "\n------------滤波前：------------");
        RCLCPP_INFO(node_->get_logger(),
                "[camera] target_x: %f, target_y: %f, target_z: %f", 
                position3d_camera(0,0), position3d_camera(1,0), position3d_camera(2,0));
        RCLCPP_INFO(node_->get_logger(),
                "[world] target_x: %f, target_y: %f, target_z: %f", 
                position3d_world(0,0),position3d_world(1,0),position3d_world(2,0));
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
        //实际目标pitch、yaw
        mTarget_yaw = float(target_yaw);
        mTarget_pitch = float(target_pitch);
        //预测击打延时--子弹飞行时间与发弹延迟
        double time = (time_stamp - last_time);
        if(time > 40)  time /= 5;  //防止因两帧间时间差过大导致的过偏移
        double predict_time = (f_position3d_world.norm()*10) / initial_vel + shoot_delay;

        // step4.滤波预测补偿
        if (same_armor){
            // 【正常滤波模式】 击打同一块装甲板，EKF滤波预测
            Eigen::MatrixXd z_k(rm_filters::Matrix_y, 1);
            z_k(0, 0) = position3d_world(0, 0), z_k(1, 0) = position3d_world(1, 0);
            z_k(2, 0) = position3d_world(2, 0);

            Eigen::MatrixXd U(rm_filters::Matrix_x, rm_filters::Matrix_x);
            U(0, 0) = 0; U(0, 1) = 0; U(0, 5) = 0;
            U(0, 2) = 0; U(0, 3) = 0;  U(0, 4) = 0;
            U(1, 0) = 0; U(1, 1) = 0;	
            U(1, 2) = 0; U(1, 3) = 0;  U(1, 4) = 0; U(1, 5) = 0;
            U(2, 0) = 0; U(2, 1) = 0;  U(2, 5) = 0;
            U(2, 2) = 0; U(2, 3) = 0;  U(2, 4) = 0;
            U(3, 0) = 0; U(3, 1) = 0;  U(3, 5) = 0;
            U(3, 2) = 0; U(3, 3) = 0;  U(3, 4) = 0;
            U(4, 0) = 0; U(4, 1) = 0;  U(4, 5) = 0; 
            U(4, 2) = 0; U(4, 3) = 0;  U(4, 4) = 0;
            U(5, 0) = 0; U(5, 1) = 0;  U(5, 2) = 0; U(5, 3) = 0;
            U(5, 4) = 0; U(5, 5) = 0;

            //预测更新
            ekf_filter->predict(U, time);
            Eigen::MatrixXd x_k = ekf_filter->update(z_k);

            //滤波的装甲板中心点
            f_position3d_world(0, 0) = z_k(0, 0);
            f_position3d_world(1, 0) = z_k(1, 0);
            f_position3d_world(2, 0) = z_k(2, 0);

            //预测的击打点
            filter_position3d_world(0, 0) = z_k(0, 0) + x_k(3, 0)*predict_time;
            filter_position3d_world(1, 0) = z_k(1, 0) + x_k(4, 0)*predict_time;
            filter_position3d_world(2, 0) = z_k(2, 0) + x_k(5, 0)*predict_time;
        
#ifdef RM_DEBUG_MODE
            RCLCPP_INFO(node_->get_logger(),"【速度】 v_x: %f, v_y: %f, v_z: %f", x_k(3, 0), x_k(4, 0), x_k(5, 0));
#endif
        } else {
            filter_position3d_world(0, 0) = position3d_world(0, 0);
            filter_position3d_world(1, 0) = position3d_world(1, 0);
            filter_position3d_world(2, 0) = position3d_world(2, 0);
        }

        // step5.反小陀螺
        end_pre = "normal";
        aim_range = 0;
        double world_pitch = rm_util::rad_to_deg(atan2(position3d_world(2, 0), sqrt(pow(position3d_world(0,0),2)+pow(position3d_world(1,0),2))));
        double world_yaw = rm_util::rad_to_deg(atan2(position3d_world(1, 0), position3d_world(0, 0)));
        if (aim_mode==0x11 && same_id && mTarget.armorDescriptor.label){
            //【反小陀螺模式之跟随】 0x11--同一id(不为 0)
            // 1.通过装甲板面积周期变化得到旋转周期 auto_aim_time[0]
            double armor_area = mTarget.armorDescriptor.armorWidth * mTarget.armorDescriptor.armorHeight;
            double area_rot = (armor_area - last_armor_area)/time;
            if(area_rot*last_armor_area_rot <= 0)
            {
                auto_aim_time[0] = time_stamp - auto_aim_time[2];
                auto_aim_time[2] = time_stamp;
            }
            // 2.切换装甲板
            if(abs(last_yaw-world_yaw)>10 || !same_armor){
                Eigen::MatrixXd z_k(rm_filters::Matrix_y, 1);
                z_k(0, 0) = 0;
                z_k(1, 0) = 0;
                z_k(2, 0) = 0;
                ekf_filter->init(z_k);
                aim_range = abs(last_yaw-world_yaw);
            }
            else{
                // 3.跟踪某一个装甲板
                // 左->右
                if (last_yaw < world_yaw)
                {
                    if((time_stamp-auto_aim_time[2])>auto_aim_time[0]/2 && area_rot<0)  
                        end_pre = "right"; //目标消失，迅速偏移重置
                }
                // 右->左
                else
                {
                    if((time_stamp-auto_aim_time[2])>auto_aim_time[0]/2 && area_rot<0) 
                        end_pre = "left";
                }
            }
            last_armor_area = armor_area;
            last_armor_area_rot = area_rot;
            last_yaw = world_yaw;
        }
        if(aim_mode==0x22 && same_id && mTarget.armorDescriptor.label){
            //【反小陀螺模式之云台不动】 0x22--同一id(不为 0)
            // 1.通过装甲板面积周期变化得到旋转周期 auto_aim_time[0]
            double armor_area = mTarget.armorDescriptor.armorWidth * mTarget.armorDescriptor.armorHeight;
            double area_rot = (armor_area - last_armor_area)/time;
            if(area_rot*last_armor_area_rot <= 0)
            {
                auto_aim_time[0] = time_stamp - auto_aim_time[2];
                auto_aim_time[2] = time_stamp;
            }
            if(time_stamp>=(auto_aim_time[2]+auto_aim_time[0]-predict_time))
                shoot = true;
            last_armor_area = armor_area;
            last_armor_area_rot = area_rot;
        }

        // 【世界->相机】滤波后目标世界坐标系--->在相机坐标系下的坐标
        Eigen::Vector3d pre_camera;
        pre_camera(0, 0) = filter_position3d_world(0, 0);
        pre_camera(1, 0) = filter_position3d_world(1, 0);
        pre_camera(2, 0) = filter_position3d_world(2, 0);
        pre_camera = cam2imu_static_.inverse() * imu_q.inverse() * pre_camera;
        /* yaw预测补偿*/
        pre_target_yaw = rm_util::rad_to_deg(atan2(pre_camera(0, 0), pre_camera(2, 0)));
        pre_target_distance = sqrt(pow(filter_position3d_world(0,0),2)+pow(filter_position3d_world(1,0),2));
        if(abs(pre_target_distance)>1000)
            return 3; //防止距离错误
        /* pitch抬枪补偿 distance(单位：mm) height(单位：m)*/
        pre_target_height = filter_position3d_world(2, 0); //距离枪口高度
        auto offset_pitch = std::make_shared<rm_trajectory::GetPitch>(initial_vel);
        double pitch_temp = offset_pitch->get_pitch(pre_target_distance*10, pre_target_height*10, initial_vel);
        pre_target_pitch = pitch_temp; //此处得到的是绝对姿态pitch，需要减去当前pitch角

        // 传输最终pitch、yaw数据到上一层
        if(abs(mTarget_pitch) <= 30 && abs(mTarget_yaw) <= 30)
        {
            mTarget_pitch = float(pre_target_pitch - c_pitch);
            if(end_pre == "right")
                mTarget_yaw = float(pre_target_yaw) - aim_range;
            else if(end_pre == "left")
                mTarget_yaw = float(pre_target_yaw) + aim_range;
            else
                mTarget_yaw = float(pre_target_yaw);
        }

        //debug输出
#ifdef RM_DEBUG_MODE
        RCLCPP_INFO(node_->get_logger(), "\n------------滤波后：------------");
        RCLCPP_INFO(node_->get_logger(), "[world] target_x: %f, target_y: %f, target_z: %f", filter_position3d_world(0, 0), filter_position3d_world(1, 0), filter_position3d_world(2, 0));
        RCLCPP_INFO(node_->get_logger(), "[camera] target_x: %f, target_y: %f, target_z: %f", pre_camera(0, 0), pre_camera(1, 0), pre_camera(2, 0));
        RCLCPP_INFO(node_->get_logger(), "pre_target_height: %f, pre_target_pitch: %f", pre_target_height, pre_target_pitch);        
        RCLCPP_INFO(node_->get_logger(), "mTarget_pitch: %f, mTarget_yaw: %f", mTarget_pitch, mTarget_yaw);
        RCLCPP_INFO(node_->get_logger(), "time: %f", time);
        time_bet = (time_bet + time) / 2;
        RCLCPP_INFO(node_->get_logger(), "平均时间间隔: %f ，fps: %f", time_bet, 1000.0 / time_bet);

        RCLCPP_INFO(node_->get_logger(), "旋转周期：%f", auto_aim_time[0]);
        RCLCPP_INFO(node_->get_logger(), "上一次装甲板面积最大时刻：%f", auto_aim_time[2]);

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
        cv::circle(debugImg, mTarget.armorDescriptor.centerPoint, 5, {0, 255, 0}, 3);
        cv::cv2eigen(mono_loacation_tool_->camera_intrinsic_, F);
        Eigen::Vector3d pre_img = F * pre_camera / pre_camera(2, 0);
        cv::circle(debugImg, {int(pre_img(0, 0)), int(pre_img(1, 0))}, 5, {255, 255, 0}, 3);
        cv::imshow("target", debugImg);
#endif            
        
        last_time = time_stamp;
        last_label = mTarget.armorDescriptor.label;
        last_position3d_world = position3d_world;
        return 0;
    }

    ArmorTarget AutoAimAlgo::getTarget()
    {
        return this->mTarget;
    }

    void AutoAimAlgo::set_target_color(bool is_red)
    {
        armor_detector_->set_target_color(is_red);
    }

    void AutoAimAlgo::setTrack(bool is_track)
    {
        mIsTrack = is_track;
    }

    bool AutoAimAlgo::is_same_armor(Eigen::Vector3d old_position3d, Eigen::Vector3d now_position3d, double distance_threshold)
    {
        double distance = (now_position3d - old_position3d).norm();
        if(distance < distance_threshold) 
            return true;
        else
            return false;

#ifdef RM_DEBUG_MODE
        if(1){
            RCLCPP_INFO(node_->get_logger(),"distance_to_last_target: %f", distance);
        }
#endif
    }
}