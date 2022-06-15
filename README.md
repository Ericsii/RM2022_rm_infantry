#rm_infantry 步兵自瞄部分
TODO:反小陀螺算法待开发
        end_pre = "normal";
        aim_range = 0;
        double world_pitch = rm_util::rad_to_deg(atan2(position3d_world(2, 0), sqrt(pow(position3d_world(0, 0), 2) + pow(position3d_world(1, 0), 2))));
        double world_yaw = rm_util::rad_to_deg(atan2(position3d_world(0, 0), position3d_world(1, 0)));
        (void)world_pitch;
        if (aim_mode == 0x11 && same_id && mTarget.armorDescriptor.label)
        {
            //【反小陀螺模式之跟随】 0x11--同一id(不为 0)
            // 1.通过装甲板面积周期变化得到旋转周期 auto_aim_time[0]
            double armor_area = mTarget.armorDescriptor.armorWidth * mTarget.armorDescriptor.armorHeight;
            double area_rot = (armor_area - last_armor_area) / time;
            if (area_rot * last_armor_area_rot <= 0)
            {
                auto_aim_time[0] = time_stamp - auto_aim_time[2];
                auto_aim_time[2] = time_stamp;
            }
            // 2.切换装甲板
            if (abs(last_yaw - world_yaw) > 10 || !same_armor)
            {
                Eigen::VectorXd x_0 = Eigen::VectorXd::Zero(6); // 初始状态
                ekf_filter->init(x_0);
                aim_range = abs(last_yaw - world_yaw);
            }
            else
            {
                // 3.跟踪某一个装甲板
                // 左->右
                if (last_yaw < world_yaw)
                {
                    if ((time_stamp - auto_aim_time[2]) > auto_aim_time[0] / 2 && area_rot < 0)
                        end_pre = "right"; //目标消失，迅速偏移重置
                }
                // 右->左
                else
                {
                    if ((time_stamp - auto_aim_time[2]) > auto_aim_time[0] / 2 && area_rot < 0)
                        end_pre = "left";
                }
            }
            last_armor_area = armor_area;
            last_armor_area_rot = area_rot;
            last_yaw = world_yaw;
        }
        if (aim_mode == 0x22 && same_id && mTarget.armorDescriptor.label)
        {
            //【反小陀螺模式之云台不动】 0x22--同一id(不为 0)
            // 1.通过装甲板面积周期变化得到旋转周期 auto_aim_time[0]
            double armor_area = mTarget.armorDescriptor.armorWidth * mTarget.armorDescriptor.armorHeight;
            double area_rot = (armor_area - last_armor_area) / time;
            if (area_rot * last_armor_area_rot <= 0)
            {
                auto_aim_time[0] = time_stamp - auto_aim_time[2];
                auto_aim_time[2] = time_stamp;
            }
            if (time_stamp >= (auto_aim_time[2] + auto_aim_time[0] - predict_time))
                shoot = true;

            last_armor_area = armor_area;
            last_armor_area_rot = area_rot;
        }