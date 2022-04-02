#ifndef RM_AUTO_AIM__ARMOR_DETECTOR_HPP
#define RM_AUTO_AIM__ARMOR_DETECTOR_HPP

#include <vector>

#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
namespace rm_auto_aim
{
    typedef struct _LightDescriptor
    {
        int id;
        cv::RotatedRect box;             // 拟合椭圆
        cv::Point2f led_top, led_bottom; // 灯条上下点坐标
        cv::Point2f led_center;          // 灯条中心点
        // light info
        float lightHeight;  // 灯条高度
        float lightWidth;   // 灯条宽度
        float lightRatioWH; // 灯条宽高比 W/H
        float lightArea;    // 灯条面积
        float lightAngle;   // 灯条倾斜角度
    } LightDescriptor;

    typedef struct _ArmorDescriptor
    {
        cv::Point2f points[4]; // 灯条角点(左上/左下/右下/右上)
        // info
        LightDescriptor *lightR, *lightL; // 装甲板两灯条
        cv::Point2f centerPoint;          // 装甲板中心点
        float armorHeight;                // 灯条高度
        float armorWidth;                 // 灯条宽度
        float armorRatioWH;               // 装甲板宽高比 W/H
        float parallelErr;                // 灯条平行角度误差
        float innerAngleErr;              // 装甲板内角误差
        float horizonLineAngle;           // 装甲板水平角度
        int label;                        // 装甲板编号
        // 0 哨兵; 1-5 编号; 6 前哨站; 7 基地; 8 基地大装甲. 其余未检测到装甲板
    } ArmorDescriptor;

    typedef struct _ArmorTarget
    {
        ArmorDescriptor armorDescriptor;
        cv::Point3f postion; 
        cv::Mat rotation;
        bool isBigArmor = false;
        float distance = 0;    // 单位cm
        float move2dCast = 0;  // 距离中心点的代价
        float track2dCast = 0; // 与上次目标的2D距离代价
        float track3dCast = 0; // 与上次目标的3D距离代价
    } ArmorTarget;

    class ArmorDetector
    {
    public:
        ArmorDetector(rclcpp::Node::SharedPtr node,
                      bool target_color_red): node_(node), target_color_red_(target_color_red) {};

        void set_target_color(bool is_red)
        {
            target_color_red_ = is_red;
        }

        /**
         * @brief 处理一帧图像
         * 
         * @param src 输入图像
         * @return int 0 正常; 1 灯条数目不够; 2 无装甲板
         */
        virtual int process(cv::Mat &src) = 0;

        /**
         * @brief Get the Armor Vector object
         * 
         * @return std::vector<ArmorDescriptor>& 保存的装甲板对象列表
         */
        virtual std::vector<ArmorDescriptor> &getArmorVector() = 0;

    protected:
        rclcpp::Node::SharedPtr node_; // rclcpp 节点
        bool target_color_red_;        // 装甲板目标颜色
    };
} // namespace rm_auto_aim

#endif // RM_AUTO_AIM__ARMOR_DETECTOR_HPP