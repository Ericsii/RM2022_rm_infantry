#ifndef RM_AUTO_AIM__ARMOR_DETECTOR_SVM_HPP_
#define RM_AUTO_AIM__ARMOR_DETECTOR_SVM_HPP_

#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rm_auto_aim/armor_detector_interface.hpp"

namespace rm_auto_aim
{
    class ArmorDetectorSVM : public ArmorDetector
    {
    public:
        ArmorDetectorSVM() = delete;
        ~ArmorDetectorSVM();

        /**
         * @brief Construct a new Armor Detector SVM object
         * 
         * @param node rclcpp 节点
         * @param armor_is_red 目标装甲颜色
         * @param xml_path 模型文件(XML)保存路径
         */
        ArmorDetectorSVM(rclcpp::Node::SharedPtr node, bool armor_is_red = true, const std::string &xml_path = "");

        /**
         * @brief 处理一帧图像
         * 
         * @param src 输入图像
         * @return int 0 正常; 1 灯条数目不够; 2 无装甲板
         */
        int process(cv::Mat &src);

        /**
         * @brief Get the Armor Vector object
         * 
         * @return std::vector<ArmorDescriptor>& 保存的装甲板对象列表
         */
        std::vector<ArmorDescriptor> &getArmorVector();

        /**
         * @brief Set the target color object
         * 
         * @param is_red 
         */
        void set_target_color(bool is_red);

    private:
        /**
         * @brief 预处理图片转换为二值图
         * 
         * @param src 输入图像
         * @param dst 输出图像
         * @return int 0
         */
        int preImg(cv::Mat &src, cv::Mat &dst);

        /**
         * @brief Get the Light Descriptor object
         * 灯条参数计算
         * 
         * @param r 匹配到的选转矩形
         * @param light 输出的lightDescriptor
         * @return int 0 正常; 1 区域大小不满足; 2 宽高比不正确; 3 灯条倾斜度不符合
         */
        int getLightDescriptor(cv::RotatedRect r, LightDescriptor &light);

        /**
         * @brief 灯条匹配
         * 
         * @param l1 灯条1
         * @param l2 灯条2
         * @param armor 返回的ArmorDescriptor
         * @return int 0 正常; 1 平行太大误差; 2 宽高比不符合; 3 装甲板正度（矩形内角=90）不符合; 4 装甲板倾斜不符合
         */
        int lightsMatch(LightDescriptor &l1, LightDescriptor &l2, ArmorDescriptor &armor);

        /**
         * @brief 灯条高度补偿
         * 将两灯条高度尽量补偿一致
         * 
         * @param light 灯条对象指针
         * @param height 补偿的目标高度
         */
        void lightHeightLong(LightDescriptor *light, float height);

        /**
         * @brief Get the Armor Number object
         * 
         * @param src 输入 RGB 图像
         * @param armor 待测装甲板
         * @return int 0 正常; -1 未检测到装甲板
         */
        int getArmorNumber(cv::Mat &src, ArmorDescriptor &armor); // 计算装甲编号

    private:
        // 装甲板信息
        cv::Ptr<cv::ml::SVM> svm_;            // SVM分类器
        std::vector<LightDescriptor> lights_; // 灯条列表
        std::vector<ArmorDescriptor> armors_; // 装甲板列表
        cv::Point2f perspective_targets_[4];  // 装甲板投影变换目标点

        // 图像信息
        cv::Mat binImg_;                               // 二值图
        cv::Mat grayImg_;                              // 灰度图
        cv::Mat transformImg_;                         // 投影变换后图片
        std::vector<std::vector<cv::Point>> contours_; // 图像轮廓
        std::vector<cv::Vec4i> hierarchy_;             // 轮廓结构信息

        //
        cv::HOGDescriptor m_hog;
    };
} // namespace rm_auto_aim

#endif // ARMOR_DETECTOR_SVM_HPP_