//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;
    cv::Mat spot_texture_data;  // 添加新的成员变量存储spot_texture.png纹理

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;

        // 在构造函数中读取spot_texture.png纹理
        std::string spot_texture_path = "../models/spot/spot_texture.png";
        spot_texture_data = cv::imread(spot_texture_path);
        if (!spot_texture_data.empty()) {
            cv::cvtColor(spot_texture_data, spot_texture_data, cv::COLOR_RGB2BGR);
        }
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    // 添加新的采样函数，使用默认纹理路径，获取纹理颜色
    Eigen::Vector3f getSpotTextureColor(float u, float v)
    {
        if (spot_texture_data.empty()) {
            return Eigen::Vector3f(0, 0, 0);
        }

        // 确保 u,v 在 [0,1] 范围内
        u = std::clamp(u, 0.0f, 1.0f);
        v = std::clamp(v, 0.0f, 1.0f);
        
        // 计算图像坐标
        auto u_img = static_cast<int>(u * (spot_texture_data.cols - 1));
        auto v_img = static_cast<int>((1 - v) * (spot_texture_data.rows - 1));
        
        // 获取像素颜色
        auto color = spot_texture_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
