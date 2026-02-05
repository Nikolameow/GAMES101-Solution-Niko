//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        u = std::max(0.0f, std::min(1.0f, u));
        v = std::max(0.0f, std::min(1.0f, v));
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v) 
    {
        u = std::max(0.0f, std::min(1.0f, u));
        v = std::max(0.0f, std::min(1.0f, v));
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        int bl_u = int(u_img);
        int bl_v = int(v_img);
        int tl_u = bl_u;
        int tl_v = bl_v + 1;
        int br_u = bl_u + 1;
        int br_v = bl_v;
        int tr_u = br_u;
		int tr_v = tl_v;
        auto color_bl = image_data.at<cv::Vec3b>(bl_v, bl_u);
        auto color_tl = image_data.at<cv::Vec3b>(tl_v, tl_u);
        auto color_br = image_data.at<cv::Vec3b>(br_v, br_u);
        auto color_tr = image_data.at<cv::Vec3b>(tr_v, tr_u);
        float s = u_img - bl_u;
        float t = v_img - bl_v;
        auto color = (1 - s) * (1 - t) * color_bl + s * (1 - t) * color_br + (1 - s) * t * color_tl + s * t * color_tr;
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
