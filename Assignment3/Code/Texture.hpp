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
        //限制范围
        u = std::fmin(1, std::fmax(u, 0));
        v = std::fmin(1, std::fmax(v, 0));
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v) 
    {
        //UV转换回屏幕坐标 然后获取临近的四个像素坐标(除了本身其他三个);
        float u1 = int(u * width);   //当前像素坐标
        float v1 = int(v * height);
              
        float u2 = int(u * width + 1);   //右上
        float v2 = int(v * height + 1);
              
        float u3 = int(u * width + 1); //右
        float v3 = int(v * height);
              
        float u4 = int(u * width); //上
        float v4 = int(v * height + 1);
        
        //采样四个像素颜色
        Eigen::Vector3f color1, color2, color3, color4;
        color1 = getColor(u1/width, v1/height);
        color2 = getColor(u2/width, v2/height);
        color3 = getColor(u3/width, v3/height);
        color4 = getColor(u4/width, v4/height);

        Eigen::Vector3f lerp1, lerp2;
        lerp1 = lerp(color1, color2, u * width - u1);
        lerp2 = lerp(color3, color4, u * width - u1);

        return lerp(lerp1, lerp2, v * height - v1);


    }

    Eigen::Vector3f lerp(Eigen::Vector3f a, Eigen::Vector3f b, float weight) 
    {
        return a + weight * (b - a);
    }

};
#endif //RASTERIZER_TEXTURE_H
