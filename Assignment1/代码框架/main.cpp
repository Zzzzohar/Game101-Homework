#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;
/// <summary>
/// MVP����
/// </summary>
/// <param name="eye_pos"></param>
/// <returns></returns>
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos) //V����
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)//M����
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    Eigen::Matrix4f translate;
    float radian = rotation_angle / 180 * MY_PI;
    translate << cos(radian), -sin(radian), 0, 0,
        sin(radian), cos(radian), 0, 0,
        0, 0, 1,0,
        0, 0, 0, 1;

    model = translate * model;

    return model;
}
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
    float zNear, float zFar)      //P����
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    eye_fov = (eye_fov / 180.0) * MY_PI;

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    Eigen::Matrix4f ProjToOrth;
    ProjToOrth <<
        zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zNear + zFar, -zNear * zFar,
        0, 0, 1, 0;

    float nearClipPlaneHeight = -2 * zNear * tan(eye_fov / 2);
    float nearClipPlaneWeigth = aspect_ratio * nearClipPlaneHeight;
    Eigen::Matrix4f Orth;
    Orth <<
        2 / nearClipPlaneWeigth, 0, 0, 0,
        0, 2 / nearClipPlaneHeight, 0, 0,
        0, 0, 2 / (zFar - zNear), 0,
        0, 0, 0, 1;



    projection = Orth * ProjToOrth * projection;
    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {//Rodrigues' Rotation Formula
    double radian = angle / 180 * MY_PI;
    Eigen::Matrix4f I;
    Eigen::Matrix4f N;
    Eigen::Matrix4f Rodrigues;

    Eigen::Vector4f n;
    Eigen::RowVector4f nT;

    //vector �������� ��������
    //      {axis.x}
    //n  =  {axis.y}
    //      {axis.z}
    //ת�� nT = {axis.x,axis.y,axis.z}

    n << axis.x(), axis.y(), axis.z(), 0;
    nT << axis.x(), axis.y(), axis.z(), 0;

    I << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    N << 0, -axis.z(), axis.y(), 0,
        axis.z(), 0, -axis.x(), 0,
        -axis.y(), axis.x(), 0, 0,
        0, 0, 0, 1;

    Rodrigues = cos(radian) * I + (1 - cos(radian)) * n * nT + sin(radian) * N;
    Rodrigues(3, 3) = 1;
    return Rodrigues;
}


int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    int AxisFlag = 0;
    std::cout <<"ʹ��Ĭ��Z����ת/�Զ�������ת (0/1)" << std::endl;
    std::cin >> AxisFlag;

    Eigen::Vector3f axis;
    if (AxisFlag == 1) {
        std::cout << "�Զ�����ת��" << std::endl;
        std::cin >> axis.x() >> axis.y()>> axis.z();
    }

    float rotationSpeed = 1.0;
    std::cout << "������ת�ٶ�" << std::endl;
    std::cin >> rotationSpeed;
    





    //�����źŴ���3��ʱ��
    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) { //�����ź�==4��ʱ��
            filename = std::string(argv[3]);//��ȡ�ļ���
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700); // ���
    /// <summary>
    /// ��ʼ������
    /// </summary>
    /// <param name="argc"></param>
    /// <param name="argv"></param>
    /// <returns></returns>
    Eigen::Vector3f eye_pos = {0, 0, 5}; 

    std::vector<Eigen::Vector3f> pos{{2, 0, 0}, {0, 2, 0}, {-2, 0, 0}}; 

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        //��ʼ��RMVP
        //r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));
        //����Z����ת ��ôM���������ת
        if (AxisFlag == 0) {
            r.set_model(get_model_matrix(angle));
            r.set_rodrigues(Eigen::Matrix4f::Identity());
        }
        else {//�Զ�����ת ��ôR���������ת  M��������ǵ�λ���� ���� �Ƚ�������
            r.set_model(Eigen::Matrix4f::Identity());
            r.set_rodrigues(get_rotation(axis, angle));
        }
        
        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        //���buffer
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        //��ʼ��RMVP
        //r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));
        //����Z����ת ��ôM���������ת
        if (AxisFlag == 0) {
            r.set_model(get_model_matrix(angle));
            r.set_rodrigues(Eigen::Matrix4f::Identity());
        }
        else {//�Զ�����ת ��ôR���������ת  M��������ǵ�λ���� ���� �Ƚ�������
            r.set_model(Eigen::Matrix4f::Identity());
            r.set_rodrigues(get_rotation(axis, angle));
        }
        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        //std::cout << "frame count: " << frame_count++ << '\n';
        //�����ж�
        if (key == 'a') {
            angle += rotationSpeed;
        }
        else if (key == 'd') {
            angle -= rotationSpeed;
        }
    }

    return 0;
}
