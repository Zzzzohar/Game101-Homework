#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    //ping pong buffer
    std::vector<cv::Point2f> cur_control_points = control_points, buf_control_points;
    for (int i = 0; i < control_points.size() - 1 ; i++) {
        
        cv::Point2f temp_point = (1 - t) * control_points[i] + t * control_points[i + 1];
        //����buffer��
        buf_control_points.push_back(temp_point);

    }

    while (buf_control_points.size() > 1) {
        //buf ���� 
        cur_control_points = buf_control_points;
        buf_control_points.clear();
        for (int i = 0; i < cur_control_points.size() - 1; i++) {

            cv::Point2f temp_point = (1 - t) * cur_control_points[i] + t * cur_control_points[i + 1];
            //����buffer��
            buf_control_points.push_back(temp_point);

        }
    }


    return buf_control_points[0];

}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for (float t = 0; t <= 1; t = t + 0.001) {
        auto point = recursive_bezier(control_points, t);
        //point��������ɫ��  �Ź������ľ�������ɫ��ͬʱ���ݾ���ƽ������������
        
        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {
                //�������Ȩ��  
                float dis = std::sqrt(std::pow(point.y - int(point.y + j) - 0.5, 2) + std::pow(point.x - int(point.x + i) - 0.5, 2));//����
                float weight =  1 - dis * std::sqrt(2) / 3;
                float color = weight * 255;

                //������point �Ź���Ҳ���ƶ� ��ֹ�µ�point ƽ����ɫ�����˴�Ȩ�صĻ���֮ǰ��point ������Ҫ�ж��Ƿ������  
                color = std::fmax(window.at<cv::Vec3b>(point.y + j, point.x + i)[1], color);
                
                //���Ź�����ɫ
                window.at<cv::Vec3b>(point.y + j, point.x + i)[1] = color;
            }
        }

    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            //naive_bezier(control_points, window);
               bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
