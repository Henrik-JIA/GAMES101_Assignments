#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

// 控制点列表
// 特别提示，控制点在点击时，特别注意点击顺序。
std::vector<cv::Point2f> control_points;

// 贝塞尔曲线的控制点数量，默认为4，即三阶贝塞尔曲线
#define NUM 4

// 鼠标回调函数，当鼠标点击时，调用mouse_handler函数
void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    // 如果鼠标左键点击，并且控制点数量小于4，则将点击的点添加到控制点列表中
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < NUM) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        // 将点击的点添加到控制点列表中
        control_points.emplace_back(x, y);
    }     
}

// 直接默认三阶贝塞尔曲线
// 这个并不具备普适性，只能用于三阶贝塞尔曲线
void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    // 只有4个控制点，p_0, p_1, p_2, p_3
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    // 从0到1，步长为0.001
    // 通过t的变化，将所有贝塞尔曲线上的点绘制到窗口中，通过离散的点来近似贝塞尔曲线
    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        // 计算贝塞尔曲线上的点
        // 三次多项式表达式：B(t) = (1 - t)^3 * p_0 + 3 * t * (1 - t)^2 * p_1 + 3 * t^2 * (1 - t) * p_2 + t^3 * p_3
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;
        // 将贝塞尔曲线上的点绘制到窗口中，opencv是BGR颜色空间，第三位是红色
        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

// 递归
cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    // 递归出口，此时为贝塞尔曲线上的点。
    if (control_points.size() == 1)
    {
        return control_points[0];
    }

    // 新建一个数组，储存新的控制点，数量每次递减1，减少到1个控制点时，就可以求得贝塞尔曲线上的点。
    std::vector<cv::Point2f> lerp_points;
    // 遍历原控制点数组，通过最基本的公式，求得新的控制点
    for (size_t i = 0; i < control_points.size() - 1; i++)
    {
        // 基本公式：(1 - t) * p0 + t * p1;
        lerp_points.push_back( (1 - t) * control_points[i] + t * control_points[i + 1] );
    }

    // 递归求解
    return recursive_bezier(lerp_points, t);

}

// 使用递归算法，可以用于任意阶的贝塞尔曲线。
void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        // 对于每一个t，求它对应的贝塞尔曲线上的点，这里是通过最基本的公式(1-t)*a+t*b递归求得
        // 实际上也可以直接用 n阶展开式（伯恩斯坦多项式），直接求得。
        auto point = recursive_bezier(control_points, t);
        // 将贝塞尔曲线上的点绘制到窗口中，opencv是BGR颜色空间，第三位是红色
        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }

}

// 伯恩斯坦多项式
void bezier_Bernstein(const std::vector<cv::Point2f> &control_points, cv::Mat &window)
{
    for (double t = 0.0; t <= 1.0; t += 0.001)
    {
        cv::Point2f point(0, 0);
        int n = control_points.size() - 1;  // 贝塞尔曲线的阶数

        for (int i = 0; i <= n; i++)  // 修改循环范围，包含n
        {
            // 计算二项式系数 C(n,i)
            float coefficient = 1.0f;
            for(int j = 0; j < i; j++)
            {
                coefficient *= (float)(n - j) / (j + 1);  // 确保浮点数除法
            }
                
            // 计算伯恩斯坦基函数 B(n,i) = C(n,i) * t^i * (1-t)^(n-i)
            float bernstein = coefficient * std::pow(t, i) * std::pow(1 - t, n - i);
            
            // 累加每个控制点的贡献
            point += bernstein * control_points[i];
        }
    
        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

int main() 
{
    // 创建一个700x700的窗口，并设置为RGB颜色空间
    cv::Mat window = cv::Mat(500, 500, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    // 设置鼠标回调函数，当鼠标点击时，调用mouse_handler函数
    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    // key为27是ESC键，当按下ESC键时，退出循环
    int key = -1;
    while (key != 27) 
    {
        // 遍历控制点列表，绘制控制点
        for (auto &point : control_points) 
        {
            // 绘制控制点，颜色为白色，半径为3
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        // 如果控制点数量为4，则绘制贝塞尔曲线
        if (control_points.size() == NUM) 
        {
            // 绘制贝塞尔曲线：
            // 1、naive_bezier不具备普适性，只能用于三阶贝塞尔曲线
            // naive_bezier(control_points, window);

            // 2、使用递归算法，可以用于任意阶的贝塞尔曲线
            // bezier(control_points, window);

            // 3、使用伯恩斯坦多项式，直接求解任意阶的贝塞尔曲线
            bezier_Bernstein(control_points, window);

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
