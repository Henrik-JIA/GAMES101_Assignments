#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
//#include <opencv4/opencv2/opencv.h>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

// 罗德里格斯旋转公式——任意旋转
Eigen::Matrix4f get_rotation_matrix(float rotation_angle, Eigen::Vector3f axis)
{
    // 将旋转角度转换为弧度
    float radian = rotation_angle / 180.0 * MY_PI;
    
    // 归一化旋转轴
    axis.normalize();
    
    // 构建罗德里格斯旋转矩阵
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f cross_matrix;
    cross_matrix << 0, -axis(2), axis(1),
                   axis(2), 0, -axis(0),
                   -axis(1), axis(0), 0;
    
    // R = cos(θ)I + (1-cos(θ))nn^T + sin(θ)N
    Eigen::Matrix3f rotation3f = cos(radian) * I +
                              (1 - cos(radian)) * axis * axis.transpose() +
                              sin(radian) * cross_matrix;
    
    // 将3x3旋转矩阵转换为4x4矩阵
    Eigen::Matrix4f rotation4f = Eigen::Matrix4f::Identity();
    rotation4f.block<3,3>(0,0) = rotation3f;
    
    return rotation4f;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity(); //创建一个单位矩阵，即主对角线上的元素都是1，其他位置都是0

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    Eigen::Matrix4f rotate;

    // 方式1：直接创建旋转矩阵
    // float radian = rotation_angle/180.0*MY_PI;
    // rotate << cos(radian), -1*sin(radian), 0, 0,
    //           sin(radian), cos(radian), 0, 0,
    //           0, 0, 1, 0,
    //           0, 0, 0, 1;//单纯实现了关于z轴的旋转矩阵

    // 方式2：使用罗德里格斯公式进行任意旋转
    // 使用 get_rotation_matrix 函数获取绕 Z 轴的旋转矩阵
    // Z 轴向量为 (0, 0, 1)
    rotate = get_rotation_matrix(rotation_angle, Eigen::Vector3f(0, 0, 1));

    model = rotate * model; 
    return model;
}


/*
 * eye_fov 视野的大小
 * aspect_ratio  长宽比？ 猜测是视野的长宽比率
 * zNear 最近处的坐标
 * zFar 最远处的坐标
 */
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function
    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f P2O = Eigen::Matrix4f::Identity();//将透视投影转换为正交投影的矩阵
    P2O<<zNear, 0, 0, 0,
         0, zNear, 0, 0,
         0, 0, zNear+zFar,(-1)*zFar*zNear,
         0, 0, 1, 0;// 进行透视投影转化为正交投影的矩阵
    float halfEyeAngelRadian = eye_fov/2.0/180.0*MY_PI;
    float t = zNear*std::tan(halfEyeAngelRadian);//top y轴的最高点
    float r=t*aspect_ratio;//right x轴的最大值
    float l=(-1)*r;//left x轴最小值
    float b=(-1)*t;//bottom y轴的最大值
    Eigen::Matrix4f ortho1=Eigen::Matrix4f::Identity();
    ortho1<<2/(r-l),0,0,0,
        0,2/(t-b),0,0,
        0,0,2/(zNear-zFar),0,
        0,0,0,1;//进行一定的缩放使之成为一个标准的长度为2的正方体
    Eigen::Matrix4f ortho2 = Eigen::Matrix4f::Identity();
    ortho2<<1,0,0,(-1)*(r+l)/2,
        0,1,0,(-1)*(t+b)/2,
        0,0,1,(-1)*(zNear+zFar)/2,
        0,0,0,1;// 把一个长方体的中心移动到原点
    Eigen::Matrix4f Matrix_ortho = ortho1 * ortho2;
    projection = Matrix_ortho * P2O;
    return projection;
}


int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    // 检测 ESC 键的按下。在 ASCII 码中，27 就是 ESC 键的编码值。
    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        // CV_32FC3指定图像的数据类型和通道数，32F表示使用32位浮点数，C3表示有3个通道。
        // r.frame_buffer().data()使用光栅化器(rasterizer)的帧缓冲区数据作为图像数据源
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        // CV_8UC3目标图像的数据类型，8U表示使用8位无符号整数，C3表示3个通道（RGB），1.0f缩放因子，用于在转换过程中对像素值进行缩放。
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image); // 在名为"image"的窗口中显示当前帧
        key = cv::waitKey(10); // 等待10毫秒并获取键盘输入，这里的10毫秒延迟可以控制动画的帧率（约100 FPS）。

        std::cout << "frame count: " << frame_count++ << '\n';// 打印并递增帧计数器

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
