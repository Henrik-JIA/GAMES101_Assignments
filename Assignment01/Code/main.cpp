#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
//#include <opencv4/opencv2/opencv.h>

constexpr double MY_PI = 3.1415926;

// 获取视图矩阵，用于设置视点位置（传入观察点位置）
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    // 将观察点位置移动到原点
    // 由于观察点位置在(0,0,5)，所以需要将(0,0,5)平移到原点(0,0,0)
    translate << 1, 0, 0, -eye_pos[0], 
                 0, 1, 0, -eye_pos[1], 
                 0, 0, 1, -eye_pos[2], 
                 0, 0, 0, 1;

    view = translate * view;

    return view;
}

// 罗德里格斯旋转公式——任意旋转（传入旋转角度和旋转轴）
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

// 获取模型矩阵，用于旋转三角形（传入旋转角度）
// 模型矩阵是直接对模型进行操作的。
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity(); //创建一个单位矩阵，即主对角线上的元素都是1，其他位置都是0

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    Eigen::Matrix4f rotate;

    // 方式1：直接创建旋转矩阵，单纯实现了关于z轴的旋转矩阵
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

// 获取投影矩阵，用于设置投影（传入视野大小、长宽比、最近处坐标、最远处坐标）
/* 
 * eye_fov 视野的大小，单位为度
 * aspect_ratio  长宽比
 * zNear 最近处的坐标，这个面离观察点的距离，相机看向-z轴，那么这个面离观察点的距离就是0.1，坐标就是(0,0,-0.1)
 * zFar 最远处的坐标，这个面离观察点的距离，相机看向-z轴，那么这个面离观察点的距离就是50，坐标就是(0,0,-50)
 */
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // 注意：这里zNear和zFar是相机看向-z轴的，zNear和zFar传入的是距离值，如果是坐标就都是负数。

    // 透视投影矩阵
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // 1.将透视投影转换为正交投影的矩阵，压缩矩阵，使得视锥体变为长方体
    Eigen::Matrix4f P2O = Eigen::Matrix4f::Identity();//将透视投影转换为正交投影的矩阵
    P2O<<zNear, 0, 0, 0,
         0, zNear, 0, 0,
         0, 0, zNear+zFar,(-1)*zFar*zNear,
         0, 0, 1, 0;// 进行透视投影转化为正交投影的矩阵

    // 2.从视场角和宽高比计算近平面参数，计算正交投影的参数
    float halfEyeAngelRadian = eye_fov/2.0/180.0*MY_PI; // 将视野大小转换为弧度
    // top与 bottom 是对称的，所以计算一个即可
    float t = zNear*std::tan(halfEyeAngelRadian);//top y轴的最高点
    float b=(-1)*t;//bottom y轴的最大值
    // right与 left 是对称的，所以计算一个即可
    float r=t*aspect_ratio;//right x轴的最大值
    float l=(-1)*r;//left x轴最小值

    // 3.正交投影矩阵
    // 将正方体移动到原点
    Eigen::Matrix4f ortho1 = Eigen::Matrix4f::Identity();
    ortho1<<1,0,0,(-1)*(r+l)/2,
            0,1,0,(-1)*(t+b)/2,
            0,0,1,(-1)*(zNear+zFar)/2,
            0,0,0,1;// 把一个长方体的中心移动到原点
    // 进行一定的缩放使之成为一个标准的长度为2的正方体
    Eigen::Matrix4f ortho2=Eigen::Matrix4f::Identity();
    ortho2<<2/(r-l),0,0,0,
            0,2/(t-b),0,0,
            0,0,2/(zNear-zFar),0,
            0,0,0,1;//进行一定的缩放使之成为一个标准的长度为2的正方体
    // 变换步骤：先将中心平移到原点，然后进行缩放（长/宽/高缩放到2）
    Eigen::Matrix4f Matrix_ortho = ortho2 * ortho1;

    // 4.先将视锥体"压缩"成长方体，然后进行正交投影
    // 将透视投影转换为正交投影的矩阵与正交投影的矩阵相乘，得到投影矩阵
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

    // 程序入口
    // 创建光栅化器对象，指定窗口的宽度和高度
    rst::rasterizer r(700, 700);

    // 设置视点位置
    Eigen::Vector3f eye_pos = {0, 0, 5};

    // 设置三角形的顶点位置
    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    // 对顶点进行分组，每三个顶点为一个三角形，ind中的每个元素对应一个三角形。
    std::vector<Eigen::Vector3i> ind{{0, 1, 2},};

    // 加载顶点位置和索引
    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    // 设置键盘输入
    int key = 0;
    // 设置帧计数器
    int frame_count = 0;

    // 如果命令行模式，则直接绘制三角形并保存图像
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
        // 清除上一帧的颜色和深度缓冲区
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // mvp矩阵
        // 设置模型矩阵
        r.set_model(get_model_matrix(angle));
        // 设置视图矩阵
        r.set_view(get_view_matrix(eye_pos));
        // 设置投影矩阵，这里相机是看向-z轴的，所以近平面离观察点的距离是0.1，坐标就是(0,0,-0.1)，远平面离观察点的距离是50，坐标就是(0,0,-50)
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        // 绘制三角形，pos_id是顶点位置的索引，ind_id是三角形的索引，rst::Primitive::Triangle是三角形的类型
        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        // CV_32FC3指定图像的数据类型和通道数，32F表示使用32位浮点数，C3表示有3个通道。
        // r.frame_buffer().data()使用光栅化器(rasterizer)的帧缓冲区数据作为图像数据源
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        // CV_8UC3目标图像的数据类型，8U表示使用8位无符号整数，C3表示3个通道（RGB），1.0f缩放因子，用于在转换过程中对像素值进行缩放。
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image); // 在名为"image"的窗口中显示当前帧
        key = cv::waitKey(10); // 等待10毫秒并获取键盘输入，这里的10毫秒延迟可以控制动画的帧率（约100 FPS）。

        std::cout << "frame count: " << frame_count++ << '\n';// 打印并递增帧计数器

        // 旋转三角形，绕z轴旋转
        // 直接作用于模型本身
        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }

        // 相机的平移变换
        //后面的 w/s/q/e/z/c 控制的是相机(eye_pos)的移动
        // 前后移动（控制z轴）
        if (key == 'w') {
            eye_pos[2] += 1;
        }
        else if (key == 's') {
            eye_pos[2] -= 1;
        }
        // 左右移动（控制x轴）
        else if (key == 'q') {
            eye_pos[0] -= 1;  // 向左移动，x值减小
        }
        else if (key == 'e') {
            eye_pos[0] += 1;  // 向右移动，x值增加
        }
        // 上下移动（控制y轴）
        else if (key == 'z') {
            //这里存在一个bug，当相机移动到y轴的负值时，会报错。
            eye_pos[1] -= 1;  // 向下移动，y值减小
            std::cout << "Camera position: (" << eye_pos[0] << ", " << eye_pos[1] << ", " << eye_pos[2] << ")" << std::endl;
        }
        else if (key == 'c') {
            eye_pos[1] += 1;  // 向上移动，y值增加
        }
		
    }

    return 0;
}
