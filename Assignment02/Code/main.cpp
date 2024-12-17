// clang-format off
#include <iostream>
#include <opencv2/opencv.hpp>
#include "rasterizer.hpp"
#include "global.hpp"
#include "Triangle.hpp"

constexpr double MY_PI = 3.1415926;

// 获取视图矩阵
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    // 将观察点位置移动到原点
    // 由于观察点位置在(0,0,5)，所以需要将(0,0,5)平移到原点(0,0,0)
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

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

// 获取模型矩阵
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f rotate = get_rotation_matrix(rotation_angle, Eigen::Vector3f(0, 0, 1));

    model = rotate * model; // 将旋转矩阵应用到模型矩阵
    return model;
}

// 获取投影矩阵
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // TODO: Copy-paste your implementation from the previous assignment.
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

    if (argc == 2)
    {
        command_line = true;
        filename = std::string(argv[1]);
    }

    // 创建光栅化器，700x700的图像
    rst::rasterizer r(700, 700);

    // 相机位置
    Eigen::Vector3f eye_pos = {0,0,5};

    // 三角形顶点坐标
    std::vector<Eigen::Vector3f> pos
            {
                    {2, 0, -2},
                    {0, 2, -2},
                    {-2, 0, -2},
                    {3.5, -1, -5},
                    {2.5, 1.5, -5},
                    {-1, 0.5, -5}
            };

    // 三角形顶点颜色
    std::vector<Eigen::Vector3f> cols
            {
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0}
            };

    // 对顶点坐标进行分组，每三个顶点组成一个三角形
    std::vector<Eigen::Vector3i> ind
            {
                    {0, 1, 2},
                    {3, 4, 5}
            };

    // 加载顶点坐标到光栅化器
    auto pos_id = r.load_positions(pos);
    // 加载三角形顶点颜色到光栅化器
    auto col_id = r.load_colors(cols);
    // 加载三角形顶点索引到光栅化器
    auto ind_id = r.load_indices(ind);


    int key = 0; // 键盘输入
    int frame_count = 0; // 帧计数器

    // 如果命令行模式，则直接绘制三角形
    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    // 如果非命令行模式，则进入循环绘制三角形
    while(key != 27)
    {
        // 清除颜色和深度缓冲区
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // 设置模型矩阵M
        r.set_model(get_model_matrix(angle));
        // 设置视图矩阵V
        r.set_view(get_view_matrix(eye_pos));
        // 设置投影矩阵P
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        // 绘制三角形
        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';
    }

    return 0;
}
// clang-format on