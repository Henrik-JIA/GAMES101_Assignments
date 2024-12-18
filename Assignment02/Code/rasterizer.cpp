// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

#define SSAA false

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(int x, int y, const Vector3f* _v)
{   
    // 向量减法获取三角形边向量
    Eigen::Vector2f p0p1(_v[1].x() - _v[0].x(), _v[1].y() - _v[0].y());
    Eigen::Vector2f p1p2(_v[2].x() - _v[1].x(), _v[2].y() - _v[1].y());
    Eigen::Vector2f p2p0(_v[0].x() - _v[2].x(), _v[0].y() - _v[2].y());

    // 向量减法获取点(x, y)到三角形顶点向量
    Eigen::Vector2f p0p(x - _v[0].x(), y - _v[0].y());
    Eigen::Vector2f p1p(x - _v[1].x(), y - _v[1].y());
    Eigen::Vector2f p2p(x - _v[2].x(), y - _v[2].y());
    
    // 计算叉积的标量值
    float cross1 = p0p1.x() * p0p.y() - p0p1.y() * p0p.x();
    float cross2 = p1p2.x() * p1p.y() - p1p2.y() * p1p.x();
    float cross3 = p2p0.x() * p2p.y() - p2p0.y() * p2p.x();

    // 判断叉积的每一个元素是否同正或同负
    return (cross1 > 0 && cross2 > 0 && cross3 > 0) ||
           (cross1 < 0 && cross2 < 0 && cross3 < 0);
}

// 计算重心坐标，用于插值计算
static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    // 获取顶点坐标
    auto& buf = pos_buf[pos_buffer.pos_id];
    // 获取三角形顶点索引
    auto& ind = ind_buf[ind_buffer.ind_id];
    // 获取三角形顶点颜色
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    // 计算MVP矩阵
    Eigen::Matrix4f mvp = projection * view * model;
    // ���历三角形顶点分组索引（这里有2个三角形，所以遍历2次）
    for (auto& i : ind)
    {
        // 创建三角形对象
        Triangle t;
        // 遍历三角形顶点
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f), // 将三角形顶点坐标转换为齐次坐标
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        
        // Homogeneous division
        // 齐次坐标归一化使其表示一个点。
        // 但是这里会丢失w的值，也就是三维空间中的深度信息丢失了。
        // 作用3中得到修正。
        for (auto& vec : v) {
            vec /= vec.w();
        }

        //Viewport transformation
        // 视口变换，将三角形顶点坐标从[-1,1]映射到[0,width]和[0,height]的屏幕坐标系
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        // 将三角形顶点坐标设置到三角形对象中
        // 这里也是存在问题的，需要v[i].head<4>()应该是取v[i]的x,y,z,w值，少了w值深度信息。
        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            // t.setVertex(i, v[i].head<3>());
            // t.setVertex(i, v[i].head<3>());
        }

        // 将三角形顶点颜色设置到三角形对象中
        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];
        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t, SSAA);
    }

    if (SSAA) {
        for (int x = 0; x< width;x++) {
            for (int y = 0; y< height;y++) {
                Eigen::Vector3f color(0,0,0);
                for (int i = 0; i<4;i++) {
                    color += frame_buf_ssaa[get_index(x,y) + i];
                }
                color /= 4;
                set_pixel(Eigen::Vector3f(x,y,1.0f), color);
            }
        }
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t, bool SSAA) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.

    // 获取AABB包围盒
    // 将x坐标和y坐标分别取出到一个数组中，并排序，取最小值和最大值
    std::vector<float> x_array{v[0].x(), v[1].x(), v[2].x()};
    std::vector<float> y_array{v[0].y(), v[1].y(), v[2].y()};
    std::sort(x_array.begin(), x_array.end());
    std::sort(y_array.begin(), y_array.end());
    int x_min = floor(x_array[0]);
    int x_max = ceil(x_array[2]);
    int y_min = floor(y_array[0]);
    int y_max = ceil(y_array[2]);

    // 遍历AABB包围盒内的所有像素点
    // 从包围盒的左下角开始遍历
    for (int x = x_min; x < x_max; ++x) {
        for (int y = y_min; y < y_max; ++y) {
            // 不在视锥体内的像素点不处理
            if (x < 0 || x >= width || y < 0 || y >= height) {
                continue;
            }

            // 进行超采样判断，SSAA默认为false不开超采样
            if (SSAA) {
                // 进行超采样
                int index = 0;
                // 划分四个小像素
                for (float i = 0.25; i<1.0; i+=0.5) {
                    for (float j = 0.25; j<1.0; j+=0.5) {
                        if (insideTriangle(x + i, y + j, t.v)) {
                            auto[alpha, beta, gamma] = computeBarycentric2D(x + i, y + j, t.v);
                            float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                            float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                            z_interpolated *= w_reciprocal;
                            int index_ans = get_index(x,y) +index;
                            if (abs(z_interpolated) < depth_buf_ssaa[index_ans]) {
                                depth_buf_ssaa[index_ans] = abs(z_interpolated);
                                frame_buf_ssaa[index_ans] = t.getColor();
                            }
                        }
                        index++;
                    }
                }
            }else{
                // 判断像素中心是否在三角形内
                // 像素中心坐标为(x + 0.5, y + 0.5)
                // t.v是三角形的三个顶点
                if (insideTriangle(x + 0.5, y + 0.5, t.v)) {
                    // 计算重心坐标
                    auto[alpha, beta, gamma] = computeBarycentric2D(x + 0.5, y + 0.5, t.v);
                    
                    // 计算深度值
                    // 这三步为透视矫正，Z的插值应该是在三维空间下才准确，在二维平面下直接插值计算权重的结果不正确。
                    // 判断三角形的前后关系，即判断深度信息，然而深度信息在之前投影矩阵中已经丢失了，现在就需要重新找回来。
                    // 透视矫正插值：https://zhuanlan.zhihu.com/p/144331875
                    // 作业1、2框架相关问题：https://zhanlan.zhihu.com/p/509902950
                    // w_reciprocal已经是三维空间下的深度信息，但对于遮挡关系还是选择二维平面下的深度信息，z_interpolated最后值为透视矫正插值后的二维平面下正确坐标深度信息。
                    // 当然也可以用w-buffer，但普遍使用z-buffer。
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;

                    // 注意：深度缓存初始时正无穷，个人习惯坐标的z值取负值。
                    // 插值出的z值与深度缓存中记录的z值比较，set_pixel将帧缓存中该像素的颜色更新，最后深度缓存更新。
                    if (abs(z_interpolated) < depth_buf[get_index(x, y)]) {
                        Eigen::Vector3f point(x, y, 1.0f); // 获取像素点坐标
                        set_pixel(point, t.getColor()); // 将三角形颜色设置到像素点
                        depth_buf[get_index(x, y)] = abs(z_interpolated); // 更新深度缓存
                    }

                }
            }
            

        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        // 清除帧缓冲区
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        // 清除SSAA帧缓冲区
        std::fill(frame_buf_ssaa.begin(), frame_buf_ssaa.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        // 清除深度缓冲区
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        // 清除SSAA深度缓冲区
        std::fill(depth_buf_ssaa.begin(), depth_buf_ssaa.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    // 帧缓存，保存图像中每个像素显示什么颜色。
    frame_buf.resize(w * h);
    // 深度缓存，保存每个像素点的深度值。
    // Depth buffer有两种：z-buffer和w-buffer。
    depth_buf.resize(w * h);

    // SSAA超采样，每个像素点划分成4个子像素区
    frame_buf_ssaa.resize(w * h * 4);
    depth_buf_ssaa.resize(w * h * 4);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on