//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <stdexcept>


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

// Bresenham's line drawing algorithm
// 绘制线段，传入线段的起点和终点
// 离散画线法，没有涉及到光栅化。
// 也就是将线段离散化，然后逐个像素点绘制。
// Code taken from a stack overflow answer: https://stackoverflow.com/a/16405254
// 原理：
void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end)
{
    auto x1 = begin.x();
    auto y1 = begin.y();
    auto x2 = end.x();
    auto y2 = end.y();

    // 设置线段的颜色，白色
    // Eigen::Vector3f line_color = {255, 255, 255}; // 白色
    Eigen::Vector3f line_color = {0, 0, 255}; // 红色

    int x,y,dx,dy,dx1,dy1,px,py,xe,ye,i;

    // 计算dx和dy
    dx=x2-x1;
    dy=y2-y1;
    // 计算dx和dy的绝对值
    dx1=fabs(dx);
    dy1=fabs(dy);
    // 计算px和py，误差判断值。
    px=2*dy1-dx1; // 初始误差值
    py=2*dx1-dy1; // 初始误差值

    // 判断线段是"更水平"还是"更垂直"
    // dy1<=dx1 表示线段的水平方向变化大于或等于垂直方向变化，即线段更接近水平线
    // 反之则表示线段更接近垂直线
    if(dy1<=dx1)
    {
        // 如果dx大于等于0，说明x2大于等于x1，即起点在终点左边
        if(dx>=0)
        {
            x=x1;
            y=y1;
            xe=x2;
        }
        else
        {
            x=x2;
            y=y2;
            xe=x1;
        }
        // 绘制起点
        Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
        // 设置像素点颜色
        set_pixel(point,line_color);
        // 从起点开始，逐个像素点绘制像素点
        // 从起点x坐标开始，一直画到终点x坐标(xe)
        for(i=0;x<xe;i++)
        {
            x=x+1; // 每次循环x坐标都向右移动一个像素

            // 关键部分
            // 每次在x方向移动一个像素后，需要决定是否在y方向也移动一个像素
            // 如果px < 0，说明当前误差较小，下一个像素点只需在x方向移动
            // 如果px >= 0，说明误差较大，下一个像素点需要在x和y方向都移动
            if(px<0)
            {
                px=px+2*dy1; // 更新误差值
            }
            else // 如果px >= 0，说明误差较大，下一个像素点需要在x方向移动基础上，再向y方向移动
            {
                // 此时需要判断移动方向（向上还是向下）
                // 如果dx和dy同号（都为正或都为负），y坐标+1
                // 如果dx和dy异号，y坐标-1

                // 如果dx<0 && dy<0，说明线段在第四象限，y方向向下移动
                // 如果dx>0 && dy>0，说明线段在第一象限，y方向向上移动
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    y=y+1; // 向上移动
                }
                else
                {
                    y=y-1; // 向下移动
                }
                px=px+2*(dy1-dx1); // 更新误差值
            }
//            delay(0);
            Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
            set_pixel(point,line_color);
        }
    }
    else
    {
        if(dy>=0)
        {
            x=x1;
            y=y1;
            ye=y2;
        }
        else
        {
            x=x2;
            y=y2;
            ye=y1;
        }
        Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
        set_pixel(point,line_color);
        for(i=0;y<ye;i++)
        {
            y=y+1;
            if(py<=0)
            {
                py=py+2*dx1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    x=x+1;
                }
                else
                {
                    x=x-1;
                }
                py=py+2*(dx1-dy1);
            }
//            delay(0);
            Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
            set_pixel(point,line_color);
        }
    }
}

// 将三维向量转换为四维向量，w是齐次坐标中的w分量，默认为1.0f。
auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

void rst::rasterizer::draw(rst::pos_buf_id pos_buffer, rst::ind_buf_id ind_buffer, rst::Primitive type)
{
    if (type != rst::Primitive::Triangle) // 判断绘制的是否是三角形
    {
        throw std::runtime_error("Drawing primitives other than triangle is not implemented yet!");
    }

    // 从顶点缓存中读取当前帧所需要绘制的三角形信息
    // pos_buf[pos_buffer.pos_id] 是顶点缓存，pos_buffer.pos_id 是顶点缓存的索引
    auto& buf = pos_buf[pos_buffer.pos_id];
    // ind_buf[ind_buffer.ind_id] 是三角形索引缓冲，ind_buffer.ind_id 是三角形索引缓冲的索引
    auto& ind = ind_buf[ind_buffer.ind_id];

    // 计算透视投影的参数
    float f1 = (100 - 0.1) / 2.0;
    float f2 = (100 + 0.1) / 2.0;

    // 1.计算mvp矩阵：
    // 先进行模型变换，再进行视图变换，最后进行投影变换。
    // 注意矩阵运算，是从右到左的。
    Eigen::Matrix4f mvp = projection * view * model;

    // 这里ind是三角形索引缓冲，ind中的每个元素对应一个三角形。i就是ind中的元素。
    // 每循环一次，就绘制一个三角形
    for (auto& i : ind)
    {
        Triangle t; // 初始化一个空三角形

        // 2.v是个数据类型为4维向量(点的齐次坐标)的数组，共有三个元素代表三角形的顶点，v数组对应MVP矩阵变换后的顶点。
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };

        // 将齐次坐标归一化，归一化齐次坐标，将w分量归一化到1.0f。
        for (auto& vec : v) {
            vec /= vec.w();
        }

        // 3.视口变换：
        // 将x-y平面[-1,1]^2变换到显示器屏幕[0, width] × [0, height]空间
        // 这是一个2D平面的变换
        // 近平面中心是(0,0)，而屏幕坐标的左下角为(0,0)。
        // 先将x=[-1,1]、y=[-1,1]分别拉伸到宽度为width、高度为height
        // 然后原本整个场景中心是(0,0)，移动到(width/2, height/2)，需要x方向移动width/2，y方向移动height/2
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        // 4.将变换后的三角形交给循环开始创建的空的三角形类
        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            // 原始代码中，这里重复了两次。所以注释掉多余的。
            // t.setVertex(i, v[i].head<3>());
            // t.setVertex(i, v[i].head<3>());
        }
        // 设置三角形的颜色
        t.setColor(0, 120.0,  0.0,  0.0);
        t.setColor(1, 0.0  ,120.0,  0.0);
        t.setColor(2, 0.0  ,  0.0,120.0);

        // 绘制三角形的边框，传入三角形类，也就是刚才实例化的三角形类
        rasterize_wireframe(t);
    }
}

// 绘制三角形的边框，传入三角形类
void rst::rasterizer::rasterize_wireframe(const Triangle& t)
{
    // 绘制三角形的边框
    draw_line(t.c(), t.a());
    draw_line(t.c(), t.b());
    draw_line(t.b(), t.a());
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
    // 清空颜色缓冲区(frame buffer)
    // 这部分代码将所有像素的颜色设置为黑色(0,0,0)。这就像是在画新的一帧之前，先把画布涂成黑色。
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{120, 0, 0}); // 设置背景颜色为蓝色
    }

    // 清空深度缓冲区(depth buffer)
    // 这部分将深度缓冲区的所有值设置为正无穷。深度缓冲区用于z-buffer算法，确保3D渲染时物体的前后关系正确。初始化为正无穷是为了保证第一个渲染的像素一定会被写入。
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h); // 设置帧缓冲区大小
    depth_buf.resize(w * h); // 深度缓冲区大小 （这里并没有用到）
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    if (point.x() < 0 || point.x() >= width ||
        point.y() < 0 || point.y() >= height) return;
    
    // 这里需要注意屏幕坐标系和像素坐标系
    // 屏幕坐标系：左上角为(0,0)，x轴向右，y轴向下
    // 像素坐标系：左下角为(0,0)，x轴向右，y轴向上
    // 所以需要将屏幕坐标系转换为像素坐标系
    auto ind = (height-point.y())*width + point.x();
    frame_buf[ind] = color;
}

