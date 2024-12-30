// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

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

// 就是叉乘判断，判断点(x, y)是否在三角形内。
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
    
    // 计算视口变换的参数
    // 视口变换：将[-1,1]的坐标系映射到[0,width]和[0,height]的屏幕坐标系
    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    // 计算MVP矩阵
    Eigen::Matrix4f mvp = projection * view * model;

    // 遍历三角形顶点分组索引（这里有2个三角形，所以遍历2次）
    for (auto& i : ind)
    {
        // 创建三角形对象
        Triangle t;
        // 遍历三角形顶点
        Eigen::Vector4f v[] = {
                // to_vec4将三角形顶点坐标转换为齐次坐标
                // 齐次坐标最后一个分量如果是1，表示一个点；如果是0，表示一个向量。
                mvp * to_vec4(buf[i[0]], 1.0f), 
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
                // 经过mvp变换后，齐次坐标最后一个分量就表示深度信息。
        };
        
        // 齐次坐标归一化使其表示一个点。
        // w分量进行了归一化处理，但是这里会丢失w的值，也就是三维空间中的深度信息丢失了。
        // 真确的应该是对x、y、z除以w分量，w分量本身不做处理。
        // 作用3中得到修正。
        for (auto& vec : v) {
            vec /= vec.w();
        }

        // Viewport transformation
        // 将mvp变化后的近平面移到屏幕坐标系上。
        // 视口变换，将三角形顶点坐标从[-1,1]映射到[0,width]和[0,height]的屏幕坐标系
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        // 将三角形顶点坐标设置到三角形对象中
        // 这里也是存在问题的，这里只是存入x,y,z值，并没有将w值存入。
        // 需要v[i].head<4>()应该是取v[i]的x,y,z,w值，少了w值深度信息。
        // 这里我们可以注意到只存入了x,y,z值，并没有存入w值，而且并没有对w分量进行归一化处理。
        // 所以这里z值是错误的。
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

        // 对三角形进行光栅化
        rasterize_triangle(t, SSAA);
    }

    if (SSAA) {
        // 对SSAA帧缓存进行平均化处理
        for (int x = 0; x< width;x++) {
            for (int y = 0; y< height;y++) {
                Eigen::Vector3f color(0,0,0);
                // 遍历4个子像素
                for (int i = 0; i<4;i++) {
                    color += frame_buf_ssaa[get_index(x,y) + i];
                }
                // 取平均值
                color /= 4;
                // 将平均值设置到像素点
                set_pixel(Eigen::Vector3f(x,y,1.0f), color);
            }
        }
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t, bool SSAA) {
    // 获取三角形顶点坐标
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
    // 我们对三角形进行光栅化，需要遍历三角形所在的包围盒内的所有像素点。
    // 我们还需要判断像素点是否在三角形内，如果不在三角形内，则不处理。
    // 所以需要获取三角形的AABB包围盒，这样可以减少遍历的像素点数量。
    // 将x坐标和y坐标分别取出到一个数组中，并排序，取最小值和最大值
    std::vector<float> x_array{v[0].x(), v[1].x(), v[2].x()};
    std::vector<float> y_array{v[0].y(), v[1].y(), v[2].y()};
    std::sort(x_array.begin(), x_array.end());
    std::sort(y_array.begin(), y_array.end());
    int x_min = floor(x_array[0]); // 向下取整
    int x_max = ceil(x_array[2]); // 向上取整
    int y_min = floor(y_array[0]); // 向下取整
    int y_max = ceil(y_array[2]); // 向上取整
    
    // 遍历AABB包围盒内的所有像素点
    // 从包围盒的左下角开始遍历
    for (int x = x_min; x < x_max; ++x) {
        for (int y = y_min; y < y_max; ++y) {
            // 不在视锥体内的像素点不处理，也就是如果超出屏幕范围的像素点直接跳过
            if (x < 0 || x >= width || y < 0 || y >= height) {
                continue;
            }

            // 这里会存在一个分支：
            // 进行超采样判断，SSAA默认为false不开超采样
            if (SSAA) {
                // 进行超采样
                // 我们相拥一维数组方式访问二维数组。每次index+1，表示访问下一个像素点。
                int index = 0;
                // 划分四个小像素
                // 初始都是0.25，所以每次+0.5，表示访问下一个分割块的中心。
                for (float i = 0.25; i<1.0; i+=0.5) {
                    for (float j = 0.25; j<1.0; j+=0.5) {
                        // 判断每个小像素点是否在三角形内
                        bool inside = insideTriangle(x + i, y + j, t.v);
                        if (inside) {
                            // 计算权重值
                            auto[alpha, beta, gamma] = computeBarycentric2D(x + i, y + j, t.v);
                            
                            // 计算正确的深度值
                            float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                            float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                            z_interpolated *= w_reciprocal;
                            
                            // 获取像素点索引
                            // 在SSAA(超采样抗锯齿)中，每个像素被分成了2x2=4个子像素
                            // 对于普通的frame_buf，每个像素只需要一个存储位置
                            // 但对于frame_buf_ssaa，每个像素需要4个连续的存储位置来存储4个子像素
                            // 这里的index每次+1，总共+4，在原本的index基础上+4，表示访问下一个子像素。
                            int index_ans = get_index(x,y) +index;

                            // 更新深度缓存和帧缓存
                            if (abs(z_interpolated) < depth_buf_ssaa[index_ans]) {
                                // 这里更新的并不是实际需要的帧缓存和深度缓存
                                // 所以最后需要进行平均化处理
                                // 现在的分辨率是比实际的分辨率大4倍，所以最后需要进行平均化处理
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
                float pix_center_x = x + 0.5;
                float pix_center_y = y + 0.5;
                // 判断像素中心是否在三角形内，t.v是三角形的三个顶点
                bool inside = insideTriangle(pix_center_x, pix_center_y, t.v);
                // 如果像素中心在三角形内，则进行插值计算
                if (inside) {
                    // α、β、γ表示三角形内某个点受三个顶点的影响权重
                    // 后面需要插值颜色、纹理，坐标等，所以需要计算三角形内任意一点受三个顶点的影响权重。
                    // 这样就可以通过α、β、γ计算出三角形内任意一点的颜色、纹理，坐标等。
                    auto[alpha, beta, gamma] = computeBarycentric2D(pix_center_x, pix_center_y, t.v);
                
                    // 计算深度值
                    // 着色需要判断三角形的前后关系，就需要判断深度信息，但是深度信息在之前投影矩阵中已经丢失了。
                    // 现在就需要重新找回来深度信息。
                    // 透视矫正插值：https://zhuanlan.zhihu.com/p/144331875
                    // 作业1、2框架相关问题（该公式1/z0 = α/z1+ β/z2 + γ/z3在这个链接中）：https://zhuanlan.zhihu.com/p/509902950
                    // w_reciprocal已经是三维空间下的深度信息，但对于遮挡关系还是选择二维平面下的深度信息（透视矫正后的正确深度插值），z_interpolated最后值为透视矫正插值后的二维平面下正确坐标深度信息。
                    // w-buffer并不符合实际的深度变化，w-buffer是线性的
                    // z-buffer是透视矫正后的正确深度插值，z-buffer是非线性的。
                    // 公式 z = (α*z1/w1 + β*z2/w2 + γ*z3/w3) / (α/w1 + β/w2 + γ/w3)
                    // 计算透视矫正的权重
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    // 计算深度值的加权和
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    // 应用透视矫正后的深度值
                    // 这里z_interpolated是一个负数，因为相机看向-z方向，所以z值为负。
                    // 近平面是原点。
                    z_interpolated *= w_reciprocal;

                    // 注意：深度缓存初始时正无穷，个人习惯坐标的z值取负值。
                    // 插值出的z值与深度缓存中记录的z值比较，set_pixel将帧缓存中该像素的颜色更新，最后深度缓存更新。
                    // 判断的是如果abs(z_interpolated)小于深度缓存中的z值，则更新深度缓存和帧缓存。
                    // 这里z_interpolated是负数，所以abs(z_interpolated)是正数，只比较数值大小，因为数值越小，越靠近相机。
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
    // 清除帧缓冲区
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        // 这里的数组都是一维数组，所以直接用std::fill填充
        // 清除帧缓冲区
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        // 清除SSAA帧缓冲区
        std::fill(frame_buf_ssaa.begin(), frame_buf_ssaa.end(), Eigen::Vector3f{0, 0, 0});
    }
    // 清除深度缓冲区
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
    // https://developer.aliyun.com/article/49272
    depth_buf.resize(w * h);

    // SSAA超采样，每个像素点划分成4个子像素区
    frame_buf_ssaa.resize(w * h * 4);
    depth_buf_ssaa.resize(w * h * 4);
}

// 屏幕坐标系 [-1,1] 
//     ↓ (视口变换)
// 像素坐标系 [0,width/height]（左下角原点）
//     ↓ (get_index转换)
// 图像存储坐标系 [0,width/height]（左上角原点）
// 传入 get_index(x, y) 的 x 和 y 是像素坐标系（左下角原点）的坐标，函数通过 height-1-y 将其转换为图像存储坐标系（左上角原点）。
int rst::rasterizer::get_index(int x, int y)
{

    return (height-1-y)*width + x; // 这里x是列偏移量，y是行偏移量
}

// 设置像素点颜色
void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    // 新的索引计算方式
    auto ind = (height-1-point.y())*width + point.x();
    // 设置像素点颜色
    frame_buf[ind] = color;

}

// clang-format on