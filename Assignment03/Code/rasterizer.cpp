//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>

const bool SSAA = false;

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions) {
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices) {
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols) {
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_normals(const std::vector<Eigen::Vector3f>& normals) {
    auto id = get_next_id();
    nor_buf.emplace(id, normals);

    normal_id = id;

    return {id};
}


// Bresenham's line drawing algorithm
void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end) {
    auto x1 = begin.x();
    auto y1 = begin.y();
    auto x2 = end.x();
    auto y2 = end.y();

    Eigen::Vector3f line_color = {255, 255, 255};

    int x,y,dx,dy,dx1,dy1,px,py,xe,ye,i;

    dx=x2-x1;
    dy=y2-y1;
    dx1=fabs(dx);
    dy1=fabs(dy);
    px=2*dy1-dx1;
    py=2*dx1-dy1;

    if(dy1<=dx1) {
        if(dx>=0) {
            x=x1;
            y=y1;
            xe=x2;
        } else {
            x=x2;
            y=y2;
            xe=x1;
        }
        Eigen::Vector2i point = Eigen::Vector2i(x, y);
        set_pixel(point,line_color);
        for(i=0;x<xe;i++) {
            x=x+1;
            if(px<0) {
                px=px+2*dy1;
            } else {
                if((dx<0 && dy<0) || (dx>0 && dy>0)) {
                    y=y+1;
                } else {
                    y=y-1;
                }
                px=px+2*(dy1-dx1);
            }
//            delay(0);
            Eigen::Vector2i point = Eigen::Vector2i(x, y);
            set_pixel(point,line_color);
        }
    } else {
        if(dy>=0) {
            x=x1;
            y=y1;
            ye=y2;
        } else {
            x=x2;
            y=y2;
            ye=y1;
        }
        Eigen::Vector2i point = Eigen::Vector2i(x, y);
        set_pixel(point,line_color);
        for(i=0;y<ye;i++) {
            y=y+1;
            if(py<=0) {
                py=py+2*dx1;
            } else {
                if((dx<0 && dy<0) || (dx>0 && dy>0)) {
                    x=x+1;
                } else {
                    x=x-1;
                }
                py=py+2*(dx1-dy1);
            }
//            delay(0);
            Eigen::Vector2i point = Eigen::Vector2i(x, y);
            set_pixel(point,line_color);
        }
    }
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f) {
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

// 就是叉乘判断，判断点(x, y)是否在三角形内。
static bool insideTriangle(int x, int y, const Vector4f* _v){
    Vector3f v[3];
    for (int i = 0; i < 3; i++)
        v[i] = {_v[i].x(), _v[i].y(), 1.0};
    Vector3f f0, f1, f2;
    f0 = v[1].cross(v[0]);
    f1 = v[2].cross(v[1]);
    f2 = v[0].cross(v[2]);
    Vector3f p(x, y, 1.);
    if ((p.dot(f0) * f0.dot(v[2]) > 0) && (p.dot(f1) * f1.dot(v[0]) > 0) && (p.dot(f2) * f2.dot(v[1]) > 0))
        return true;
    return false;
}

// 计算重心坐标，用于插值计算
static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector4f* v){
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / 
	(v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - 
	v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / 
	(v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - 
	v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / 
	(v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - 
	v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(std::vector<Triangle *> &TriangleList) {

    // 根据远平面与近平面计算z值
    // 0.1 是近平面距离（zNear）
    // 50 是远平面距离（zFar）
    // 屏幕空间的深度值，范围是 [0.1, 50]
    // 当 z = 1 (近平面) 时：1 f1 + f2 = 0.1
    // 当 z = -1 (远平面) 时： -1 f1 + f2 =50
    // 为了移动到负半轴，需要对f2进行调整，原来f2 = (50 + 0.1) / 2.0;
    // 现在需要调整为f2 = (-50 + -0.1) / 2.0;
    // 这样z=-1时，-1 f1 + f2 = -50，z=1时，1 f1 + f2 = -0.1，这样就移动到了负半轴。
    float f1 = (50 - 0.1) / 2.0;
    float f2 = (-50 + -0.1) / 2.0;

    // 计算mvp矩阵
    Eigen::Matrix4f mvp = projection * view * model;
    // 遍历三角形列表，每一次循环就处理一个三角形
    for (const auto& t:TriangleList) {
        // 创建一个新的 Triangle 对象 newtri。
        // 并将指针 t 所指向的 Triangle 对象的内容复制到 newtri 中。
        // 解引用指针：*t 解引用指针 t，获取 t 所指向的 Triangle 对象。
        // 拷贝构造：使用拷贝构造函数将 *t 的内容复制到新的 Triangle 对象 newtri 中。
        // 这里newtri表示新三角形，表示屏幕空间中的三角形。
        // 这里t表示旧三角形，就是原来三维空间中的三角形。
        Triangle newtri = *t;

        // t 是一个指向 Triangle 对象的指针。t->v 表示访问 t 所指向的 Triangle 对象的成员变量 v。
        // 取得三维空间下的坐标（还未进行投影变换，没有投影到平面）
        std::array<Eigen::Vector4f, 3> mm {
                (view * model * t->v[0]),
                (view * model * t->v[1]),
                (view * model * t->v[2])
        };

        // viewspace_pos是一个包含三个Eigen::Vector3f的数组。
        std::array<Eigen::Vector3f, 3> viewspace_pos;
        // 将mm中的每个元素的x、y、z坐标提取出来，存储到viewspace_pos中。
        // 这样viewspace_pos就包含了三角形的三个顶点在三维空间中的坐标。
        std::transform( mm.begin(), mm.end(), viewspace_pos.begin(), [](auto& v) { 
                return v.template head<3>(); 
        });

        // 这里是将顶点坐标从模型空间转换到裁剪空间。进行完整mvp变换。
        // 这里v[]存放的是裁剪空间下的顶点坐标。
        Eigen::Vector4f v[] = {
                mvp * t->v[0],
                mvp * t->v[1],
                mvp * t->v[2]
        };
        // Homogeneous division 齐次除法，将裁剪空间坐标转换为NDC坐标（归一化设备坐标，NDC坐标的范围通常是[-1, 1]）
        // 注意：这里w分量并没有除以w分量，是保留了w分量值的，保留了三维空间中的z值。
        for (auto& vec : v) {
            vec.x() /= vec.w();
            vec.y() /= vec.w();
            vec.z() /= vec.w();
        }

        // 计算法线矩阵
        // inv_trans是 view * model 矩阵的逆矩阵的转置。
        // 相关说明法线矩阵为什么是view * model的逆矩阵的转置：https://blog.csdn.net/danshiming/article/details/132525514
        Eigen::Matrix4f inv_trans = (view * model).inverse().transpose();
        // 对法线进行变换，将法线从模型空间转换到裁剪空间，法线矩阵的w分量设置为0，因为法线是方向向量，w分量是0，如果w分量是1，表示一个点。
        Eigen::Vector4f n[] = {
                inv_trans * to_vec4(t->normal[0], 0.0f),
                inv_trans * to_vec4(t->normal[1], 0.0f),
                inv_trans * to_vec4(t->normal[2], 0.0f)
        };

        // 进行视口变换 Viewport transformation
        // 将裁剪空间坐标（在中心点的平面）移动到屏幕空间坐标上。
        // 注意z值，经过投影矩阵变换后，z值的值域是[-1, 1]，所以需要进行缩放。
        // 经过视口变换后，z值的值域重新变换到了[50, 0.1]，这里f1是近平面，f2是远平面，且都是正值。
        // f1 = (50 - 0.1) / 2.0; f2 = (50 + 0.1) / 2.0;
        // 1*f1+f2=50; -1*f1+f2=0.1;
        // 这样通过变换，z值的值域就从[-1, 1]变换到了[50, 0.1]，原来的1是近平面，-1是远平面。
        // 移动后，近平面是50，远平面是0.1，这样就会导致深度判断错误，将远平面0.1判断为近平面。
        // 这里会出现深度判断错误，解决办法两种：
        // 第一种：我们可以将深度平移会到z轴的负半轴，这样近平面就是-0.1，远平面就是-50，移动到负半轴的话，需要对f2进行调整，即float f2 = (-50 + -0.1) / 2.0;。
        // 第二种：我们可以将近平面与远平面进行翻转，这样远平面就是50，近平面就是0.1。翻转的话可以在投影矩阵的正交投影矩阵中进行第三行进行调整，用-zNear-zFar。
        for (auto & vert : v) {
            vert.x() = 0.5 * width * (vert.x() + 1.0);
            vert.y() = 0.5 * height * (vert.y() + 1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        // 更新顶点、法线、颜色，并不用更新纹理，纹理在初始阶段以及加载了。
        // 对新创建的三角形newtri进行设置
        // 将经过mvp变换后，且经过视口变换后的顶点坐标设置到newtri中
        for (int i = 0; i < 3; ++i) {
            // screen space coordinates屏幕空间坐标
            newtri.setVertex(i, v[i]);
        }
        // 将经过法线矩阵变换后的法线设置到newtri中
        for (int i = 0; i < 3; ++i) {
            // view space normal视图空间法线，因为法线只与相机位置有关，与模型位置无关。
            newtri.setNormal(i, n[i].head<3>());
        }
        // 将颜色设置到newtri中，这里是默认在newtri中设置的颜色
        newtri.setColor(0, 148, 121.0, 92.0);
        newtri.setColor(1, 148, 121.0, 92.0);
        newtri.setColor(2, 148, 121.0, 92.0);

        // 将viewspace_pos传递给rasterize_triangle函数，进行三角形光栅化
        // 因为要进行光线作用，所以需要viewspace_pos原本在三维空间中的坐标。
        rasterize_triangle(newtri, viewspace_pos);
    }
    if (SSAA) {
        for (int x = 0; x < width; x++) {
            for (int y = 0; y < height; y++) {
                Eigen::Vector2i p = {(float) x, (float) y};
                Eigen::Vector3f color(0, 0, 0);
                for (int i = 0; i < 4; i++)
                    color += frame_buf_2xSSAA[get_index(x, y)][i];
                color /= 4;
                set_pixel(p, color);
            }
        }
    }
}

// 插值函数
static Eigen::Vector3f 
interpolate(float alpha, float beta, float gamma, const Eigen::Vector3f& vert1, const Eigen::Vector3f& vert2, 
			const Eigen::Vector3f& vert3, float weight) {
    return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}

static Eigen::Vector2f 
interpolate(float alpha, float beta, float gamma, const Eigen::Vector2f& vert1, const Eigen::Vector2f& vert2, 
			const Eigen::Vector2f& vert3, float weight) {
    auto u = (alpha * vert1[0] + beta * vert2[0] + gamma * vert3[0]);
    auto v = (alpha * vert1[1] + beta * vert2[1] + gamma * vert3[1]);

    u /= weight;
    v /= weight;

    return Eigen::Vector2f(u, v);
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t, const std::array<Eigen::Vector3f, 3>& view_pos) {
    // TODO: From your HW3, get the triangle rasterization code.
    // TODO: Inside your rasterization loop:
    //    * v[i].w() is the vertex view space depth value z.
    //    * Z is interpolated view space depth for the current pixel
    //    * zp is depth between zNear and zFar, used for z-buffer

    // float Z = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    // float zp = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    // zp *= Z;

    // TODO: Interpolate the attributes:
    // auto interpolated_color
    // auto interpolated_normal
    // auto interpolated_texcoords
    // auto interpolated_shadingcoords

    // Use: fragment_shader_payload payload( interpolated_color, interpolated_normal.normalized(), interpolated_texcoords, texture ? &*texture : nullptr);
    // Use: payload.view_pos = interpolated_shadingcoords;
    // Use: Instead of passing the triangle's color directly to the frame buffer, pass the color to the shaders first to get the final color;
    // Use: auto pixel_color = fragment_shader(payload);

    // 获取mvp变换以及视口变换后的顶点坐标
    auto v = t.toVector4();
    // 计算包围盒
    int min_x = INT_MAX;
    int max_x = INT_MIN;
    int min_y = INT_MAX;
    int max_y = INT_MIN;
    for(auto point: v) {
        if (point[0] < min_x) min_x = point[0];
        if (point[0] > max_x) max_x = ceil(point[0]);
        if (point[1] < min_y) min_y = point[1];
        if (point[1] > max_y) max_y = ceil(point[1]);
    }
    // 遍历包围盒中的所有像素点
     for (int x = min_x; x <= max_x; x++) {
        for (int y = min_y; y <= max_y; y++) {
            // 如果开启SSAA，则每个像素点划分四个独立的小像素点
            if(SSAA) {
                int index = 0;
                // 划分四个小的像素
                for (float i = 0.25; i < 1.0; i += 0.5) {
                    for (float j = 0.25; j < 1.0; j += 0.5) {
                        if (insideTriangle(x + i, y + j, t.v)) {
                            //得到这个点的重心坐标
                            auto abg = computeBarycentric2D((float) x + i, (float) y + j, t.v);
                            float alpha = std::get<0>(abg);
                            float beta = std::get<1>(abg);
                            float gamma = std::get<2>(abg);
                            //z-buffer插值
                            float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w()); //归一化系数
                            float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() +
                                                   gamma * v[2].z() / v[2].w();
                            z_interpolated *= w_reciprocal;

                            if (abs(z_interpolated) < depth_buf_2xSSAA[get_index(x, y)][index]) {
                                Eigen::Vector2i p = {(float) x, (float) y};
                                // 颜色插值
                                auto interpolated_color = interpolate(alpha, beta, gamma, t.color[0], t.color[1],
                                                                      t.color[2], 1);
                                // 法向量插值
                                auto interpolated_normal = interpolate(alpha, beta, gamma, t.normal[0], t.normal[1],
                                                                       t.normal[2], 1);
                                // 纹理颜色插值
                                auto interpolated_texcoords = interpolate(alpha, beta, gamma, t.tex_coords[0],
                                                                          t.tex_coords[1], t.tex_coords[2], 1);
                                // 内部点位置插值
                                auto interpolated_shadingcoords = interpolate(alpha, beta, gamma, view_pos[0],
                                                                              view_pos[1], view_pos[2], 1);

                                fragment_shader_payload payload(interpolated_color, interpolated_normal.normalized(),
                                                                interpolated_texcoords, texture ? &*texture : nullptr);

                                payload.view_pos = interpolated_shadingcoords;
                                auto pixel_color = fragment_shader(payload);
                                // set_pixel(p, pixel_color); //设置颜色
                                frame_buf_2xSSAA[get_index(x, y)][index] = pixel_color;
                                depth_buf_2xSSAA[get_index(x, y)][index] = abs(z_interpolated);//更新z值
                            }
                        }
                        index++;
                    }
                }
            }
            else {
                // 以像素中心点作为采样点
                if (insideTriangle((float) x+0.5, (float) y+0.5, t.v)) {
                    // 计算重心坐标
                    auto abg = computeBarycentric2D((float) x+0.5, (float) y+0.5, t.v);
                    float alpha = std::get<0>(abg);
                    float beta = std::get<1>(abg);
                    float gamma = std::get<2>(abg);

                    // z-buffer插值
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = 
							alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;

                    // 进行深度判断
                    // 如果当前像素的深度更靠近相机，则更新深度缓冲
                    if (abs(z_interpolated)<depth_buf[get_index(x, y)]) {
                        
                        Eigen::Vector2i p = {(float)x, (float)y};

                        // 插值计算
                        // 插值函数interpolate最后一个参数是权重，这里设置为1，表示平均的一个插值。
                        // 1.颜色插值
                        auto interpolated_color = interpolate(alpha, beta, gamma, t.color[0], t.color[1], 
															  t.color[2], 1);
                        // 2.法线插值
                        auto interpolated_normal = interpolate(alpha, beta, gamma, t.normal[0], t.normal[1], 
															  t.normal[2], 1);
                        // 3.纹理插值
                        auto interpolated_texcoords = interpolate(alpha, beta, gamma, t.tex_coords[0], t.tex_coords[1], 
															  t.tex_coords[2], 1);
                        // 4.内部点三维空间插值，也就是当前着色的点在三维空间中的坐标x，y，z，而不是屏幕空间的插值
                        auto interpolated_shadingcoords = interpolate(alpha, beta, gamma, view_pos[0], view_pos[1], 
															  view_pos[2], 1);

                        // 将插值后的颜色、法线、纹理坐标、着色点三维坐标传递给着色器
                        fragment_shader_payload payload(interpolated_color, interpolated_normal.normalized(),
                            							interpolated_texcoords, texture ? &*texture : nullptr); //如果纹理不是空指针，则传递纹理
                        // 将着色点三维坐标传递给着色器
                        payload.view_pos = interpolated_shadingcoords;

                        // 调用着色器，计算着色点颜色
                        // 这里的着色器是在main函数中设置的，r.set_fragment_shader(active_shader);
                        // 这里着色器传入的参数就是当前像素对应的信息，着色器就根据这些信息计算出着色点颜色
                        auto pixel_color = fragment_shader(payload);

                        // 设置当前像素点的颜色
                        // 这里的p是屏幕空间坐标，这里的屏幕空间坐标原点与OpenCV图像中的坐标原点是不一样的，
                        // 所以在set_pixel函数中，需要将原点从左下角改为左上角，以适应OpenCV图像中的坐标。
                        set_pixel(p, pixel_color); 

                        // 更新深度缓冲区
                        // 这里get_index函数进行转换，将屏幕空间坐标原点从左下角改为左上角，以适应OpenCV图像中的坐标
                        depth_buf[get_index(x, y)] = abs(z_interpolated);
                    }

                }


            }
        }
    }

 
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m) {
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v) {
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p) {
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff) {
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color) {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        // 存储小像素的颜色信息
        for(int i=0; i<frame_buf_2xSSAA.size(); i++) {
            frame_buf_2xSSAA[i].resize(4);
            std::fill(frame_buf_2xSSAA[i].begin(), frame_buf_2xSSAA[i].end(), Eigen::Vector3f{0, 0, 0});
        }
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth) {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        // 存储小像素的深度信息
        for(int i=0; i<depth_buf_2xSSAA.size(); i++) {
            depth_buf_2xSSAA[i].resize(4);
            std::fill(depth_buf_2xSSAA[i].begin(), depth_buf_2xSSAA[i].end(), std::numeric_limits<float>::infinity());
        }
    }
}

rst::rasterizer::rasterizer(int h, int w) : width(w), height(h) {
    // 帧缓存
    frame_buf.resize(w * h);
    // 深度缓存
    depth_buf.resize(w * h);

    // SSAA超采样，每个像素划分四个独立的小像素。
    frame_buf_2xSSAA.resize(w * h);
    depth_buf_2xSSAA.resize(w * h);

    // std::nullopt是C++17引入的一个特殊值
    // 用于表示std::optional对象不包含值的特殊标识。std::optional是一个模板类，可以用来存储一个可能为空的值。
    texture = std::nullopt;
}

// 获取当前像素点的索引
// 也是进行转换，将屏幕空间坐标原点从左下角改为左上角，以适应OpenCV图像中的坐标
int rst::rasterizer::get_index(int x, int y) {
    return (height- 1 -y)*width + x;
}

// 设置当前像素点的颜色
// 进行转换，将屏幕空间坐标原点从左下角改为左上角，以适应OpenCV图像中的坐标
// 并更新帧缓存，进行颜色设置
void rst::rasterizer::set_pixel(const Vector2i &point, const Eigen::Vector3f &color) {
    //old index: auto ind = point.y() + point.x() * width;
    int ind = (height- 1 -point.y())*width + point.x();
    // 将颜色设置到当前像素点
    frame_buf[ind] = color;
}

void rst::rasterizer::set_vertex_shader(std::function<Eigen::Vector3f(vertex_shader_payload)> vert_shader) {
    vertex_shader = vert_shader;
}

void rst::rasterizer::set_fragment_shader(std::function<Eigen::Vector3f(fragment_shader_payload)> frag_shader) {
    fragment_shader = frag_shader;
}

