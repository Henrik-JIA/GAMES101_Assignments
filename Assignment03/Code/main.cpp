#include <iostream>
#include <opencv2/opencv.hpp>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"

// 这是一个三方库，用于帮助我们加载obj模型的。
#include "OBJ_Loader.h" 

// 获取视图矩阵
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos) {
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

Eigen::Matrix4f get_model_matrix(float angle) {
    Eigen::Matrix4f rotation;
    angle = angle * MY_PI / 180.f;
    rotation << cos(angle), 0, sin(angle), 0,
                0, 1, 0, 0,
                -sin(angle), 0, cos(angle), 0,
                0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale << 2.5, 0, 0, 0,
              0, 2.5, 0, 0,
              0, 0, 2.5, 0,
              0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    return translate * rotation * scale;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar) {
    // TODO: Use the same projection matrix from the previous assignments
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f m;
    m << -zNear, 0, 0, 0,
            0, -zNear, 0, 0,
            0, 0, -zNear - zFar, -zNear * zFar,
            0, 0, 1, 0;


    float halve = (eye_fov / 2) * MY_PI / 180;
    float top = tan(halve) * zNear;
    float bottom = -top;
    float right = top * aspect_ratio;
    float left = -right;
    Eigen::Matrix4f n, p;
    n << 2 / (right - left), 0, 0, 0,
            0, 2 / (top - bottom), 0, 0,
            0, 0, 2 / (zFar - zNear), 0,
            0, 0, 0, 1;

    p << 1, 0, 0, -(right + left) / 2,
            0, 1, 0, -(top + bottom) / 2,
            0, 0, 1, -(-zFar + -zNear) / 2,
            0, 0, 0, 1;

    projection = n * p * m;

    return projection;
}

// 顶点着色器
// 这里只返回顶点坐标。
// 但是在opengl中，顶点着色器会做一些mvp变换，将顶点坐标从模型空间转换到裁剪空间。
Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload) {
    return payload.position;
}

// 法线着色器，参数是当前像素对应的信息，返回的是法线颜色
Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload) {
    // 归一化后的法线向量的分量初始在 [-1, 1]，在三维空间中，法线向量的每个分量（x, y, z）可以在 [-1, 1] 范围内变化，以表示不同的方向。例如，(0, 0, 1) 表示指向 z 轴正方向，而 (0, 0, -1) 表示指向 z 轴负方向。
    // 将归一化后的法线向量的每个分量加1，然后除以2。这是为了将法线向量的值从 [-1, 1] 映射到 [0, 1] 区间。
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    // 将法线颜色从 [0, 1] 映射到 [0, 255] 区间，以适应图像的像素值范围。
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis) {
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}

struct light {
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

// Blinn-Phong模型着色器
Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload) {
    // 环境光系数
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    // 漫反射光系数，该系数就是当前像素的颜色
    Eigen::Vector3f kd = payload.color;
    // 镜面反射光系数
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    // 光源，光源位置和光源强度
    auto l1 = light{{20, 20, 20}, 
					{500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, 
					{500, 500, 500}};
    std::vector<light> lights = {l1, l2};
    // 环境光强度
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    // 观察点位置，因为这里的观察点位置已经在原点了，相机通过视图矩阵将观察点位置移动到原点，所以这里设置为原点。
    Eigen::Vector3f eye_pos{0, 0, 0};

    // p是高光系数，表示光泽度。p越大，高光范围越小，就像一个点一样；p越小，高光范围越大，就像一个片一样。
    float p = 150;

    // 当前像素的颜色
    Eigen::Vector3f color = payload.color;
    // 当前像素的三维空间位置
    Eigen::Vector3f point = payload.view_pos;
    // 当前像素的法线
    Eigen::Vector3f normal = payload.normal;

    // 最终颜色
    Eigen::Vector3f result_color = {0, 0, 0};
    // 遍历所有光源，目前只有两个光源
    for (auto& light : lights) {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        auto v = eye_pos - point; //v为出射光方向（指向眼睛）
        auto l = light.position - point; //l为指向入射光源方向
        auto h = (v.normalized() + l.normalized()).normalized(); //h为半程向量即v+l归一化后的单位向量
        auto r_2 = l.dot(l); //衰减因子，光源到 shading point的距离的平方，用于计算光能量的的损失
        
        // 环境光，可以看清物体轮廓
        auto ambient = ka.cwiseProduct(amb_light_intensity);
        // 漫反射光，可以看清物体颜色
        auto diffuse = kd.cwiseProduct(light.intensity / r_2) * std::max(0.0f, normal.normalized().dot(l.normalized()));
        // 镜面反射光，可以看清物体光泽度
        auto specular = ks.cwiseProduct(light.intensity / r_2) * std::pow(std::max(0.0f, normal.normalized().dot(h)), p);
        
        // 将环境光、漫反射光、镜面反射光累加到最终颜色中
        result_color += (ambient + diffuse + specular);        
    }

    return result_color * 255.f;
}

// 纹理着色器
Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload) {
    // 关键部分：如果纹理不为空，则获取纹理颜色
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        return_color = payload.texture->getColor(payload.tex_coords.x(), payload.tex_coords.y()); 
    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    // 环境光系数（与phong模型中的环境光系数相同）
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    // 漫反射光系数，该系数就是当前像素的颜色，如何是纹理着色器，则该系数就是纹理的颜色
    // 也就是直接用纹理图片的颜色作为漫反射光的颜色，如果用phong模型，则就需要使用插值出来的颜色
    Eigen::Vector3f kd = texture_color / 255.f;
    // 镜面反射光系数（与phong模型中的镜面反射光系数相同）
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 0};

    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        auto v = eye_pos - point; //v为出射光方向（指向眼睛）
        auto l = light.position - point; //l为指向入射光源方向
        auto h = (v.normalized() + l.normalized()).normalized(); //h为半程向量即v+l归一化后的单位向量
        auto r = l.dot(l); //衰减因子
        auto ambient = ka.cwiseProduct(amb_light_intensity);
        auto diffuse = kd.cwiseProduct(light.intensity / r) * std::max(0.0f, normal.normalized().dot(l.normalized()));
        auto specular = ks.cwiseProduct(light.intensity / r) * std::pow(std::max(0.0f, normal.normalized().dot(h)), p);
        result_color += (ambient + diffuse + specular);
    }

    return result_color * 255.f;
}

// 可视化凹凸向量（凹凸贴图）
Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
    // 环境光系数，与phong模型中的环境光系数相同
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    // 漫反射光系数，与phong模型中的漫反射光系数相同
    Eigen::Vector3f kd = payload.color;
    // 镜面反射光系数，与phong模型中的镜面反射光系数相同
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    // 光源，光源位置和光源强度
    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    // 光源列表
    std::vector<light> lights = {l1, l2};
    // 环境光强度
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    // 观察点位置，因为这里的观察点位置已经在原点了，相机通过视图矩阵将观察点位置移动到原点，所以这里设置为原点。
    Eigen::Vector3f eye_pos{0, 0, 0};

    // 高光系数
    float p = 150;

    // 当前像素的颜色
    Eigen::Vector3f color = payload.color; 
    // 当前像素的三维空间位置
    Eigen::Vector3f point = payload.view_pos;
    // 当前像素的法线
    Eigen::Vector3f normal = payload.normal;


    float kh = 0.2, kn = 0.1;

    // TODO: Implement bump mapping here 
    // 计算凹凸向量流程：
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Normal n = normalize(TBN * ln)

    // 这一部分都是上面计算凹凸向量的流程：
    float x = normal.x();
    float y = normal.y();
    float z = normal.z();

    Eigen::Vector3f t = Eigen::Vector3f(x * y / std::sqrt(x * x + z * z), std::sqrt(x * x + z * z), z * y / std::sqrt(x * x + z * z));
    Eigen::Vector3f b = normal.cross(t);

    Eigen::Matrix3f TBN;
    TBN <<  t.x(), b.x(), normal.x(),
            t.y(), b.y(), normal.y(),
            t.z(), b.z(), normal.z();

    float u = payload.tex_coords.x();
    float v = payload.tex_coords.y();
    float w = payload.texture->width;
    float h = payload.texture->height;

    // float dU = kh * kn * (payload.texture->getColor(u + 1.0f / w, v).norm() - payload.texture->getColor(u, v).norm());
    // float dV = kh * kn * (payload.texture->getColor(u, v + 1.0f / h).norm() - payload.texture->getColor(u, v).norm());
    // 计算du时确保不会超出纹理边界
    // 计算dv时确保不会超出纹理边界
    float dU;
    if (u + 1.0f / w >= 1.0f) {
        // 如果即将超出右边界，使用向左的差分
        dU = kh * kn * (payload.texture->getColor(u, v).norm() - payload.texture->getColor(u - 1.0f / w, v).norm());
    } else {
        dU = kh * kn * (payload.texture->getColor(u + 1.0f / w, v).norm() - payload.texture->getColor(u, v).norm());
    }

    // 计算dv时确保不会超出纹理边界
    float dV;
    if (v + 1.0f / h >= 1.0f) {
        // 如果即将超出上边界，使用向下的差分
        dV = kh * kn * (payload.texture->getColor(u, v).norm() - payload.texture->getColor(u, v - 1.0f / h).norm());
    } else {
        dV = kh * kn * (payload.texture->getColor(u, v + 1.0f / h).norm() - payload.texture->getColor(u, v).norm());
    }

    Eigen::Vector3f ln = Eigen::Vector3f(-dU, -dV, 1.0f);
    normal = TBN * ln;
    // ============================================

    // 将计算出来的法线颜色赋值给最终颜色
    Eigen::Vector3f result_color = {0, 0, 0};
    result_color = normal.normalized();

    return result_color * 255.f;
}

// 位移模型着色器
Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 0};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;
    
    // TODO: Implement displacement mapping here
    // 计算凹凸向量流程：
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Position p = p + kn * n * h(u,v)
    // Normal n = normalize(TBN * ln)

    // 这一部分都是上面计算凹凸向量的流程：
    float x = normal.x();
    float y = normal.y();
    float z = normal.z();

    Eigen::Vector3f t = Eigen::Vector3f(x * y / std::sqrt(x * x + z * z), std::sqrt(x * x + z * z), z * y / std::sqrt(x * x + z * z));
    Eigen::Vector3f b = normal.cross(t);

    Eigen::Matrix3f TBN;
    TBN <<
        t.x(), b.x(), normal.x(),
            t.y(), b.y(), normal.y(),
            t.z(), b.z(), normal.z();

    float u = payload.tex_coords.x();
    float v = payload.tex_coords.y();
    float w = payload.texture->width;
    float h = payload.texture->height;

    // float dU = kh * kn * (payload.texture->getColor(u + 1.0f / w, v).norm() - payload.texture->getColor(u, v).norm());
    // float dV = kh * kn * (payload.texture->getColor(u, v + 1.0f / h).norm() - payload.texture->getColor(u, v).norm());
    // 计算du时确保不会超出纹理边界
    // 计算dv时确保不会超出纹理边界
    float dU;
    if (u + 1.0f / w >= 1.0f) {
        // 如果即将超出右边界，使用向左的差分
        dU = kh * kn * (payload.texture->getColor(u, v).norm() - payload.texture->getColor(u - 1.0f / w, v).norm());
    } else {
        dU = kh * kn * (payload.texture->getColor(u + 1.0f / w, v).norm() - payload.texture->getColor(u, v).norm());
    }

    // 计算dv时确保不会超出纹理边界
    float dV;
    if (v + 1.0f / h >= 1.0f) {
        // 如果即将超出上边界，使用向下的差分
        dV = kh * kn * (payload.texture->getColor(u, v).norm() - payload.texture->getColor(u, v - 1.0f / h).norm());
    } else {
        dV = kh * kn * (payload.texture->getColor(u, v + 1.0f / h).norm() - payload.texture->getColor(u, v).norm());
    }

    Eigen::Vector3f ln = Eigen::Vector3f(-dU, -dV, 1.0f);

    // 计算点是如何移动的
    point += (kn * normal * payload.texture->getColor(u, v).norm());

    // 计算新的法线
    normal = (TBN * ln).normalized();
    // ============================================

    Eigen::Vector3f result_color = {0, 0, 0};

    // 根据新的点，新的法线，计算新的颜色
    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.

        // 计算光源方向
        Eigen::Vector3f light_dir = (light.position - point).normalized();
        // 计算观察方向
        Eigen::Vector3f view_dir = (eye_pos - point).normalized();
        // 计算半程向量
        Eigen::Vector3f half_vector = (light_dir + view_dir).normalized();

        // 距离衰减
        float r2 = (light.position - point).dot(light.position - point);

        //环境光
        //cwiseProduct()：矩阵点对点相乘
        Eigen::Vector3f La = ka.cwiseProduct(amb_light_intensity);

        //漫反射
        Eigen::Vector3f Ld = kd.cwiseProduct(light.intensity / r2);
        Ld *= std::max(0.0f, normal.normalized().dot(light_dir));

        //高光
        Eigen::Vector3f Ls = ks.cwiseProduct(light.intensity / r2);
        Ls *= std::pow(std::max(0.0f, normal.normalized().dot(half_vector)), p);

        result_color += (La + Ld + Ls);

    }

    return result_color * 255.f;
}

// 位移模型着色器，使用高度贴图和纹理，返回颜色
Eigen::Vector3f displacement_texture_fragment_shader(const fragment_shader_payload& payload)
{
    // 关键部分：初始化光栅器中默认设置的是一个凹凸纹理。
    // 所以，我们需要获取另外的一个纹理，来获取颜色。
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        return_color = payload.texture->getSpotTextureColor(payload.tex_coords.x(), payload.tex_coords.y()); 
    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    // 环境光系数（与phong模型中的环境光系数相同）
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    // 漫反射光系数，该系数就是当前像素的颜色，如何是纹理着色器，则该系数就是纹理的颜色
    // 也就是直接用纹理图片的颜色作为漫反射光的颜色，如果用phong模型，则就需要使用插值出来的颜色
    Eigen::Vector3f kd = texture_color / 255.f;
    // 镜面反射光系数（与phong模型中的镜面反射光系数相同）
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 0};

    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;
    
    // TODO: Implement displacement mapping here
    // 计算凹凸向量流程：
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Position p = p + kn * n * h(u,v)
    // Normal n = normalize(TBN * ln)

    // 这一部分都是上面计算凹凸向量的流程：
    float x = normal.x();
    float y = normal.y();
    float z = normal.z();

    Eigen::Vector3f t = Eigen::Vector3f(x * y / std::sqrt(x * x + z * z), std::sqrt(x * x + z * z), z * y / std::sqrt(x * x + z * z));
    Eigen::Vector3f b = normal.cross(t);

    Eigen::Matrix3f TBN;
    TBN <<
        t.x(), b.x(), normal.x(),
            t.y(), b.y(), normal.y(),
            t.z(), b.z(), normal.z();

    float u = payload.tex_coords.x();
    float v = payload.tex_coords.y();
    float w = payload.texture->width;
    float h = payload.texture->height;

    // float dU = kh * kn * (payload.texture->getColor(u + 1.0f / w, v).norm() - payload.texture->getColor(u, v).norm());
    // float dV = kh * kn * (payload.texture->getColor(u, v + 1.0f / h).norm() - payload.texture->getColor(u, v).norm());
    // 计算du时确保不会超出纹理边界
    // 计算dv时确保不会超出纹理边界
    float dU;
    if (u + 1.0f / w >= 1.0f) {
        // 如果即将超出右边界，使用向左的差分
        dU = kh * kn * (payload.texture->getColor(u, v).norm() - payload.texture->getColor(u - 1.0f / w, v).norm());
    } else {
        dU = kh * kn * (payload.texture->getColor(u + 1.0f / w, v).norm() - payload.texture->getColor(u, v).norm());
    }

    // 计算dv时确保不会超出纹理边界
    float dV;
    if (v + 1.0f / h >= 1.0f) {
        // 如果即将超出上边界，使用向下的差分
        dV = kh * kn * (payload.texture->getColor(u, v).norm() - payload.texture->getColor(u, v - 1.0f / h).norm());
    } else {
        dV = kh * kn * (payload.texture->getColor(u, v + 1.0f / h).norm() - payload.texture->getColor(u, v).norm());
    }

    Eigen::Vector3f ln = Eigen::Vector3f(-dU, -dV, 1.0f);

    // 计算点是如何移动的
    // point += (kn * normal * payload.texture->getColor(u, v).norm());

    // 计算新的法线
    normal = (TBN * ln).normalized();
    // ============================================

    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        auto v = eye_pos - point; //v为出射光方向（指向眼睛）
        auto l = light.position - point; //l为指向入射光源方向
        auto h = (v.normalized() + l.normalized()).normalized(); //h为半程向量即v+l归一化后的单位向量
        auto r = l.dot(l); //衰减因子
        auto ambient = ka.cwiseProduct(amb_light_intensity);
        auto diffuse = kd.cwiseProduct(light.intensity / r) * std::max(0.0f, normal.normalized().dot(l.normalized()));
        auto specular = ks.cwiseProduct(light.intensity / r) * std::pow(std::max(0.0f, normal.normalized().dot(h)), p);
        result_color += (ambient + diffuse + specular);
    }

    return result_color * 255.f;
}

// 主函数
int main(int argc, const char** argv) {
    // 创建一个三角形列表，元素是一个指针，指向一个三角形对象
    // 列表总存的是地址，意味着 TriangleList 中的每个元素都是一个指向 Triangle 对象的地址。
    std::vector<Triangle*> TriangleList;

    float angle = 140.0;
    bool command_line = false;

    std::string filename = "output.png"; 
    objl::Loader Loader; //第三方库用于加载模型。
    std::string obj_path = "../models/spot/";

    // Load .obj File 加载模型
    Loader.LoadFile("../models/spot/spot_triangulated_good.obj");
    // 循环模型中的每一个三角形，并添加到三角形列表中。
    for(auto mesh:Loader.LoadedMeshes) {
        for(int i=0;i<mesh.Vertices.size();i+=3) {
            auto* t = new Triangle(); // 三角形对象指针
            for(int j=0;j<3;j++) {
                // 将三角形对应的数据加载进来
                // 设置顶点信息
                t->setVertex(j, Vector4f(mesh.Vertices[i + j].Position.X, mesh.Vertices[i + j].Position.Y,
                                         mesh.Vertices[i + j].Position.Z, 1.0));
                // 设置法线信息
                t->setNormal(j, Vector3f(mesh.Vertices[i + j].Normal.X, mesh.Vertices[i + j].Normal.Y,
                                         mesh.Vertices[i + j].Normal.Z));
                // 设置纹理坐标
                t->setTexCoord(j, Vector2f(mesh.Vertices[i + j].TextureCoordinate.X,
                                           mesh.Vertices[i + j].TextureCoordinate.Y));
            }
            // 将三角形添加到三角形列表中
            TriangleList.push_back(t);
        }
    }

    // 初始化光栅器，并设置光栅器的大小
    rst::rasterizer r(700, 700);

    // 纹理图片的路径，默认使用的是凹凸贴图的纹理，是用来更改法线贴图的。
    auto texture_path = "hmap.jpg";
    // 设置纹理
    r.set_texture(Texture(obj_path + texture_path));

    // 设置着色器，默认使用phong着色器，如果命令行参数有2个，则使用命令行参数中对应的着色器
    // 这里设置的是光栅器中的着色器，在光栅器中，着色器是根据当前像素对应的信息计算出着色点颜色
    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = phong_fragment_shader;

    // 如果命令行参数有2个，则使用命令行参数
    if (argc >= 2) {
        command_line = true;
        // 获取命令行参数，获取图片的名称
        filename = std::string(argv[1]);
        // 如果命令行参数有3个，并且第二个参数是texture，则使用纹理着色器
        // 纹理模型
        if (argc == 3 && std::string(argv[2]) == "texture") {   
            // 设置纹理着色器
            std::cout << "Rasterizing using the texture shader\n";
            active_shader = texture_fragment_shader;
            // 设置纹理图片的路径
            texture_path = "spot_texture.png";
            // 设置纹理，纹理图像是直接设置在光栅器中的
            r.set_texture(Texture(obj_path + texture_path));
        } else if (argc == 3 && std::string(argv[2]) == "normal") { // 法线模型
            std::cout << "Rasterizing using the normal shader\n";
            active_shader = normal_fragment_shader;
        } else if (argc == 3 && std::string(argv[2]) == "phong") { // 高光phong模型
            std::cout << "Rasterizing using the phong shader\n";
            active_shader = phong_fragment_shader;
        } else if (argc == 3 && std::string(argv[2]) == "bump") { // 凹凸模型，使用法线贴图
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = bump_fragment_shader;
        } else if (argc == 3 && std::string(argv[2]) == "displacement") { // 位移模型，使用高度贴图
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = displacement_fragment_shader;
        } else if (argc == 3 && std::string(argv[2]) == "displacement_texture") { // 位移模型，使用高度贴图和纹理
            std::cout << "Rasterizing using the displacement_texture shader\n";
            active_shader = displacement_texture_fragment_shader;
        }

    }

    // 设置相机位置
    Eigen::Vector3f eye_pos = {0, 0, 10};

    // 设置顶点着色器，将顶点着色器传给光栅器
    r.set_vertex_shader(vertex_shader);
    // 设置片段着色器，将片段着色器传给光栅器
    r.set_fragment_shader(active_shader);

    int key = 0;
    int frame_count = 0;

    // 如果命令行参数有2个，command_line就为true，进入命令行模式
    if (command_line) {
        // 清除颜色和深度缓冲区
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        // 设置模型矩阵
        r.set_model(get_model_matrix(angle));
        // 设置视图矩阵
        r.set_view(get_view_matrix(eye_pos));
        // 设置投影矩阵
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        // 绘制三角形
        r.draw(TriangleList);
        // 获取光栅器中的颜色缓冲区
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        // 将颜色缓冲区转换为8位无符号整数类型
        image.convertTo(image, CV_8UC3, 1.0f);
        // 将颜色缓冲区转换为BGR格式
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        // 将颜色缓冲区保存为图片
        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);
        cv::imwrite(filename, image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';
        
        if (key == 'a' ) {
            angle -= 0.1;
        } else if (key == 'd') {
            angle += 0.1;
        }

    }
    return 0;
}
