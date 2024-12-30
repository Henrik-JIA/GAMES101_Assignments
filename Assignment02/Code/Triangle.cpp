//
// Created by LEI XU on 4/11/19.
//

#include "Triangle.hpp"
#include <algorithm>
#include <array>


Triangle::Triangle() {
    v[0] << 0,0,0;
    v[1] << 0,0,0;
    v[2] << 0,0,0;

    color[0] << 0.0, 0.0, 0.0;
    color[1] << 0.0, 0.0, 0.0;
    color[2] << 0.0, 0.0, 0.0;

    tex_coords[0] << 0.0, 0.0;
    tex_coords[1] << 0.0, 0.0;
    tex_coords[2] << 0.0, 0.0;
}

// 设置三角形顶点坐标
void Triangle::setVertex(int ind, Vector3f ver){
    v[ind] = ver;
}

// 设置三角形顶点法向量
void Triangle::setNormal(int ind, Vector3f n){
    normal[ind] = n;
}

// 设置三角形顶点颜色
void Triangle::setColor(int ind, float r, float g, float b) {
    if((r<0.0) || (r>255.) ||
       (g<0.0) || (g>255.) ||
       (b<0.0) || (b>255.)) {
        fprintf(stderr, "ERROR! Invalid color values");
        fflush(stderr);
        exit(-1);
    }
    // 将颜色值归一化到[0,1]
    color[ind] = Vector3f((float)r/255.,(float)g/255.,(float)b/255.);
    return;
}
void Triangle::setTexCoord(int ind, float s, float t) {
    tex_coords[ind] = Vector2f(s,t);
}

std::array<Vector4f, 3> Triangle::toVector4() const
{
    std::array<Eigen::Vector4f, 3> res;
    // 因为之前只存入了x,y,z值，所以这里需要将w值设置为1.0f，这样齐次坐标就表示一个点。如果w分量是0，则表示一个向量。
    std::transform(std::begin(v), std::end(v), res.begin(), [](auto& vec) { return Eigen::Vector4f(vec.x(), vec.y(), vec.z(), 1.f); });
    return res;
}
