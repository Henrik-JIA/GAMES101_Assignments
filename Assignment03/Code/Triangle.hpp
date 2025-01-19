//
// Created by LEI XU on 4/11/19.
//

#ifndef RASTERIZER_TRIANGLE_H
#define RASTERIZER_TRIANGLE_H

#include <Eigen/Eigen>
#include "Texture.hpp"

using namespace Eigen;
class Triangle{

public:
    // 三角形的三个顶点
    Vector4f v[3]; /*the original coordinates of the triangle, v0, v1, v2 in counter clockwise order*/
    /*Per vertex values*/
    // 每个顶点的颜色
    Vector3f color[3]; //color at each vertex;
    // 每个顶点的纹理坐标
    Vector2f tex_coords[3]; //texture u,v
    // 每个顶点的法线
    Vector3f normal[3]; //normal vector for each vertex
    // 空的纹理指针
    Texture *tex= nullptr;

    // 构造函数
    Triangle();

    // 获取三角形的三个顶点
    Eigen::Vector4f a() const { return v[0]; }
    Eigen::Vector4f b() const { return v[1]; }
    Eigen::Vector4f c() const { return v[2]; }

    void setVertex(int ind, Vector4f ver); /*set i-th vertex coordinates */
    void setNormal(int ind, Vector3f n); /*set i-th vertex normal vector*/
    void setColor(int ind, float r, float g, float b); /*set i-th vertex color*/

    void setNormals(const std::array<Vector3f, 3>& normals);
    void setColors(const std::array<Vector3f, 3>& colors);
    void setTexCoord(int ind,Vector2f uv ); /*set i-th vertex texture coordinate*/
    std::array<Vector4f, 3> toVector4() const;
};






#endif //RASTERIZER_TRIANGLE_H
