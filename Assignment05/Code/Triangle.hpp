#pragma once

#include "Object.hpp"

#include <cstring>

bool rayTriangleIntersect(const Vector3f& v0, const Vector3f& v1, const Vector3f& v2, const Vector3f& orig,
                          const Vector3f& dir, float& tnear, float& u, float& v)
{
    // TODO: Implement this function that tests whether the triangle
    // that's specified bt v0, v1 and v2 intersects with the ray (whose
    // origin is *orig* and direction is *dir*)
    // Also don't forget to update tnear, u and v.
    // 判断光线是否与三角形相交
    // 详见lecture13 ppt 29页
    Vector3f e1 = v1 - v0;
    Vector3f e2 = v2 - v0;
    Vector3f s0 = orig - v0;
    Vector3f s1 = crossProduct(dir, e2);
    Vector3f s2 = crossProduct(s0, e1);

    Vector3f s = Vector3f(dotProduct(s2, e2),dotProduct(s1, s0),dotProduct(s2, dir))/dotProduct(s1, e1);
    tnear = s.x;
    u = s.y;
    v = s.z;

    if(tnear >= 0 && u >= 0 && v >= 0 && (u + v) <= 1)
    {
        return true;
    }

    return false;
}

class MeshTriangle : public Object
{
public:
    // 构造函数，传入顶点，顶点索引，三角形数量，纹理坐标
    MeshTriangle(const Vector3f* verts, const uint32_t* vertsIndex, const uint32_t& numTris, const Vector2f* st)
    {
        // 这一段代码是在找vertsIndex中的最大值，用于确定顶点的数量
        // 根据maxIndex存的最大值，这样便于在动态开辟数组时少一些消耗。
        uint32_t maxIndex = 0;
        for (uint32_t i = 0; i < numTris * 3; ++i)
            if (vertsIndex[i] > maxIndex)
                maxIndex = vertsIndex[i];
        maxIndex += 1;

        
        // 创建了一个指针，指向Vector3f类型的数组的智能指针，并分配了maxIndex个元素的内存空间。
        vertices = std::unique_ptr<Vector3f[]>(new Vector3f[maxIndex]);
        // 将verts拷贝到新创建的vertices中
        // vertices.get()返回的是指向分配的内存块的原始指针，即数组的起始地址。
        memcpy(vertices.get(), verts, sizeof(Vector3f) * maxIndex);

        // 创建了一个指针，指向uint32_t类型的数组的智能指针，并分配了numTris * 3个元素的内存空间。
        vertexIndex = std::unique_ptr<uint32_t[]>(new uint32_t[numTris * 3]);
        // 将vertsIndex拷贝到新创建的vertexIndex中
        memcpy(vertexIndex.get(), vertsIndex, sizeof(uint32_t) * numTris * 3);

        // 将三角形数目拷贝进来
        numTriangles = numTris;
        
        // 将纹理坐标拷贝进来
        stCoordinates = std::unique_ptr<Vector2f[]>(new Vector2f[maxIndex]);
        memcpy(stCoordinates.get(), st, sizeof(Vector2f) * maxIndex);
    }

    // 判断光线是否与一堆三角形相交
    bool intersect(const Vector3f& orig, const Vector3f& dir, float& tnear, uint32_t& index,
                   Vector2f& uv) const override
    {
        bool intersect = false;
        // 遍历所有三角形
        for (uint32_t k = 0; k < numTriangles; ++k)
        {
            // 取得三角形的顶点坐标
            const Vector3f& v0 = vertices[vertexIndex[k * 3]];
            const Vector3f& v1 = vertices[vertexIndex[k * 3 + 1]];
            const Vector3f& v2 = vertices[vertexIndex[k * 3 + 2]];
            float t, u, v;
            
            // 判断光线是否与单个三角形相交
            // 因为有多个三角形，所以需要找到最近的三角形
            if (rayTriangleIntersect(v0, v1, v2, orig, dir, t, u, v) && t < tnear)
            {
                tnear = t;
                uv.x = u;
                uv.y = v;
                index = k;
                intersect |= true;
            }
        }

        return intersect;
    }

    // 重写父类Object的getSurfaceProperties函数
    // P是碰撞点，dir是光线方向，index是物体索引，uv是物体纹理坐标
    // N是法向量，st是纹理坐标
    // 这里传入的index表示光线与物体相交的三角形的索引
    // 计算法向量和纹理坐标
    // 碰撞点在物体表面的其他信息，如法线，三角形中的纹理坐标。
    void getSurfaceProperties(const Vector3f&, const Vector3f&, const uint32_t& index, const Vector2f& uv, Vector3f& N,
                              Vector2f& st) const override
    {
        const Vector3f& v0 = vertices[vertexIndex[index * 3]];
        const Vector3f& v1 = vertices[vertexIndex[index * 3 + 1]];
        const Vector3f& v2 = vertices[vertexIndex[index * 3 + 2]];

        // 计算两个边向量
        Vector3f e0 = normalize(v1 - v0);
        Vector3f e1 = normalize(v2 - v1);

        // 计算法向量，边向量叉乘
        N = normalize(crossProduct(e0, e1));

        // 计算纹理坐标
        const Vector2f& st0 = stCoordinates[vertexIndex[index * 3]];
        const Vector2f& st1 = stCoordinates[vertexIndex[index * 3 + 1]];
        const Vector2f& st2 = stCoordinates[vertexIndex[index * 3 + 2]];

        // 纹理坐标重心插值
        st = st0 * (1 - uv.x - uv.y) + st1 * uv.x + st2 * uv.y;
    }

    // 重写父类Object的evalDiffuseColor函数
    // 返回真实的纹理颜色
    Vector3f evalDiffuseColor(const Vector2f& st) const override
    {
        // 控制棋盘的密度
        float scale = 5;

        // 检测 st该点落的位置的颜色，非0即1。
        // 非常巧妙的实现了棋盘
        // 这里画个[0,1]^2的纹理图就明白了
        // st.x，st.y对应纹理图的坐标
        // " ^ "使用异或符号，0 ^ 0 = 0；0 ^ 1 = 1，也就是两个相同得0；两个不同得1。
        // fmodf函数取余数，st.x * scale，st.y * scale，取余数，然后比较大小。
        float pattern = (fmodf(st.x * scale, 1) > 0.5) ^ (fmodf(st.y * scale, 1) > 0.5);

        // pattern非0即1，其实这里没必要用线性插值函数来做。
        return lerp(Vector3f(0.815, 0.235, 0.031), Vector3f(0.937, 0.937, 0.231), pattern);
    }

    std::unique_ptr<Vector3f[]> vertices;
    uint32_t numTriangles;
    std::unique_ptr<uint32_t[]> vertexIndex;
    std::unique_ptr<Vector2f[]> stCoordinates;
};
