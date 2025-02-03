#pragma once

#include "Object.hpp"
#include "Vector.hpp"

// 球类Sphere继承父类Object
class Sphere : public Object
{
public:
    // Sphere的构造函数，这里是显示曲面，判断光线是否与其相交比较容易。
    // 传入球心位置和半径
    Sphere(const Vector3f& c, const float& r)
        : center(c)
        , radius(r)
        , radius2(r * r)
    {}

    // 判断光线是否与球体相交
    // 重写父类Object的intersect函数
    // orig是光线原点，dir是光线方向，tnear是光线与物体相交的距离，index是物体索引，uv是物体纹理坐标
    bool intersect(const Vector3f& orig, const Vector3f& dir, float& tnear, uint32_t&, Vector2f&) const override
    {
        // analytic solution
        // 详见lecture13 ppt 23页。
        Vector3f L = orig - center;
        float a = dotProduct(dir, dir);
        float b = 2 * dotProduct(dir, L);
        float c = dotProduct(L, L) - radius2;
        float t0, t1;
        if (!solveQuadratic(a, b, c, t0, t1))
            return false;
        if (t0 < 0)
            t0 = t1;
        if (t0 < 0)
            return false;
        
        // 返回最近的
        tnear = t0;

        return true;
    }

    // 重写父类Object的getSurfaceProperties函数
    // P是碰撞点，dir是光线方向，index是物体索引，uv是物体纹理坐标
    // N是法向量，st是纹理坐标
    // 只需计算法向量
    void getSurfaceProperties(const Vector3f& P, const Vector3f&, const uint32_t&, const Vector2f&,
                              Vector3f& N, Vector2f&) const override
    {
        // 这里的法线是从球心指向碰撞点，并进行归一化
        N = normalize(P - center);
    }

    Vector3f center;
    float radius, radius2;
};
