#pragma once

#include "Vector.hpp"
#include "global.hpp"

class Object
{
public:
    // Sphere 和 MeshTriangle的父类
    Object()
        : materialType(DIFFUSE_AND_GLOSSY) // 材质类型
        , ior(1.3) // 折射率
        , Kd(0.8) // Kd漫反射系数
        , Ks(0.2) // Ks镜面反射系数
        , diffuseColor(0.2) // 漫反射颜色
        , specularExponent(25) // 光泽度，高光项里的幂
    {}

    virtual ~Object() = default;

    // 判断光线是否与物体相交
    // orig是光线原点，dir是光线方向，tNear是光线与物体相交的距离，index是物体索引，uv是物体纹理坐标
    virtual bool intersect(const Vector3f&, const Vector3f&, float&, uint32_t&, Vector2f&) const = 0;

    // 获取物体表面属性
    virtual void getSurfaceProperties(const Vector3f&, const Vector3f&, const uint32_t&, const Vector2f&, Vector3f&,
                                      Vector2f&) const = 0;

    virtual Vector3f evalDiffuseColor(const Vector2f&) const
    {
        return diffuseColor;
    }

    // material properties
    MaterialType materialType;
    float ior;
    float Kd, Ks;
    Vector3f diffuseColor;
    float specularExponent;
};
