//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "Vector.hpp"

// 材质类型
enum MaterialType { DIFFUSE_AND_GLOSSY, REFLECTION_AND_REFRACTION, REFLECTION };

// 材质类
/*
参数：
    t: 材质类型
    c: 材质颜色
    e: 材质发射的颜色
*/
class Material{
public:
    MaterialType m_type;
    Vector3f m_color;
    Vector3f m_emission;
    float ior;
    float Kd, Ks;
    float specularExponent;
    //Texture tex;

    inline Material(MaterialType t=DIFFUSE_AND_GLOSSY, Vector3f c=Vector3f(1,1,1), Vector3f e=Vector3f(0,0,0));
    inline MaterialType getType();
    inline Vector3f getColor();
    inline Vector3f getColorAt(double u, double v);
    inline Vector3f getEmission();

};

// 构造函数
// 参数：
// t: 材质类型
// c: 材质颜色
// e: 材质发射的颜色
Material::Material(MaterialType t, Vector3f c, Vector3f e){
    m_type = t;
    m_color = c;
    m_emission = e;
}

// 获取材质类型
MaterialType Material::getType(){return m_type;}

// 获取材质颜色
Vector3f Material::getColor(){return m_color;}

// 获取材质发射的颜色
Vector3f Material::getEmission() {return m_emission;}

// 获取材质颜色在(u, v)处的颜色
Vector3f Material::getColorAt(double u, double v) {
    return Vector3f();
}
#endif //RAYTRACING_MATERIAL_H
