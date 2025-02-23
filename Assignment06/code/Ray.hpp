//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_RAY_H
#define RAYTRACING_RAY_H
#include "Vector.hpp"
struct Ray{
    //Destination = origin + t * direction
    Vector3f origin; // 起点
    Vector3f direction; // 方向向量
    Vector3f direction_inv; // 倒数

    double t;   //transportation time,
    double t_min, t_max;

    // 构造函数
    Ray(const Vector3f& ori, const Vector3f& dir, const double _t = 0.0): origin(ori), direction(dir), t(_t)
    {
        // 预计算方向倒数用于快速AABB相交检测
        direction_inv = Vector3f(1.f/direction.x, 1.f/direction.y, 1.f/direction.z);
        t_min = 0.0;
        t_max = std::numeric_limits<double>::max();

    }

    // 计算光线在t时刻的点
    Vector3f operator()(double t) const{
        return origin + direction * t; //光线方程，origin是起点，direction是方向，t是时间。
    }

    // 重载输出运算符
    friend std::ostream &operator<<(std::ostream& os, const Ray& r){
        // 输出光线信息
        os<<"[origin:="<<r.origin<<", direction="<<r.direction<<", time="<< r.t<<"]\n";
        return os;
    }
};
#endif //RAYTRACING_RAY_H
