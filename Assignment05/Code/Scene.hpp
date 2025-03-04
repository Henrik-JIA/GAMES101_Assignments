#pragma once

#include <vector>
#include <memory>
#include "Vector.hpp"
#include "Object.hpp"
#include "Light.hpp"

class Scene
{
public:
    // setting up options
    int width = 1280;
    int height = 960;
    // 可视角度默认90°
    double fov = 90; 
    // 背景颜色 RGB格式，天蓝色
    Vector3f backgroundColor = Vector3f(0.235294, 0.67451, 0.843137);
    // 最大递归深度
    int maxDepth = 5;
    // 这个偏移量的目的是避免 反射光 或者 折射光 与命中点所在的表面相交。
    float epsilon = 0.00001;

    // 构造函数
    Scene(int w, int h) : width(w), height(h){}

    // 将物体添加到场景中
    void Add(std::unique_ptr<Object> object) { objects.push_back(std::move(object)); }
    // 将光源添加到场景中
    void Add(std::unique_ptr<Light> light) { lights.push_back(std::move(light)); }

    [[nodiscard]] const std::vector<std::unique_ptr<Object> >& get_objects() const { return objects; }
    [[nodiscard]] const std::vector<std::unique_ptr<Light> >&  get_lights() const { return lights; }

private:
    // creating the scene (adding objects and lights)
    // 物体数组
    std::vector<std::unique_ptr<Object> > objects;
    // 光源数组
    std::vector<std::unique_ptr<Light> > lights;
};