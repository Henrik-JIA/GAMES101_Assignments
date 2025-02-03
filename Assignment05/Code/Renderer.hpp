#pragma once
#include "Scene.hpp"

// 碰撞信息
// tNear是光线与物体相交的距离
// index是物体索引
// uv存储相交点的u和v重心坐标
// *hit_obj是存储指向相交物体的指针（用于存储材质信息等）
struct hit_payload
{
    float tNear;
    uint32_t index;
    Vector2f uv;
    Object* hit_obj;
};

class Renderer
{
public:
    void Render(const Scene& scene);

private:
};