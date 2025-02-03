#include "Scene.hpp"
#include "Sphere.hpp"
#include "Triangle.hpp"
#include "Light.hpp"
#include "Renderer.hpp"

// In the main function of the program, we create the scene (create objects and lights)
// as well as set the options for the render (image width and height, maximum recursion
// depth, field-of-view, etc.). We then call the render function().
int main()
{
    // 设置一个1280*960的分辨率场景
    Scene scene(1280, 960);

    // 创建一个球体，该球继承object类，这个td::make_unique<Sphere>是一个智能指针
    auto sph1 = std::make_unique<Sphere>(Vector3f(-1, 0, -12), 2); // 球体的位置，半径
    // 更新球体的材质类型为漫反射和光泽模型
    sph1->materialType = DIFFUSE_AND_GLOSSY; 
    // 更新球体的漫反射颜色
    sph1->diffuseColor = Vector3f(0.6, 0.7, 0.8); 

    // 创建第二个球体
    auto sph2 = std::make_unique<Sphere>(Vector3f(0.5, -0.5, -8), 1.5);
    // 更新球体的折射率
    sph2->ior = 1.5;
    // 更新球体的材质类型为反射和折射模型
    sph2->materialType = REFLECTION_AND_REFRACTION;

    // 将物体加入到场景中，std::move类似于引用，减少传值的开销。
    // 这里球是继承了Object类，所以这里是用于将物体添加到场景中的。
    scene.Add(std::move(sph1));
    scene.Add(std::move(sph2));

    // 通过三角形来构建一个矩形
    // 定义了两个三角形，由于有公用顶点的存在，只有四个三维顶点
    Vector3f verts[4] = {{-5,-3,-6}, {5,-3,-6}, {5,-3,-16}, {-5,-3,-16}};
    // 定义了四个纹理坐标，用于描述三角形的顶点在纹理空间中的位置，四个点位置
    Vector2f st[4] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
    // vertIndex来解释哪些顶点组成一个三角形，此处为[0, 1, 3]和[1, 2, 3]
    uint32_t vertIndex[6] = {0, 1, 3, 1, 2, 3};

    // 创建一堆三角形，std::make_unique<>这个是指针，MeshTriangle通过这个来实例化
    auto mesh = std::make_unique<MeshTriangle>(verts, vertIndex, 2, st);
    // 更新三角形的材质类型为漫反射和光泽模型
    mesh->materialType = DIFFUSE_AND_GLOSSY;

    // 这里三角形是继承了Object类，所以这里是用于将物体添加到场景中的。
    scene.Add(std::move(mesh));

    // 这里将光源添加进来，分别设置光源位置于光强intensity
    scene.Add(std::make_unique<Light>(Vector3f(-20, 70, 20), 0.5));
    scene.Add(std::make_unique<Light>(Vector3f(30, 50, -12), 0.5));    

    // 准备工作就绪，开始渲染
    Renderer r;
    // 调用渲染函数
    r.Render(scene);

    return 0;
}