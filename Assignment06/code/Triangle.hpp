#pragma once

#include "BVH.hpp"
#include "Intersection.hpp"
#include "Material.hpp"
#include "OBJ_Loader.hpp"
#include "Object.hpp"
#include "Triangle.hpp"
#include <cassert>
#include <array>

// 判断光线是否与三角形相交
// 这个rayTriangleIntersect只是判断光线是否与三角形相交，
// 而inline Intersection Triangle::getIntersection(Ray ray)是获取光线与三角形的交点信息。
// 这两个函数使用的是同一个算法。
// 详见lecture13 ppt 29页
/*
参数：
    v0, v1, v2: 三角形的三个顶点
    orig: 光线的起点
    dir: 光线的方向
    tnear: 最近交点到光线的起点距离
    u, v: 光线与三角形交点的参数
返回值：
    true: 光线与三角形相交
    false: 光线与三角形不相交
也就是求解线性方程组。
*/
bool rayTriangleIntersect(const Vector3f& v0, const Vector3f& v1,
                          const Vector3f& v2, const Vector3f& orig,
                          const Vector3f& dir, float& tnear, float& u, float& v)
{
    Vector3f edge1 = v1 - v0;
    Vector3f edge2 = v2 - v0;

    Vector3f pvec = crossProduct(dir, edge2);
    float det = dotProduct(edge1, pvec);
    if (det == 0 || det < 0) 
        return false;

    Vector3f tvec = orig - v0;
    u = dotProduct(tvec, pvec);
    if (u < 0 || u > det)
        return false;

    Vector3f qvec = crossProduct(tvec, edge1);
    v = dotProduct(dir, qvec);
    if (v < 0 || u + v > det)
        return false;

    float invDet = 1 / det;

    tnear = dotProduct(edge2, qvec) * invDet;
    u *= invDet;
    v *= invDet;

    return true;
}

// 三角形类
/*
参数：
    v0, v1, v2: 三角形的三个顶点
    e1, e2: 三角形的两条边
    t0, t1, t2: 三角形的三个纹理坐标
    normal: 三角形的法向量
    m: 三角形的材质
*/
class Triangle : public Object
{
public:
    Vector3f v0, v1, v2; // vertices A, B ,C , counter-clockwise order
    Vector3f e1, e2;     // 2 edges v1-v0, v2-v0;
    Vector3f t0, t1, t2; // texture coords
    Vector3f normal;
    Material* m;

    // 构造函数
    // 参数：
    // _v0, _v1, _v2: 三角形的三个顶点
    // _m: 三角形的材质
    Triangle(Vector3f _v0, Vector3f _v1, Vector3f _v2, Material* _m = nullptr)
        : v0(_v0), v1(_v1), v2(_v2), m(_m)
    {
        e1 = v1 - v0;
        e2 = v2 - v0;
        normal = normalize(crossProduct(e1, e2));
    }

    // 重载 intersect 函数，用于判断光线是否与三角形相交
    bool intersect(const Ray& ray) override;
    // 重载 intersect 函数，用于判断光线是否与三角形相交，并返回交点信息
    bool intersect(const Ray& ray, float& tnear, uint32_t& index) const override;
    // 重载 getIntersection 函数，用于获取光线与三角形的交点信息
    Intersection getIntersection(Ray ray) override;
    // 重载 getSurfaceProperties 函数，用于获取光线与三角形的交点信息
    void getSurfaceProperties(const Vector3f& P, const Vector3f& I,
                              const uint32_t& index, const Vector2f& uv,
                              Vector3f& N, Vector2f& st) const override
    {
        N = normal;
        //        throw std::runtime_error("triangle::getSurfaceProperties not
        //        implemented.");
    }
    // 重载 evalDiffuseColor 函数，用于获取光线与三角形的交点信息
    Vector3f evalDiffuseColor(const Vector2f&) const override;
    // 重载 getBounds 函数，用于获取三角形的包围盒
    Bounds3 getBounds() override;
};

// 网格三角形类
class MeshTriangle : public Object
{
public:
    // 构造函数
    // 参数：
    // filename: 模型的文件名
    MeshTriangle(const std::string& filename)
    {
        // 利用第三方库加载模型
        objl::Loader loader;
        loader.LoadFile(filename);

        // 断言，确保模型加载成功
        assert(loader.LoadedMeshes.size() == 1);
        // 获取加载的模型
        auto mesh = loader.LoadedMeshes[0];

        // 初始化包围盒
        // min_vert 和 max_vert初始化， 后续计算模型的包围盒
        Vector3f min_vert = Vector3f{std::numeric_limits<float>::infinity(),
                                     std::numeric_limits<float>::infinity(),
                                     std::numeric_limits<float>::infinity()};

        Vector3f max_vert = Vector3f{-std::numeric_limits<float>::infinity(),
                                     -std::numeric_limits<float>::infinity(),
                                     -std::numeric_limits<float>::infinity()};

        // 遍历所有三角形
        for (int i = 0; i < mesh.Vertices.size(); i += 3) {
            // 用来储存一个三角形
            std::array<Vector3f, 3> face_vertices;

            for (int j = 0; j < 3; j++) {
                // 乘以 60.f的目的是对顶点进行缩放。通过将每个顶点的坐标乘以 60.f，可以使模型在渲染时变大。
                // 这种缩放可以用于调整模型的大小，以适应特定的渲染环境或视觉效果。
                auto vert = Vector3f(mesh.Vertices[i + j].Position.X,
                                     mesh.Vertices[i + j].Position.Y,
                                     mesh.Vertices[i + j].Position.Z) * 60.f;

                // 赋值
                face_vertices[j] = vert;

                // 更新模型包围盒
                // 注意：min_vert 和 max_vert并不一定是真实的顶点
                min_vert = Vector3f(std::min(min_vert.x, vert.x),
                                    std::min(min_vert.y, vert.y),
                                    std::min(min_vert.z, vert.z));

                max_vert = Vector3f(std::max(max_vert.x, vert.x),
                                    std::max(max_vert.y, vert.y),
                                    std::max(max_vert.z, vert.z));
            }

            // 三角形的材质信息
            auto new_mat =
                new Material(MaterialType::DIFFUSE_AND_GLOSSY,
                             Vector3f(0.5, 0.5, 0.5), Vector3f(0, 0, 0));
            new_mat -> Kd = 0.6;
            new_mat -> Ks = 0.0;
            new_mat -> specularExponent = 0;

            // 单个三角形放入 triangles中
            // emplace_back函数接受一组参数，用于构造一个新的元素，并将其直接插入容器的尾部
            triangles.emplace_back(face_vertices[0], face_vertices[1],face_vertices[2], new_mat);
        }

        // 遍历完所有三角形，模型的包围盒最小化
        // 后面在场景中建立 BVH树需要用到
        bounding_box = Bounds3(min_vert, max_vert);

        std::vector<Object*> ptrs;
        for (auto& tri : triangles)
            ptrs.push_back(&tri);

        // 对模型内部进行 BVH细分，加速结构实现的重点
        // 虽然 main.cpp后面又一次 scene.buildBVH() 但是它是把模型看作一个 Object，模型内部还需继续细分
        bvh = new BVHAccel(ptrs);
    }

    bool intersect(const Ray& ray) 
    {
        return true; 
    }

    bool intersect(const Ray& ray, float& tnear, uint32_t& index) const
    {
        bool intersect = false;
        for (uint32_t k = 0; k < numTriangles; ++k) {
            const Vector3f& v0 = vertices[vertexIndex[k * 3]];
            const Vector3f& v1 = vertices[vertexIndex[k * 3 + 1]];
            const Vector3f& v2 = vertices[vertexIndex[k * 3 + 2]];
            float t, u, v;
            if (rayTriangleIntersect(v0, v1, v2, ray.origin, ray.direction, t,
                                     u, v) &&
                t < tnear) {
                tnear = t;
                index = k;
                intersect |= true;
            }
        }

        return intersect;
    }

    Bounds3 getBounds() 
    {
         return bounding_box; 
    }

    // 获取光线与三角形的交点信息
    // 这个函数在已知相交后：
    // 1. 使用顶点插值计算法线
    // 2. 插值纹理坐标
    // 不涉及相交检测
    // 参数（所有参数都是引用）：
    // P: 光线与三角形的交点
    // I: 光线方向
    // index: 三角形的索引
    // uv: 交点在三角形上的纹理坐标
    // N: 交点处的法向量
    // st: 交点处的纹理坐标
    void getSurfaceProperties(const Vector3f& P, const Vector3f& I,
                              const uint32_t& index, const Vector2f& uv,
                              Vector3f& N, Vector2f& st) const override
    {
        const Vector3f& v0 = vertices[vertexIndex[index * 3]];
        const Vector3f& v1 = vertices[vertexIndex[index * 3 + 1]];
        const Vector3f& v2 = vertices[vertexIndex[index * 3 + 2]];
        Vector3f e0 = normalize(v1 - v0);
        Vector3f e1 = normalize(v2 - v1);
        N = normalize(crossProduct(e0, e1));
        const Vector2f& st0 = stCoordinates[vertexIndex[index * 3]];
        const Vector2f& st1 = stCoordinates[vertexIndex[index * 3 + 1]];
        const Vector2f& st2 = stCoordinates[vertexIndex[index * 3 + 2]];
        st = st0 * (1 - uv.x - uv.y) + st1 * uv.x + st2 * uv.y;
    }

    Vector3f evalDiffuseColor(const Vector2f& st) const override
    {
        float scale = 5;
        float pattern =
            (fmodf(st.x * scale, 1) > 0.5) ^ (fmodf(st.y * scale, 1) > 0.5);
        return lerp(Vector3f(0.815, 0.235, 0.031),
                    Vector3f(0.937, 0.937, 0.231), pattern);
    }

    Intersection getIntersection(Ray ray)
    {
        Intersection intersec;

        if (bvh) {
            intersec = bvh->Intersect(ray);
        }

        return intersec;
    }

    Bounds3 bounding_box; // 模型的包围盒
    std::unique_ptr<Vector3f[]> vertices;
    uint32_t numTriangles;
    std::unique_ptr<uint32_t[]> vertexIndex;
    std::unique_ptr<Vector2f[]> stCoordinates;

    std::vector<Triangle> triangles;

    BVHAccel* bvh;

    Material* m;
};

inline bool Triangle::intersect(const Ray& ray) { return true; }
inline bool Triangle::intersect(const Ray& ray, float& tnear,
                                uint32_t& index) const
{
    return false;
}

inline Bounds3 Triangle::getBounds() 
{
    return Union(Bounds3(v0, v1), v2);
}

// 光线与三角形相交
// 使用Möller-Trumbore算法
// 这个函数会将光线与三角形的交点信息存储在 inter 中
inline Intersection Triangle::getIntersection(Ray ray)
{
    Intersection inter;

    Vector3f e1 = v1 - v0;
    Vector3f e2 = v2 - v0;
    Vector3f s0 = ray.origin - v0;
    Vector3f s1 = crossProduct(ray.direction, e2);
    Vector3f s2 = crossProduct(s0, e1);

    Vector3f s = Vector3f(dotProduct(s2, e2), dotProduct(s1, s0), dotProduct(s2, ray.direction)) / dotProduct(s1, e1);
    float tnear = s.x;
    float u = s.y;
    float v = s.z;

    if (tnear >= 0 && u >= 0 && v >= 0 && (u + v) <= 1)
    {
        inter.happened = true;
        inter.coords = Vector3f(1 - u - v , u, v);
        inter.normal = normal;
        inter.m = m;
        inter.obj = this;
        inter.distance = tnear;
        return inter;
    }

    return inter;
}

inline Vector3f Triangle::evalDiffuseColor(const Vector2f&) const
{
    return Vector3f(0.5, 0.5, 0.5);
}
