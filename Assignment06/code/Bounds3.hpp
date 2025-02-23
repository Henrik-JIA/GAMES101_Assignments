//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_BOUNDS3_H
#define RAYTRACING_BOUNDS3_H
#include "Ray.hpp"
#include "Vector.hpp"
#include <limits>
#include <array>

// 包围盒类
// 包围盒类，用于表示一个三维空间中的包围盒，
/*
参数：
    pMin, pMax: 包围盒的边界点，是三维向量
*/
class Bounds3
{
  public:
    // 包围盒的边界点，是三维向量
    Vector3f pMin, pMax; // two points to specify the bounding box
    
    // 构造函数
    // 无参数构造函数
    Bounds3()
    {
        // 初始化包围盒的边界点
        double minNum = std::numeric_limits<double>::lowest();
        double maxNum = std::numeric_limits<double>::max();
        pMax = Vector3f(minNum, minNum, minNum);
        pMin = Vector3f(maxNum, maxNum, maxNum);
    }

    // 构造函数
    // 参数：
    // p: 包围盒的边界点，是三维向量
    Bounds3(const Vector3f p) : pMin(p), pMax(p) {}

    // 构造函数
    // 参数：
    // p1, p2: 包围盒的边界点，是三维向量
    Bounds3(const Vector3f p1, const Vector3f p2)
    {
        pMin = Vector3f(fmin(p1.x, p2.x), fmin(p1.y, p2.y), fmin(p1.z, p2.z));
        pMax = Vector3f(fmax(p1.x, p2.x), fmax(p1.y, p2.y), fmax(p1.z, p2.z));
    }

    // 计算包围盒的对角线长度
    Vector3f Diagonal() const { 
        /*
        对角线向量的物理意义：
        d = pMax - pMin 得到的是包围盒从最小角到最大角的向量
        其x分量 d.x = pMax.x - pMin.x 就是包围盒在x轴方向的长度
        同理，y分量是高度，z分量是深度
        */
        return pMax - pMin; 
    }

    // 用于确定包围盒在哪个坐标轴方向延伸最长（即包围盒的最长维度）
    // 返回0，表示第一维x最长；
    // 返回1，表示第二维y最长；
    // 返回2，表示第三维z最长。
    int maxExtent() const
    {
        Vector3f d = Diagonal(); // 获取包围盒对角线向量（x,y,z分量即各轴长度）
        if (d.x > d.y && d.x > d.z) // 先比较x轴是否同时大于y和z轴
            return 0; // 0表示x轴最长
        else if (d.y > d.z) // 若x不是最长，则比较y和z
            return 1; // 1表示y轴最长
        else
            return 2; // 2表示z轴最长
    }

    // 获取包围盒表面积
    // 立方体有6个面，每对相对面的面积相等
    double SurfaceArea() const
    {
        Vector3f d = Diagonal();
        return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
    }

    // 计算包围盒的几何中心
    Vector3f Centroid() { 
        return 0.5 * (pMin + pMax); 
    }
    
    // 计算与另一个包围盒的交集（重叠区域）
    // 用于计算两个轴对齐包围盒（AABB）的交集区域
    Bounds3 Intersect(const Bounds3& b)
    {
        return Bounds3(
            // 新包围盒的最小角点：取两个包围盒各轴最小值的较大者
            // 新pMin.x = max(当前盒的pMin.x, 参数盒的pMin.x)
            // 新pMin.y = max(当前盒的pMin.y, 参数盒的pMin.y)
            // 新pMin.z = max(当前盒的pMin.z, 参数盒的pMin.z)
            Vector3f(fmax(pMin.x, b.pMin.x), 
                     fmax(pMin.y, b.pMin.y), 
                     fmax(pMin.z, b.pMin.z)),
            
            // 新包围盒的最大角点：取两个包围盒各轴最大值的较小者
            // 新pMax.x = min(当前盒的pMax.x, 参数盒的pMax.x)
            // 新pMax.y = min(当前盒的pMax.y, 参数盒的pMax.y)
            // 新pMax.z = min(当前盒的pMax.z, 参数盒的pMax.z)
            Vector3f(
                     fmin(pMax.x, b.pMax.x), 
                     fmin(pMax.y, b.pMax.y), 
                     fmin(pMax.z, b.pMax.z))
        );
    }

    // 用于将点的绝对坐标转换为相对于包围盒的归一化坐标（比例位置）
    // 将点坐标映射到[0,1]范围
    Vector3f Offset(const Vector3f& p) const
    {
        // 1. 计算点相对于包围盒左下角(pMin)的绝对偏移量
        Vector3f o = p - pMin; // 得到各轴方向的原始偏移量
        
        // 2. 对每个轴进行归一化处理（映射到0-1范围）
        // 对每个轴进行归一化处理
        if (pMax.x > pMin.x)  // 防止除零（当包围盒在x轴无厚度时）
            o.x /= pMax.x - pMin.x; // x方向比例 = x偏移量 / 包围盒x轴长度
        if (pMax.y > pMin.y)
            o.y /= pMax.y - pMin.y; // y方向同理
        if (pMax.z > pMin.z)
            o.z /= pMax.z - pMin.z; // z方向同理
        return o;
    }

    // 用于判断两个包围盒是否重叠：
    // 轴对齐包围盒（AABB）相交检测的核心原理：
    // 轴对齐特性：包围盒的边与坐标轴平行，因此只需检查各轴向的投影区间是否重叠
    // 分离轴定理（SAT）应用：对于AABB，若存在任一坐标轴使得两个包围盒的投影区间不重叠，则包围盒不相交
    bool Overlaps(const Bounds3& b1, const Bounds3& b2)
    {
        // 判断两个包围盒在x轴方向是否重叠
        bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
        // 判断两个包围盒在y轴方向是否重叠
        bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
        // 判断两个包围盒在z轴方向是否重叠
        bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);
        // 只有当三个轴方向都重叠时，才认为两个包围盒重叠
        return (x && y && z);
    }

    // 用于判断点是否在包围盒内
    bool Inside(const Vector3f& p, const Bounds3& b)
    {
        // 点p的每个坐标都必须在包围盒b的对应范围内
        return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y &&
                p.y <= b.pMax.y && p.z >= b.pMin.z && p.z <= b.pMax.z);
    }

    // 用于访问包围盒的边界点
    // 返回包围盒的边界点
    // 参数：
    // i: 0表示pMin，1表示pMax
    inline const Vector3f& operator[](int i) const
    {
        // 返回pMin或pMax
        return (i == 0) ? pMin : pMax;
    }

    // 用于判断光线是否与包围盒相交
    // 参数：
    // ray: 光线
    // invDir: 光线方向的倒数
    // dirIsNeg: 光线方向的符号
    inline bool IntersectP(const Ray& ray, const Vector3f& invDir,
                           const std::array<int, 3>& dirisNeg) const;
};

// 光线与轴对齐包围盒（AABB）的快速相交检测，采用经典的"slab method"算法。
/*
参数：
    ray: 光线
    invDir: 光线方向的倒数
    （预计算目的：用乘法代替除法提升性能）
    dirIsNeg: 光线方向的符号
    （预计算目的：正确处理光线负方向带来的区间反转，当光线方向为负时，实际先遇到pMax平面）
*/
inline bool Bounds3::IntersectP(const Ray& ray, const Vector3f& invDir,
                                const std::array<int, 3>& dirIsNeg) const
{
    // invDir: ray direction(x,y,z), invDir=(1.0/x,1.0/y,1.0/z), use this because Multiply is faster that Division
    // dirIsNeg: ray direction(x,y,z), dirIsNeg=[int(x>0),int(y>0),int(z>0)], use this to simplify your logic
    // TODO test if ray bound intersects

    const auto& origin = ray.origin;

    // 包围盒的进出时间，初始化光线进入和退出时间
    float tEnter = -std::numeric_limits<float>::infinity(); // 光线最早进入时间
    float tExit = std::numeric_limits<float>::infinity(); // 光线最晚退出时间
    
    // 遍历包围盒的三个轴，也就是遍历x,y,z轴
    for (int i = 0; i < 3; i++)
    {
        // 1. 单轴相交计算获取进入和退出时间
        // 计算光线在当前轴上的进入和退出时间（使用预计算的倒数）
        // 计算光线在当前轴向上与两个平面（pMin和pMax平面）的相交时间
        // 光线方程：origin + t * direction
        // 平面方程：pMin[i]
        // 解平面方程：origin[i] + t * direction[i] = pMin[i]
        // 解得：t = (pMin[i] - origin[i]) / direction[i]
        float min = (pMin[i] - origin[i]) * invDir[i]; // 光线进入时间
        float max = (pMax[i] - origin[i]) * invDir[i]; // 光线退出时间

        // 2. 负方向修正，光线永远是正方向
        // 考虑负数的大小判断问题，处理光线方向为负的情况（需要交换tMin/tMax）
        // 负方向修正：
        // 当光线方向为负时：
        // 实际先与pMax平面相交（对应更大的坐标值）
        // 后与pMin平面相交（对应更小的坐标值）
        // 因此需要交换min/max值
        if (!dirIsNeg[i]){ 
            std::swap(min, max); 
        }

        // 3. 全局区间更新
        // 更新全局的进入和退出时间       
        // 区间交集原理：
        // 有效相交区间 = 所有轴相交区间的重叠部分
        // 进入时间取最大（必须进入所有轴向）
        // 退出时间取最小（只要退出任一轴向）
        tEnter = std::max(min, tEnter); // 取各轴进入时间的最大值
        tExit = std::min(max, tExit); // 取各轴退出时间的最小值

    }
    
    // 最终判断条件
    // 光线在包围盒呆过的条件：
    // 最终相交条件：
    // 1. 进入时间 < 退出时间（光线在包围盒内停留过）
    // 2. 退出时间 >= 0（光线在传播路径上与包围盒相交）
    return tEnter < tExit && tExit >= 0;
}

// 两个包围盒交集：用于合并两个包围盒
inline Bounds3 Union(const Bounds3& b1, const Bounds3& b2)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b1.pMin, b2.pMin); // 取两个包围盒各轴最小值
    ret.pMax = Vector3f::Max(b1.pMax, b2.pMax); // 取两个包围盒各轴最大值
    return ret; // 返回能包含两个原始包围盒的新包围盒
}

// 包围盒与点交集的新包围盒：用于合并一个包围盒和一个点
inline Bounds3 Union(const Bounds3& b, const Vector3f& p)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b.pMin, p); // 比较包围盒最小点和当前点
    ret.pMax = Vector3f::Max(b.pMax, p); // 比较包围盒最大点和当前点
    return ret; // 返回能包含原始包围盒和点的新包围盒
}

#endif // RAYTRACING_BOUNDS3_H
