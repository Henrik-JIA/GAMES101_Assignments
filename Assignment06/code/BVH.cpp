#include <algorithm>
#include <cassert>
#include "BVH.hpp"

// BVH.hpp中定义的一个类
// class::class():property1(value1),property2(value2){...}
// 构造函数
// 参数：
// p: 物体列表
// maxPrimsInNode: 最大物体数量
// splitMethod: 分割方法
// 属性：
// maxPrimsInNode: 最大物体数量
// splitMethod: 分割方法
// primitives: 物体列表
BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), 
      splitMethod(splitMethod), 
      primitives(std::move(p)) // 其中std::move(p)表示将p的所有权转移给primitives
{
    // 初始化时间
    time_t start, stop;
    // 获取当前时间
    time(&start);
    // 如果物体列表为空，则返回
    if (primitives.empty())
        return;

    // 建立BVH树，返回根节点
    root = recursiveBuild(primitives);
    // 结束时间
    time(&stop);
    // 计算建立 BVH树花费的时间
    double diff = difftime(stop, start);

    // 计算建立 BVH树花费的时间
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    // 打印建立 BVH树花费的时间
    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

// 递归建立 BVH树
// 参数：
// objects: 物体列表
// 返回值：
// BVHBuildNode*: BVH树的根节点
BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    // 建立一个新的节点
    BVHBuildNode* node = new BVHBuildNode();

    // 叶子节点中只有一个物体，划分的是否过细了一点？
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];  // 叶子节点存放物体
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) { // 两个物体
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else { // 多个物体
        /*// Compute bounds of all primitives in BVH node
        Bounds3 bounds;
        for (int i = 0; i < objects.size(); ++i)
            bounds = Union(bounds, objects[i]->getBounds());*/

        // 质心包围盒
        // 遍历所有物体，计算它们的包围盒质心
        // 然后构建一个包含所有质心的新包围盒
        Bounds3 centroidBounds; 
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds = Union(centroidBounds, objects[i]->getBounds().Centroid());

        // 如果包围盒是不均匀的，也就是某一个轴特别长
        // 就需要优先划分
        // 注意：这里的划分是基于对物体的划分，而不是对空间的划分
        int dim = centroidBounds.maxExtent();
        // 所有物体质心坐标在该轴上的跨度
        switch (dim) {
            // X轴过长
            case 0:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().x < f2->getBounds().Centroid().x;
                });
                break;
            // Y轴过长
            case 1:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().y < f2->getBounds().Centroid().y;
                });
                break;
            // Z轴过长
            case 2:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().z < f2->getBounds().Centroid().z;
                });
                break;
        }

        // 使用 C++标准库中的 迭代器 和 容器 来进行对象切片的操作
        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        // 使用 assert断言来检查对象切片后的总数量是否等于原始容器 objects的数量，以确保切片操作正确执行。
        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        // 节点向下递归
        // 中间节点无需将物体保存到节点中
        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        // 当前节点的包围盒大小递归返回
        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

// 收集所有节点包围盒
void BVHAccel::CollectAllBounds(BVHBuildNode* node, std::vector<Bounds3>& boundsList) const {
    if (!node) return;
    
    // 添加当前节点的包围盒
    boundsList.push_back(node->bounds);
    
    // 递归收集子节点
    CollectAllBounds(node->left, boundsList);
    CollectAllBounds(node->right, boundsList);
}

// 获取所有节点包围盒
std::vector<Bounds3> BVHAccel::GetAllNodeBounds() const {
    std::vector<Bounds3> boundsList;
    CollectAllBounds(root, boundsList);
    return boundsList;
}

// 光线在 BVH树中碰撞
// 参数：
// ray: 光线
// 返回值：
// Intersection: 碰撞信息
Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    // 如果BVH树根节点为空，则返回空的碰撞信息
    if (!root) return isect;
    // 求得碰撞信息，这里的碰撞是与BVH树根节点中所有物体进行碰撞检测，返回Intersection isect，
    // 如果碰撞到了，则isect.happened为true，否则为false
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

// 求得碰撞信息
// 参数：
// node: BVH树的根节点
// ray: 光线
// 返回值：
// Intersection: 碰撞信息
Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection 遍历BVH树以找到交点
    // 初始化碰撞信息
    Intersection isect;

    // 计算光线方向的符号：
    // 1 表示该轴方向非负（≥0）
    // 0 表示该轴方向负（<0）  
    std::array<int, 3> dirIsNeg{};
    dirIsNeg[0] = int(ray.direction.x >= 0);
    dirIsNeg[1] = int(ray.direction.y >= 0);
    dirIsNeg[2] = int(ray.direction.z >= 0);

    // 1. 判断光线是否与包围盒相交
    // 如果光线连包围盒都没有撞击到，则一定不会与包围盒中的物体相碰撞
    // 直接返回空的碰撞信息，其中Intersection中的成员happened默认是false
    // 参数：
    // ray: 光线
    // ray.direction_inv: 光线方向的倒数
    // dirIsNeg: 光线方向的符号
    if (!node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg))
    {
        return isect;
    }

    // 2. 光线进入过包围盒
    // 如果该节点是叶子节点，这时检测是否与该包围盒中的物体碰撞
    if (node -> left == nullptr && node -> right == nullptr)
    {
        // 求得碰撞信息
        isect = node->object->getIntersection(ray);
        return isect;
    }

    // 如果该节点是中间节点，向下递归
    // 当访问一个中间节点时：
    // 1. 检查光线是否与当前节点的包围盒相交
    // 2. 如果相交：
    //     a. 递归检查左子节点
    //     b. 递归检查右子节点
    //     c. 合并两个子节点的碰撞结果
    // 3. 如果不相交：
    //     直接返回无碰撞
    auto hit1 = getIntersection(node->left, ray);
    auto hit2 = getIntersection(node->right, ray);

    return hit1.distance < hit2.distance ? hit1 : hit2;
}