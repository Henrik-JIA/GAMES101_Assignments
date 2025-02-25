//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");

    // 此时场景里面的物体就只有一个模型，也就是 MeshTriangle
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this -> bvh -> Intersect(ray); // 光线先在场景中的 BVH树中碰撞
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of the Whitted-syle light transport algorithm (E [S*] (D|G) L)
//
// This function is the function that compute the color at the intersection point
// of a ray defined by a position and a direction. Note that thus function is recursive (it calls itself).
//
// If the material of the intersected object is either reflective or reflective and refractive,
// then we compute the reflection/refracton direction and cast two new rays into the scene
// by calling the castRay() function recursively. When the surface is transparent, we mix
// the reflection and refraction color using the result of the fresnel equations (it computes
// the amount of reflection and refractin depending on the surface normal, incident view direction
// and surface refractive index).
//
// If the surface is duffuse/glossy we use the Phong illumation model to compute the color
// at the intersection point.
// 光线追踪的实现
// 这个函数计算一个由位置和方向定义的射线在交点处的颜色，注意这个函数是一个递归的（自己调用自己）。
// 这里Ray是光线类，&ray是光学对象，包括射线起点、方向、时间、最小和最大距离。
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // 超过递归最大深度则返回黑色
    if (depth > this -> maxDepth) {
        return Vector3f(0.0,0.0,0.0);
    }

    // 光线先在场景中的 BVH树中碰撞
    Intersection intersection = Scene::intersect(ray);

    // 默认背景颜色，也就是光线碰撞物体的默认颜色，这个就是默认的背景颜色。
    Vector3f hitColor = this -> backgroundColor;

    // 如果光线碰撞到了物体，intersection.happened这个为true，也就是说光线与物体中间没有任何遮挡。
    if(intersection.happened) {
        // origin + t * direction
        Vector3f hitPoint = ray.origin + ray.direction * intersection.distance;
        Material *m = intersection.m; // 材质
        Object *hitObject = intersection.obj; // 碰撞的所属物体
        Vector3f N = intersection.normal; // normal
        Vector2f st; // st coordinates

        // 此时碰撞信息已经包含了所有我们需要的信息
        // hitObject->getSurfaceProperties(hitPoint, ray.direction, index, uv, N, st);

        switch (m -> getType()) {
            case REFLECTION_AND_REFRACTION: // 折射+反射
            {
                // 计算反射方向
                Vector3f reflectionDirection = normalize(reflect(ray.direction, N));
                // 计算折射方向
                Vector3f refractionDirection = normalize(refract(ray.direction, N, m->ior));

                Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ?
                                             hitPoint - N * EPSILON :
                                             hitPoint + N * EPSILON;

                Vector3f refractionRayOrig = (dotProduct(refractionDirection, N) < 0) ?
                                             hitPoint - N * EPSILON :
                                             hitPoint + N * EPSILON;

                // 递归计算反射光线和折射光线的颜色
                Vector3f reflectionColor = castRay(Ray(reflectionRayOrig, reflectionDirection), depth + 1);
                Vector3f refractionColor = castRay(Ray(refractionRayOrig, refractionDirection), depth + 1);

                // 使用了菲涅尔反射系数kr，来计算反射和折射的比例
                float kr;
                fresnel(ray.direction, N, m->ior, kr);

                hitColor = reflectionColor * kr + refractionColor * (1 - kr);
                break;
            }
            case REFLECTION: // 反射
            {
                float kr;
                fresnel(ray.direction, N, m->ior, kr);
                Vector3f reflectionDirection = reflect(ray.direction, N);

                Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ?
                                             hitPoint + N * EPSILON :
                                             hitPoint - N * EPSILON;

                hitColor = castRay(Ray(reflectionRayOrig, reflectionDirection),depth + 1) * kr;
                break;
            }
            default: // 漫反射+镜面反射
            {
                // [comment]
                // We use the Phong illumation model int the default case. The phong model
                // is composed of a diffuse and a specular reflection component.
                // [/comment]
                // 漫反射，镜面反射
                Vector3f lightAmt = 0, specularColor = 0;

                // 先计算射线方向与法线点乘，获得光线方向与法线方向的夹角
                // 当光线方向与法线方向夹角 > 90度（即光线从物体外部射入）时，点积为负
                // 当光线方向与法线方向夹角 < 90度（即光线从物体内部射出）时，点积为正
                // 由于计算机中浮点精度问题，会导致光线与平面交点存在上下偏移。
                // 如果夹角小于0，则交点物体背面，因此需要将该交点向法线反方向偏移 EPSILON，这样可以避免光线与物体表面相交。
                // 如果夹角大于0，则交点物体正面，因此需要将该交点向法线正方向偏移 EPSILON，这样可以避免光线与物体表面相交。
                Vector3f shadowPointOrig = (dotProduct(ray.direction, N) < 0) ?
                                           hitPoint + N * EPSILON :
                                           hitPoint - N * EPSILON;
                // [comment]
                // Loop over all lights in the scene and sum their contribution up
                // We also apply the lambert cosine law
                // [/comment]
                // 遍历场景中的所有光源，并计算它们对交点的影响
                for (uint32_t i = 0; i < get_lights().size(); ++i)
                {
                    // 获取光源类型
                    // 如果光源类型为AreaLight，则需要计算光源的面积
                    auto area_ptr = dynamic_cast<AreaLight*>(this->get_lights()[i].get());
                    if (area_ptr)
                    {
                        // Do nothing for this assignment
                        // 计算光源的面积
                    }
                    else // 如果光源类型为PointLight，则需要计算光源的距离
                    {
                        // 计算光源到碰撞点的距离，为了计算漫反射光照计算公式中的r
                        Vector3f lightDir = get_lights()[i]->position - hitPoint;
                        // square of the distance between hitPoint and the light
                        // 计算光源到碰撞点的距离的平方r^2
                        float lightDistance2 = dotProduct(lightDir, lightDir);
                        // 计算碰撞点指向光源的方向（注意这里光源方向始终是朝外的）
                        lightDir = normalize(lightDir);
                        // 计算光源方向与法线方向的夹角，用来表面接收到多少光（能量），因为光线与平面并不是垂直的接收全部光能。
                        // 也就是漫反射光照计算公式中的max(0, n · l)部分。
                        float LdotN = std::max(0.f, dotProduct(lightDir, N));

                        // 这里就是碰撞点与光源之间是否有碰撞点，那么这里的光线就是从碰撞点射向光源的光线。
                        // 这里的BVH树碰撞检测，是与BVH树根节点中所有物体进行碰撞检测，返回Intersection shadow_res，
                        // 如果碰撞到了，则shadow_res.happened为true，否则为false
                        // is the point in shadow, and is the nearest occluding object closer to the object than the light itself?
                        Intersection shadow_res = bvh->Intersect(Ray(shadowPointOrig, lightDir));
                        // 如果碰撞到了，则shadow_res.happened为true，否则为false
                        // 如果碰撞到了，则shadow_res.distance * shadow_res.distance < lightDistance2，这里就是判断光源到新碰撞点距离是否小于光源到碰撞点距离，其实就是判断障碍物是否在光源后面。
                        // 两个同时满足才可以碰撞点在阴影中，否则碰撞点不在阴影中。
                        // 也就是说这个碰撞点是不能直接被光源照亮的。需要间接光照。通过其他物体反射过来的光照来照亮，这里还未涉及。所以这里并不是全局光照，只是直接光照。
                        bool inShadow = shadow_res.happened && (shadow_res.distance * shadow_res.distance < lightDistance2);
                        
                        // 光强度随距离平方的衰减，这里由于距离太远，光照强度衰减太快，所以就不进行光照衰减。
                        // float attenuation = 1.0f / (lightDistance2 * 0.0001);
                        float attenuation = 1.0f;
                        
                        // 这里是漫反射光照计算公式：
                        // 计算光照强度，如果碰撞点在阴影中，则光照强度为0，否则为光源强度乘以光线与法线夹角的余弦值。
                        // 这里有多个光源，因此需要累加。
                        lightAmt += (1 - int(inShadow)) * (get_lights()[i]->intensity * attenuation) * LdotN;

                        // 计算镜面反射方向，是光线的反方向，因为光线是从碰撞点射向光源的。
                        Vector3f reflectionDirection = reflect(-lightDir, N);

                        // 计算镜面反射光照强度，这里使用的是Phong光照模型，也就是镜面反射光照强度。
                        specularColor += powf(std::max(0.f, -dotProduct(reflectionDirection, ray.direction)),
                                              m->specularExponent) * (get_lights()[i]->intensity * attenuation);
                    }
                }
                hitColor = lightAmt * (hitObject->evalDiffuseColor(st) * m->Kd + specularColor * m->Ks);
                break;
            }
        }
    }

    return hitColor;
}