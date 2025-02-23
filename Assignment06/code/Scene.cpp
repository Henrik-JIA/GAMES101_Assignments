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
            case REFLECTION_AND_REFRACTION:
            {
                Vector3f reflectionDirection = normalize(reflect(ray.direction, N));
                Vector3f refractionDirection = normalize(refract(ray.direction, N, m->ior));

                Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ?
                                             hitPoint - N * EPSILON :
                                             hitPoint + N * EPSILON;

                Vector3f refractionRayOrig = (dotProduct(refractionDirection, N) < 0) ?
                                             hitPoint - N * EPSILON :
                                             hitPoint + N * EPSILON;

                Vector3f reflectionColor = castRay(Ray(reflectionRayOrig, reflectionDirection), depth + 1);
                Vector3f refractionColor = castRay(Ray(refractionRayOrig, refractionDirection), depth + 1);

                float kr;
                fresnel(ray.direction, N, m->ior, kr);

                hitColor = reflectionColor * kr + refractionColor * (1 - kr);
                break;
            }
            case REFLECTION:
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
            default:
            {
                // [comment]
                // We use the Phong illumation model int the default case. The phong model
                // is composed of a diffuse and a specular reflection component.
                // [/comment]
                Vector3f lightAmt = 0, specularColor = 0;

                Vector3f shadowPointOrig = (dotProduct(ray.direction, N) < 0) ?
                                           hitPoint + N * EPSILON :
                                           hitPoint - N * EPSILON;
                // [comment]
                // Loop over all lights in the scene and sum their contribution up
                // We also apply the lambert cosine law
                // [/comment]
                for (uint32_t i = 0; i < get_lights().size(); ++i)
                {
                    auto area_ptr = dynamic_cast<AreaLight*>(this->get_lights()[i].get());
                    if (area_ptr)
                    {
                        // Do nothing for this assignment
                    }
                    else
                    {
                        Vector3f lightDir = get_lights()[i]->position - hitPoint;
                        // square of the distance between hitPoint and the light
                        float lightDistance2 = dotProduct(lightDir, lightDir);
                        lightDir = normalize(lightDir);

                        float LdotN = std::max(0.f, dotProduct(lightDir, N));

                        // is the point in shadow, and is the nearest occluding object closer to the object than the light itself?
                        Intersection shadow_res = bvh->Intersect(Ray(shadowPointOrig, lightDir));
                        bool inShadow = shadow_res.happened && (shadow_res.distance * shadow_res.distance < lightDistance2);

                        lightAmt += (1 - int(inShadow)) * get_lights()[i]->intensity * LdotN;
                        Vector3f reflectionDirection = reflect(-lightDir, N);
                        specularColor += powf(std::max(0.f, -dotProduct(reflectionDirection, ray.direction)),
                                              m->specularExponent) * get_lights()[i]->intensity;
                    }
                }
                hitColor = lightAmt * (hitObject->evalDiffuseColor(st) * m->Kd + specularColor * m->Ks);
                break;
            }
        }
    }

    return hitColor;
}