#include <fstream>
#include "Vector.hpp"
#include "Renderer.hpp"
#include "Scene.hpp"
#include <optional>

// 将角度转换为弧度
inline float deg2rad(const float &deg)
{ return deg * M_PI/180.0; }

// Compute reflection direction
// 计算反射光
Vector3f reflect(const Vector3f &I, const Vector3f &N)
{
    return I - 2 * dotProduct(I, N) * N;
}

// [comment]
// Compute refraction direction using Snell's law
//
// We need to handle with care the two possible situations:
//
//    - When the ray is inside the object
//
//    - When the ray is outside.
//
// If the ray is outside, you need to make cosi positive cosi = -N.I
//
// If the ray is inside, you need to invert the refractive indices and negate the normal N
// [/comment]
// 计算折射光，需要折射率
Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior)
{
    float cosi = clamp(-1, 1, dotProduct(I, N));
    float etai = 1, etat = ior;
    Vector3f n = N;
    if (cosi < 0) { cosi = -cosi; } else { std::swap(etai, etat); n= -N; }
    float eta = etai / etat;
    float k = 1 - eta * eta * (1 - cosi * cosi);
    return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
}

// [comment]
// Compute Fresnel equation
//
// \param I is the incident view direction
//
// \param N is the normal at the intersection point
//
// \param ior is the material refractive index
// [/comment]
// 菲涅尔反射系数
// 计算比例，用于求解反射颜色占的多，还是折射颜色占的多。
float fresnel(const Vector3f &I, const Vector3f &N, const float &ior)
{
    float cosi = clamp(-1, 1, dotProduct(I, N));
    float etai = 1, etat = ior;
    if (cosi > 0) {  std::swap(etai, etat); }
    // Compute sini using Snell's law
    float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
    // Total internal reflection
    if (sint >= 1) {
        return 1;
    }
    else {
        float cost = sqrtf(std::max(0.f, 1 - sint * sint));
        cosi = fabsf(cosi);
        float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
        float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
        return (Rs * Rs + Rp * Rp) / 2;
    }
    // As a consequence of the conservation of energy, transmittance is given by:
    // kt = 1 - kr;
}

// [comment]
// Returns true if the ray intersects an object, false otherwise.
//
// \param orig is the ray origin
// \param dir is the ray direction
// \param objects is the list of objects the scene contains
// \param[out] tNear contains the distance to the cloesest intersected object.
// \param[out] index stores the index of the intersect triangle if the interesected object is a mesh.
// \param[out] uv stores the u and v barycentric coordinates of the intersected point
// \param[out] *hitObject stores the pointer to the intersected object (used to retrieve material information, etc.)
// \param isShadowRay is it a shadow ray. We can return from the function sooner as soon as we have found a hit.
// [/comment]
// 如果光线与物体相交，则返回true，否则返回false
// orig是光线原点，dir是光线方向，objects是场景中的物体列表
std::optional<hit_payload> trace(
        const Vector3f &orig, const Vector3f &dir,
        const std::vector<std::unique_ptr<Object> > &objects)
{
    // 初始化tNear为无穷大
    float tNear = kInfinity;
    // 初始化payload为空，碰撞信息
    std::optional<hit_payload> payload;
    // 遍历场景中的所有物体
    // 这里objects存在两种物体，一种是球体，一种是三角形，不同的物体函数实现不同，因此intersect函数实现也不同。
    for (const auto & object : objects)
    {
        // 初始化tNearK为无穷大
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        // 如果光线与物体相交，并且tNearK小于tNear，找到离观测点最近的物体
        // 这里球体与三角形的intersect函数实现不同。
        // intersect函数是判断光线是否与物体相交，不同的物体函数实现不同
        // orig是光线原点，dir是光线方向，tNearK是光线与物体相交的距离，indexK是物体索引，uvK是物体纹理坐标
        if (object->intersect(orig, dir, tNearK, indexK, uvK) && tNearK < tNear)
        {
            // 清理之前payload中写入的碰撞信息
            payload.emplace();
            // 将碰撞信息存储到payload中
            payload->hit_obj = object.get();
            payload->tNear = tNearK;
            payload->index = indexK;
            payload->uv = uvK;
            // 更新tNear
            tNear = tNearK;
        }
    }

    // 此时，payload中存储了光线与物体相交的碰撞信息，如果payload为空，则返回空，否则返回payload
    return payload;
}

// [comment]
// Implementation of the Whitted-style light transport algorithm (E [S*] (D|G) L)
//
// This function is the function that compute the color at the intersection point
// of a ray defined by a position and a direction. Note that thus function is recursive (it calls itself).
//
// If the material of the intersected object is either reflective or reflective and refractive,
// then we compute the reflection/refraction direction and cast two new rays into the scene
// by calling the castRay() function recursively. When the surface is transparent, we mix
// the reflection and refraction color using the result of the fresnel equations (it computes
// the amount of reflection and refraction depending on the surface normal, incident view direction
// and surface refractive index).
//
// If the surface is diffuse/glossy we use the Phong illumation model to compute the color
// at the intersection point.
// [/comment]
// 光线追踪的实现
// 这个函数计算一个由位置和方向定义的射线在交点处的颜色，注意这个函数是一个递归的（自己调用自己）。
Vector3f castRay( const Vector3f &orig, const Vector3f &dir, const Scene& scene, int depth )
{
    // 超过递归最大深度则返回黑色
    if (depth > scene.maxDepth) {
        return Vector3f(0.0,0.0,0.0);
    }

    // 默认背景颜色，也就是光线碰撞物体的默认颜色，这个就是默认的背景颜色。
    Vector3f hitColor = scene.backgroundColor;

    // 如果光线碰撞到了之前我们加入到场景中的物体，则进入分支
    // 否则该像素块还是背景颜色。
    // trace函数传入orig是光线原点，dir是光线方向（这个方向是对应每一个像素点的方向射出去的），objects是场景中的物体列表。
    // 其中第一个球体是漫反射材质，第二个球体是反射和折射材质，三角形全部是漫反射和光泽材质。
    // trace函数返回一个optional<hit_payload>，如果光线与物体相交，则返回碰撞信息，否则返回空
    if (auto payload = trace(orig, dir, scene.get_objects()); payload)
    {
        // 通过trace函数返回的payload，获取碰撞信息，算出碰撞点
        Vector3f hitPoint = orig + dir * payload->tNear;

        // 获取碰撞点的法向量和纹理坐标
        Vector3f N; // 法向量
        Vector2f st; // 纹理坐标

        // payload保存了碰撞信息，通过碰撞信息获取碰撞点的法向量和纹理坐标
        // getSurfaceProperties函数是用来求得碰撞点在物体表面的其他信息，不同物体实现不同
        payload->hit_obj->getSurfaceProperties(hitPoint, dir, payload->index, payload->uv, N, st);

        // 根据碰撞点的材质类型，选择不同的材质，并计算出不同的颜色
        // 这里payload->hit_obj->materialType是获取碰撞点的材质类型
        // 物体列表中，其中第一个球体是漫反射材质，第二个球体是反射和折射材质，三角形全部是漫反射和光泽材质。
        switch (payload->hit_obj->materialType) {
            // 反射和折射
            case REFLECTION_AND_REFRACTION:
            {   
                // 反射光
                Vector3f reflectionDirection = normalize(reflect(dir, N));
                // 折射光
                Vector3f refractionDirection = normalize(refract(dir, N, payload->hit_obj->ior));

                // 判断光线是从内部射向表面，还是外部射向表面
                // 这个偏移量的目的是避免光线与命中点所在表面相交。
                Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ?
                                             hitPoint - N * scene.epsilon :
                                             hitPoint + N * scene.epsilon;
                Vector3f refractionRayOrig = (dotProduct(refractionDirection, N) < 0) ?
                                             hitPoint - N * scene.epsilon :
                                             hitPoint + N * scene.epsilon;

                // 递归调用castRay函数，计算反射光和折射光的颜色
                Vector3f reflectionColor = castRay(reflectionRayOrig, reflectionDirection, scene, depth + 1);
                Vector3f refractionColor = castRay(refractionRayOrig, refractionDirection, scene, depth + 1);
                
                // 计算反射光和折射光的权重
                // 使用菲涅尔反射系数
                float kr = fresnel(dir, N, payload->hit_obj->ior);
                hitColor = reflectionColor * kr + refractionColor * (1 - kr);
                break;
            }
            // 反射
            case REFLECTION:
            {
                // 使用菲涅尔反射系数
                float kr = fresnel(dir, N, payload->hit_obj->ior);
                
                // 反射光
                Vector3f reflectionDirection = normalize(reflect(dir, N));
                // 判断光线是从内部射向表面，还是外部射向表面
                // 这个偏移量的目的是避免光线与命中点所在表面相交。
                Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ?
                                             hitPoint + N * scene.epsilon :
                                             hitPoint - N * scene.epsilon;
                
                // 递归调用castRay函数，计算反射光的颜色
                hitColor = castRay(reflectionRayOrig, reflectionDirection, scene, depth + 1) * kr;
                break;
            }
            // 默认（漫反射和光泽），使用Phong光照模型计算颜色，Phong模型由漫反射和镜面反射两个组成部分组成。
            // 这里是直接光照的判定，没有考虑间接光照（其他物体上反射过来的光线）。
            default:
            {
                // [comment]
                // We use the Phong illumation model int the default case. The phong model
                // is composed of a diffuse and a specular reflection component.
                // [/comment]
                // 初始化两个向量，一个是漫反射，另一个是高光
                Vector3f lightAmt = 0, specularColor = 0;
                // 判断碰撞点是否处于阴影下，还需要从碰撞点射向光源。
                // 如果光线方向与法向量夹角小于90度，则碰撞点向法向量方向偏移epsilon，否则向反方向偏移epsilon。
                // scene.epsilon 是一个很小的值。
                // 目的是为了避免，由于计算机对浮点数的精度问题，导致光线与物体相交的点后，反射或折射光线与这个点所在物体表面相交的情况。
                // 也就是说，避免浮点精度问题导致出射光线与表面相交的情况产生。
                Vector3f shadowPointOrig = (dotProduct(dir, N) < 0) ?
                                           hitPoint + N * scene.epsilon :
                                           hitPoint - N * scene.epsilon;
                // [comment]
                // Loop over all lights in the scene and sum their contribution up
                // We also apply the lambert cosine law
                // [/comment]
                // 遍历场景中的所有光源，并将它们的贡献相加。
                // 使用lambert余弦定律计算光照强度。
                for (auto& light : scene.get_lights()) {
                    // 目前lightDir为光源到碰撞点的距离。
                    Vector3f lightDir = light->position - hitPoint;
                    // square of the distance between hitPoint and the light
                    float lightDistance2 = dotProduct(lightDir, lightDir);

                    // lightDir为归一化光源方向
                    lightDir = normalize(lightDir);

                    // 碰撞点射向光源
                    // （如果碰撞到了场景中的其他物体，这里使用了trace函数） && （该点与碰撞物体距离 小于 该点与光源距离） 说明碰撞点在阴影下
                    // 这里的shadowPointOrig是碰撞点，lightDir是指向光源方向，scene.get_objects()是场景中的所有物体
                    // is the point in shadow, and is the nearest occluding object closer to the object than the light itself?
                    auto shadow_res = trace(shadowPointOrig, lightDir, scene.get_objects());
                    // 如果碰撞到了场景中的其他物体，并且该点与碰撞物体距离 小于 该点与光源距离，则说明碰撞点在阴影下
                    // 这里是直接光照的判定，没有考虑间接光照（其他物体上反射过来的光线）。
                    // 所以这里不是全局光照（全局光照 = 直接光照 + 间接光照），这里只有直接光照
                    bool inShadow = shadow_res && (shadow_res->tNear * shadow_res->tNear < lightDistance2);

                    // Phong光照模型的漫反射部分：
                    // 计算光源方向与法向量的夹角
                    float LdotN = std::max(0.f, dotProduct(lightDir, N));
                    // 这里的阴影是硬阴影，并不是软阴影，没有考虑全局光照。
                    // 在阴影下，光照强度为0，否则为光源强度 * 漫反射系数。
                    // 这个是累加的，因为有多个光源。
                    lightAmt += inShadow ? 0 : light->intensity * LdotN;

                    // 镜面反射，注意lightDir调转方向（原来是从碰撞点指向光源）。
                    Vector3f reflectionDirection = reflect(-lightDir, N);

                    // Phong光照模型，计算镜面反射部分：(I/r²) * max(0, n·h)^p
                    specularColor += powf(std::max(0.f, -dotProduct(reflectionDirection, dir)),
                        payload->hit_obj->specularExponent) * light->intensity;
                }

                // 计算碰撞点的颜色
                // 漫反射颜色 * 漫反射系数 + 镜面反射颜色 * 镜面反射系数
                // 注意：payload->hit_obj->evalDiffuseColor(st)就是本身的颜色
                // 球类evalDiffuseColor方法并没有重写，但是三角形有重写。
                hitColor = (lightAmt * payload->hit_obj->evalDiffuseColor(st) * payload->hit_obj->Kd) 
                           + (specularColor * payload->hit_obj->Ks);
                break;
            }
        }
    }

    return hitColor;
}

// [comment]
// The main render function. This where we iterate over all pixels in the image, generate
// primary rays and cast these rays into the scene. The content of the framebuffer is
// saved to a file.
// [/comment]
// 主要的渲染函数：
// 在这里，我们遍历所有像素，生成主光线，并将这些光线投射到场景中。
// 帧缓冲区的内容保存到一个文件中。
void Renderer::Render(const Scene& scene)
{
    // 创建一个帧缓冲区，用于存储每个像素的颜色值
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    // 只需scale与imageAspectRatio，就可以计算出所有的像素点在三维空间中的位置。
    // 设置缩放因子，这里fov默认是90°，tan(90°/2 = 45°) = 1，如果提供一个z值，那么z乘以tan(fov/2)就可以计算出一个三维空间中的半高。
    float scale = std::tan(deg2rad(scene.fov * 0.5f));
    // 设置图像的宽高比
    float imageAspectRatio = scene.width / (float)scene.height;

    // Use this variable as the eye position to start your rays.
    // 设置眼睛的位置，这里默认是(0, 0, 0)
    Vector3f eye_pos(0);

    // 遍历所有像素，生成主光线，并将这些光线投射到场景中。
    // 这里j是行，i是列，原点在左上角，这个就是屏幕坐标。
    // 将所有像素点都遍历一遍，计算出每个像素点在三维空间中的位置，然后生成主光线，并将这些光线投射到场景中。
    int m = 0;
    for (int j = 0; j < scene.height; ++j)
    {
        for (int i = 0; i < scene.width; ++i)
        {
            // generate primary ray direction
            // 计算当前像素在三维空间中的位置
            // 将像素坐标转换为三维空间中的位置，三维空间的坐标原点(0, 0, 0)在屏幕中心，相机位置就是在(0, 0, 0)处。
            float x = (2 * (i + 0.5) / scene.width - 1) * imageAspectRatio * scale;
            float y = (1 - 2 * ((j + 0.5) / scene.height)) * 1 * scale;
            // TODO: Find the x and y positions of the current pixel to get the direction
            // vector that passes through it.
            // Also, don't forget to multiply both of them with the variable *scale*, and
            // x (horizontal) variable with the *imageAspectRatio*            
            
            // 计算当前像素在三维空间中的方向向量，这里z为-1，表示三维空间中屏幕的位置是在z轴的-1处。
            // 屏幕坐标原点在三维空间中是左上角为(-1, 1, -1)，右下角为(1, -1, -1)
            Vector3f dir = Vector3f(x, y, -1); // Don't forget to normalize this direction!
            // 归一化光线方向
            dir = normalize(dir);

            // 将光线投射到场景中
            // 在光线追踪中，本质上是从光源发出光线，但那样比较复杂，由于光路可逆，也就是从眼睛发出光线，在场景中弹射，最终打到光源上。
            // 这里eye_pos是相机位置（起始点），dir是光线方向（方向），scene是场景（需要与场景中的物体进行碰撞），0是深度（避免光线在场景中无限弹射）
            // 光线追踪的实现castRay，返回三维向量，也就是颜色。并将颜色存储到framebuffer中。
            framebuffer[m++] = castRay(eye_pos, dir, scene, 0);
        }
        // 更新进度条
        UpdateProgress(j / (float)scene.height);
    }

    // save framebuffer to file
    // 光线追踪并考虑实时显示，
    // 放弃使用opencv库，反正我们只需维护 scene.height * scene.width 大小的vector3f数组，
    // 不考虑实时显示的话，opencv对于我们来说就是个累赘。
    // .ppm后缀的图像，windows下无法直接打开，vscode有个插件可以打开，但需要安装插件。
    // linux下可以直接查看。
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (char)(255 * clamp(0, 1, framebuffer[i].x));
        color[1] = (char)(255 * clamp(0, 1, framebuffer[i].y));
        color[2] = (char)(255 * clamp(0, 1, framebuffer[i].z));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
