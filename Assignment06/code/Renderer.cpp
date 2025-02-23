//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    // 帧缓冲
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    // 计算缩放比例
    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;

    // 相机位置
    Vector3f eye_pos(-1, 5, 10);

    // 累计遍历像素总数
    int m = 0;

    // 遍历所有像素
    for (uint32_t i = 0; i < scene.height; ++i) {
        for (uint32_t j = 0; j < scene.width; ++j) {

            // generate primary ray direction 生成主光线方向
            float x = (2 * ((j + 0.5f) / scene.width) - 1.0f) * imageAspectRatio * 1 * scale;
            float y = (1.0f - 2 * ((i + 0.5f) / scene.height)) * 1 * scale;

            // TODO: Find the x and y positions of the current pixel to get the direction vector that passes through it.
            // 获取当前像素的x和y位置，得到通过该像素的方向向量。
            // Also, don't forget to multiply both of them with the variable *scale*, and x (horizontal) variable with the *imageAspectRatio*
            // 不要忘记将它们与变量*scale*相乘，并将x（水平）变量与*imageAspectRatio*相乘。
            // Don't forget to normalize this direction!
            // 不要忘记归一化化这个方向！

            // 生成主光线方向，这里z为-1，表示相机在z轴的正方向，射线向z轴的负方向发射
            Vector3f dir = Vector3f(x, y, -1); // Don't forget to normalize this direction!
            dir = normalize(dir); // 方向向量

            // 光线封装成一个Ray对象，origin是相机位置，dir是光线方向，t是时间，这里t默认为0。
            Ray ray(eye_pos, dir);
            // 光线投射，返回颜色
            framebuffer[m++] = scene.castRay(ray, 0);

        }
        UpdateProgress(i / (float)scene.height);
    }
    UpdateProgress(1.f);

    // save framebuffer to file 保存帧缓冲区到文件
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].x));
        color[1] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].y));
        color[2] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].z));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
