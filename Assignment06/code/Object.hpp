//
// Created by LEI XU on 5/13/19.
//
#pragma once
#ifndef RAYTRACING_OBJECT_H
#define RAYTRACING_OBJECT_H

#include "Vector.hpp"
#include "global.hpp"
#include "Bounds3.hpp"
#include "Ray.hpp"
#include "Intersection.hpp"

class Object
{
public:
    Object() {}
    virtual ~Object() {}
    virtual bool intersect(const Ray& ray) = 0;
    virtual bool intersect(const Ray& ray, float &, uint32_t &) const = 0;
    virtual Intersection getIntersection(Ray _ray) = 0;
    virtual void getSurfaceProperties(const Vector3f& P, const Vector3f& I,
                                    const uint32_t& index, const Vector2f& uv,
                                    Vector3f& N, Vector2f& st) const = 0;
    virtual Vector3f evalDiffuseColor(const Vector2f& st) const =0;
    virtual Bounds3 getBounds()=0;
};



#endif //RAYTRACING_OBJECT_H
