#include "Cone.h"
#include "ResourceLayer/Factory.h"

bool Cone::rayIntersectShape(Ray &ray, int *primID, float *u, float *v) const {
    //* todo 完成光线与圆柱的相交 填充primId,u,v.如果相交，更新光线的tFar
    //* 1.光线变换到局部空间
    Ray localRay = transform.inverseRay(ray);
    //* 2.联立方程求解
    Point3f origin = localRay.origin;
    Vector3f direction = localRay.direction;
    Vector3f V{.0f, .0f, -1.f};
    Vector3f CO = origin - Point3f{.0f, .0f, height};   // 注意这里的O不是原点而是光线的起点
    float A = pow(dot(direction, V), 2) - cosTheta * cosTheta;
    float B = 2 * (dot(direction, V) * dot(CO, V) - dot(direction, CO) * cosTheta * cosTheta);
    float C = pow(dot(CO, V), 2) - dot(CO, CO) * cosTheta * cosTheta;
    float t[2];
    bool hasT = Quadratic(A, B, C, &t[0], &t[1]);
    if (!hasT)
        return false;
    //* 3.检验交点是否在圆锥范围内
    for (int i = 0; i < 2; i++) {
        float currT = t[i];
        Point3f point = localRay.at(currT);
        if (currT < ray.tNear || currT > ray.tFar)
            continue;
        if (point[2] < 0 || point[2] > height)  // 交点的z坐标不在圆柱高度范围内
            continue;
        float phi = 0;
        if (point[0] != 0) { 
            float tanPhi = point[1] / point[0];
            phi = atan(tanPhi);
            if (point[0] < 0)
                phi = PI + phi;
            else if (point[0] > 0 && phi < 0)
                phi = 2 * PI + phi;
        }
        else {
            if (point[1] > 0)
                phi = PI / 2;
            else if (point[1] < 0)
                phi = PI * 3 / 2;
        }
        if (phi > phiMax)
            continue;
        //* 4.更新ray的tFar,减少光线和其他物体的相交计算次数
        *primID = 0;
        *u = phi / phiMax;
        *v = point[2] / height;
        ray.tFar = currT;
        return true;
    }
    //* Write your code here.
    return false;
}

void Cone::fillIntersection(float distance, int primID, float u, float v, Intersection *intersection) const {
    /// ----------------------------------------------------
    //* todo 填充圆锥相交信息中的法线以及相交位置信息
    //* 1.法线可以先计算出局部空间的法线，然后变换到世界空间
    //* 2.位置信息可以根据uv计算出，同样需要变换
    float phi = u * phiMax;
    float z = v * height;
    Point3f point{.0f, .0f, z};
    point[0] = radius * (1 - v) * cos(phi);
    point[1] = radius * (1 - v) * sin(phi); 
    intersection->position = transform.toWorld(point);
    Point3f C{.0f, .0f, height};
    Vector3f MC = C - point;
    Point3f K{.0f, .0f, .0f};
    K[2] = height - MC.length() / cosTheta;
    Vector3f normal = point - K;
    intersection->normal = normalize(transform.toWorld(normal));
    //* Write your code here.
    /// ----------------------------------------------------


    intersection->shape = this;
    intersection->distance = distance;
    intersection->texCoord = Vector2f{u, v};
    Vector3f tangent{1.f, 0.f, .0f};
    Vector3f bitangent;
    if (std::abs(dot(tangent, intersection->normal)) > .9f) {
        tangent = Vector3f(.0f, 1.f, .0f);
    }
    bitangent = normalize(cross(tangent, intersection->normal));
    tangent = normalize(cross(intersection->normal, bitangent));
    intersection->tangent = tangent;
    intersection->bitangent = bitangent;
}

void Cone::uniformSampleOnSurface(Vector2f sample, Intersection *result, float *pdf) const {

}

Cone::Cone(const Json &json) : Shape(json) {
    radius = fetchOptional(json, "radius", 1.f);
    height = fetchOptional(json, "height", 1.f);
    phiMax = fetchOptional(json, "phi_max", 2 * PI);
    float tanTheta = radius / height;
    cosTheta = sqrt(1/(1+tanTheta * tanTheta));
    //theta = fetchOptional(json,)
    AABB localAABB = AABB(Point3f(-radius,-radius,0),Point3f(radius,radius,height));
    boundingBox = transform.toWorld(localAABB);
    boundingBox = AABB(Point3f(-100,-100,-100),Point3f(100,100,100));
}

REGISTER_CLASS(Cone, "cone")
