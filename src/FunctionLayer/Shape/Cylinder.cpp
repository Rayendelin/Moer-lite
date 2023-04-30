#include "Cylinder.h"
#include "ResourceLayer/Factory.h"
bool Cylinder::rayIntersectShape(Ray &ray, int *primID, float *u, float *v) const {
    //* todo 完成光线与圆柱的相交 填充primId,u,v.如果相交，更新光线的tFar
    //* 1.光线变换到局部空间
    Ray localRay = transform.inverseRay(ray);
    //* 2.联立方程求解
    Point3f origin = localRay.origin;
    Vector3f direction = localRay.direction;
    float A = direction[0] * direction[0] + direction[1] * direction[1];
    float B = 2 * direction[0] * origin[0] + 2 * direction[1] * origin[1];
    float C = origin[0] * origin[0] + origin[1] * origin[1] - radius * radius;
    float t[2];
    bool hasT = Quadratic(A, B, C, &t[0], &t[1]);
    if (!hasT)
        return false;
    //* 3.检验交点是否在圆柱范围内
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

void Cylinder::fillIntersection(float distance, int primID, float u, float v, Intersection *intersection) const {
    /// ----------------------------------------------------
    //* todo 填充圆柱相交信息中的法线以及相交位置信息
    //* 1.法线可以先计算出局部空间的法线，然后变换到世界空间
    float phi = u * phiMax;
    Vector3f normal{.0f, .0f, .0f};
    normal[0] = cos(phi);
    normal[1] = sin(phi);
    intersection->normal = normalize(transform.toWorld(normal));
    //* 2.位置信息可以根据uv计算出，同样需要变换
    float z = v * height;
    Point3f point{.0f, .0f, z};
    point[0] = radius * cos(phi);
    point[1] = radius * sin(phi); 
    intersection->position = transform.toWorld(point);
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

void Cylinder::uniformSampleOnSurface(Vector2f sample, Intersection *result, float *pdf) const {

}

Cylinder::Cylinder(const Json &json) : Shape(json) {
    radius = fetchOptional(json,"radius",1.f);
    height = fetchOptional(json,"height",1.f);
    phiMax = fetchOptional(json,"phi_max",2 * PI);
    AABB localAABB = AABB(Point3f(-radius,-radius,0),Point3f(radius,radius,height));
    boundingBox = transform.toWorld(localAABB);
}

REGISTER_CLASS(Cylinder,"cylinder")
