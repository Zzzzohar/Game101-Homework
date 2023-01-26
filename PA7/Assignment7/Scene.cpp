//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
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

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray& ray, int depth) const
{
    
    // TODO Implement Path Tracing Algorithm here
    
    Vector3f dir{ 0,0,0 }, indir{ 0,0,0 };

    Intersection inter_P = intersect(ray);
    if (!inter_P.happened)
    {
        return Vector3f();
    }
    //如果有交点，但是是光源直接输出 emission

    if (inter_P.m->hasEmission())
    {
        return inter_P.m->getEmission();
    }


    //对光源进行采样

    Intersection inter_Lghit;
    float pdf_light;
    sampleLight(inter_Lghit, pdf_light);

    //准备着色点相关数据

    Vector3f& p = inter_P.coords;
    Vector3f& N = inter_P.normal;
    Vector3f wo = (ray.origin - p).normalized();
    Material* m = inter_P.m;
    // 准备光源相关数据

    Vector3f& x = inter_Lghit.coords;
    Vector3f& NN = inter_Lghit.normal;
    Vector3f& emit = inter_Lghit.emit;
    Vector3f ws = (x - p).normalized();
    float dis = (x - p).norm();

    // Shoot a ray from p to x 检测是否有遮挡

    Ray light_to_obj(p, ws);
    Intersection i = intersect(light_to_obj);
    // dis>i.distance就说明有遮挡 反之没有，EPSILON用于精度处理

    if (i.distance - dis > -EPSILON)
    {
        dir = emit * m->eval(wo, ws, N) * dotProduct(ws, N) * dotProduct(-ws, NN) / (dis * dis * pdf_light);
    }

    //RR

    float f = get_random_float();
    if (f < RussianRoulette)
    {
        Vector3f wi = m->sample(wo, N).normalized();
        // 二次反射用的射线

        Ray r(p, wi);
        Intersection i = intersect(r);

        if (i.happened && !i.m->hasEmission())
        {
            indir = castRay(r, depth + 1) * m->eval(wo, wi, N) * dotProduct(wi, N) / m->pdf(wo, wi, N) / RussianRoulette;
        }
    }
    return dir + indir;
}
