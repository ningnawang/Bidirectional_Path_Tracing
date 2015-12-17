#include "light.h"

#include <iostream>

#include "../sampler.h"

namespace CMU462 { namespace StaticScene {

// Directional Light //

DirectionalLight::DirectionalLight(const Spectrum& rad,
                                   const Vector3D& lightDir)
    : radiance(rad) {
  dirToLight = -lightDir.unit();
}

Spectrum DirectionalLight::sample_L(const Vector3D& p, Vector3D* wi,
                                    float* distToLight, float* pdf) const {
  *wi = dirToLight;
  *distToLight = INF_D;
  *pdf = 1.0;
  return radiance;
}

Spectrum DirectionalLight::sample_light_ray(Ray *lightRay, float *pdf) const {

  return Spectrum();
  
  const Vector3D& lightRay_d = -dirToLight;
  *lightRay = Ray(hemisphere_sampler.get_sample(), lightRay_d);
  *pdf = 1.0;
  return radiance;
}


// Infinite Hemisphere Light //

InfiniteHemisphereLight::InfiniteHemisphereLight(const Spectrum& rad)
    : radiance(rad) {
  sampleToWorld[0] = Vector3D(1,  0,  0);
  sampleToWorld[1] = Vector3D(0,  0, -1);
  sampleToWorld[2] = Vector3D(0,  1,  0);
}

Spectrum InfiniteHemisphereLight::sample_L(const Vector3D& p, Vector3D* wi,
                                           float* distToLight,
                                           float* pdf) const {
  *wi = sampleToWorld * sampler.get_sample();
  *distToLight = INF_D;
  *pdf = 1.0 / (2.0 * M_PI);
  return radiance;
}

Spectrum InfiniteHemisphereLight::sample_light_ray(Ray *lightRay, float *pdf) const {
  // return Spectrum();

  const Vector3D& lightRay_d = -sampleToWorld * sampler.get_sample(); 
  // *lightRay = Ray(sampleToWorld * sampler.get_sample(), lightRay_d);
  *lightRay = Ray(sampleToWorld * sampler.get_sample(), lightRay_d);
  *pdf = 1.0 / (2.0 * M_PI);
  return radiance;
}

// Point Light //

PointLight::PointLight(const Spectrum& rad, const Vector3D& pos) :
  radiance(rad), position(pos) { }

Spectrum PointLight::sample_L(const Vector3D& p, Vector3D* wi,
                             float* distToLight,
                             float* pdf) const {
  Vector3D d = position - p;
  double dist = d.norm();
  *wi = d / dist;
  *distToLight = dist;
  *pdf = 1.0;
  return radiance;
}

Spectrum PointLight::sample_light_ray(Ray *lightRay, float *pdf) const {
  return Spectrum();

  Vector3D rd = hemisphere_sampler.get_sample();
  const Vector3D& lightRay_o = position;
  const Vector3D& lightRay_d = lightRay_o - rd;
  float cosTheta = dot(lightRay_d, lightRay_d);
  float sqDist = lightRay_d.norm2();
  float dist = sqrt(sqDist);
  Vector3D wi = lightRay_d / dist;
  *lightRay = Ray(lightRay_o, wi);
  *pdf = 1.0;
  return radiance;

}

// Spot Light //

SpotLight::SpotLight(const Spectrum& rad, const Vector3D& pos,
                     const Vector3D& dir, float angle) {

}

Spectrum SpotLight::sample_L(const Vector3D& p, Vector3D* wi,
                             float* distToLight, float* pdf) const {
  return Spectrum();
}

Spectrum SpotLight::sample_light_ray(Ray *lightRay, float *pdf) const {
  return Spectrum();
}


// Area Light //

AreaLight::AreaLight(const Spectrum& rad,
                     const Vector3D& pos,   const Vector3D& dir,
                     const Vector3D& dim_x, const Vector3D& dim_y)
  : radiance(rad), position(pos), direction(dir),
    dim_x(dim_x), dim_y(dim_y), area(dim_x.norm() * dim_y.norm()) { }


Spectrum AreaLight::sample_L(const Vector3D& p, Vector3D* wi,
                             float* distToLight, float* pdf) const {

  const Vector2D& sample = sampler.get_sample() - Vector2D(0.5f, 0.5f);
  const Vector3D& d = position + sample.x * dim_x + sample.y * dim_y - p;
  float cosTheta = dot(d, direction);
  float sqDist = d.norm2();
  float dist = sqrt(sqDist);
  *wi = d / dist;
  *distToLight = dist;
  *pdf = sqDist / (area * fabs(cosTheta));
  return cosTheta < 0 ? radiance : Spectrum();
};

Spectrum AreaLight::sample_light_ray(Ray *lightRay, float *pdf) const {

  Vector3D rd = hemisphere_sampler.get_sample();
  const Vector2D& sample = sampler.get_sample() - Vector2D(0.5f, 0.5f);
  const Vector3D& lightRay_o = position + sample.x * dim_x + sample.y * dim_y;
  const Vector3D& lightRay_d = lightRay_o - rd;
  float cosTheta = dot(lightRay_d, direction);
  float sqDist = lightRay_d.norm2();
  float dist = sqrt(sqDist);
  Vector3D wi = lightRay_d / dist;
  *lightRay = Ray(lightRay_o, wi);
  *pdf = sqDist / (area * fabs(cosTheta));
  return cosTheta < 0 ? radiance : Spectrum();
  // return radiance;

};

// Sphere Light //

SphereLight::SphereLight(const Spectrum& rad, const SphereObject* sphere) {

}

Spectrum SphereLight::sample_L(const Vector3D& p, Vector3D* wi,
                               float* distToLight, float* pdf) const {

  return Spectrum();
}

Spectrum SphereLight::sample_light_ray(Ray *lightRay, float *pdf) const {
  return Spectrum();
}

// Mesh Light

MeshLight::MeshLight(const Spectrum& rad, const Mesh* mesh) {

}

Spectrum MeshLight::sample_L(const Vector3D& p, Vector3D* wi,
                             float* distToLight, float* pdf) const {
  return Spectrum();
}


Spectrum MeshLight::sample_light_ray(Ray *lightRay, float *pdf) const {
  return Spectrum();
}

} // namespace StaticScene
} // namespace CMU462
