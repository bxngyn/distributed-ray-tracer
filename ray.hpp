#ifndef RAY_HPP_
#define RAY_HPP_
#include "SETTINGS.h"

VEC3 eye(-6, 0.5, 1);
VEC3 lookingAt(5, 0.5, 1);
VEC3 up(0,1,0);

int xRes = 640;
int yRes = 480;
float near = 1.0;
float fovy = 45.0 * (M_PI / 180.0);
float aspect = 4.0 / 3.0;

VEC3 offset = VEC3(1e-5, 1e-5, 1e-5);

class Ray {
  private:
    VEC3 origin;
    VEC3 direction;

  public:
    Ray() = default;
    ~Ray() = default;

    Ray(const VEC3 &origin, const VEC3 &dir)
      : origin(origin), direction(dir) {}

    VEC3 getOrigin() const { return origin; }
    VEC3 getDirection() const { return direction; }

    static Ray generateEyeRay(int x, int y)
    {
      float t = near * tan(fovy / 2.0);
      float b = -t;
      float r = t * aspect;
      float l = -r;
      swap(l, r);

      VEC3 s = VEC3(-(l + ((r - l) * (x + 0.5)) / xRes),
                    (b + ((t - b) * (yRes - y + 0.5)) / yRes),
                    1.0);
      VEC3 origin = eye;
      VEC3 d = (s - eye).normalized(); 
      return Ray(origin, d);
    }

    bool canRefract(VEC3 normal, const VEC3 &hit, float ior_ratio, bool inside, Ray &out, float &theta, float &phi) const
    {
      if (inside) { normal = -normal; }

      float cos_theta = -direction.dot(normal); // ?
      float sin_theta = sqrt(1.0 - cos_theta * cos_theta);
      float sin_phi = ior_ratio * sin_theta;

      if ((1.0 - (sin_phi * sin_phi)) < 0) { return false; }
    
      float cos_phi = sqrt(1.0 - sin_phi * sin_phi);
      theta = acos(cos_theta);
      phi = acos(cos_phi);

      VEC3 b = (direction + normal * cos_theta) / sin_theta;
      VEC3 outDir = (sin_phi * b - cos_phi * normal).normalized();
      out = Ray(hit + outDir.cwiseProduct(offset), outDir);;
      return true;
    }
};

#endif