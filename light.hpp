#ifndef LIGHT_HPP_
#define LIGHT_HPP_
#include "SETTINGS.h"

class Light {
protected:
    VEC3 color;

public:
    Light(const VEC3 &c) : color(c) {}
    virtual ~Light() = default;

    VEC3 getColor() const { return color; }
    virtual VEC3 sampleLight(const VEC3 &hit) const = 0;
    virtual VEC3 getDirection(const VEC3 &hit) const = 0;
    virtual bool isAreaLight() const = 0;
};

class PointLight : public Light {
private:
    VEC3 position;

public:
    PointLight(const VEC3 &p, const VEC3 &c)
        : Light(c), position(p) {}

    VEC3 sampleLight(const VEC3 & /*hit*/) const override {
        return position;
    }
    VEC3 getDirection(const VEC3 &hit) const override {
        return (position - hit).normalized();
    }
    bool isAreaLight() const override {
      return false;
    }
};

class AreaLight : public Light {
private:
    VEC3 corner;
    VEC3 uVec, vVec;

public:
    AreaLight(const VEC3 &c, const VEC3 &corner, const VEC3 &uVec, const VEC3 &vVec)
        : Light(c), corner(corner), uVec(uVec), vVec(vVec) {}

    VEC3 sampleLight(const VEC3 & /*hit*/) const override {
        float u = float(rand()) / RAND_MAX;
        float v = float(rand()) / RAND_MAX;
        return corner + u * uVec + v * vVec;
    }
    VEC3 getDirection(const VEC3 &hit) const override {
        VEC3 samplePoint = sampleLight(hit);
        return (samplePoint - hit).normalized();
    }
    bool isAreaLight() const override {
      return true;
    }
};

#endif