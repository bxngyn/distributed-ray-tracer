#ifndef PRIMITIVES_HPP_
#define PRIMITIVES_HPP_

#include "ray.hpp"
#include "light.hpp"
#include "material.hpp"
#include "SETTINGS.h"

class Primitive {
  protected:
    VEC3 color;
    Material material;

  public:
    Primitive(const VEC3 &color, Material material) 
      : color(color), material(material) {}
    ~Primitive() = default;

    virtual bool intersect(const Ray &r, VEC3 &hit) const = 0;
    virtual VEC3 normal(const VEC3 &hit) const = 0;
    virtual bool operator==(const Primitive &other) const = 0;

    VEC3 getColor() const { return color; }
    Material getMaterial() const { return material; }
};

class Sphere : public Primitive {
  private:
    VEC3 center;
    float radius;

    bool operator==(const Primitive &other) const override
    {
      const Sphere *otherSphere = dynamic_cast<const Sphere*>(&other);
      if (otherSphere)
      {
        return (center == otherSphere->center) && 
               (radius == otherSphere->radius) && 
               (color == otherSphere->color) &&
               (material == otherSphere->material);
      }
      return false;
    }

    bool intersect(const Ray &r, VEC3 &hit) const override
    {
      float A = r.getDirection().dot(r.getDirection());
      float B = 2.0 * r.getDirection().dot(r.getOrigin() - center);
      float C = (r.getOrigin() - center).dot(r.getOrigin() - center) - radius * radius;
      float discriminant = B * B - 4 * A * C;
      if (discriminant < 0)
      {
        // complete miss
        return false;
      }
      else
      {
        // if both positive, take smaller one
        // if both negative, miss
        // if one positive, one neg, pick positive one
        float t_pos = (-B + sqrt(B * B - 4 * A * C)) / 2 * A;
        float t_neg = (-B - sqrt(B * B - 4 * A * C)) / 2 * A;
        float t = 0.0;
        if (t_pos >= 0 && t_neg <= 0)
        {
          hit = r.getOrigin() + t_pos * r.getDirection();
        }
        else if (t_neg >= 0 && t_pos <= 0)
        {
          hit = r.getOrigin() + t_neg * r.getDirection();
        }
        else if (t_neg >= 0 && t_pos >= 0)
        {
          hit = r.getOrigin() + min(t_pos, t_neg) * r.getDirection();
        }
        else if (t_neg < 0 && t_pos < 0)
        {
          return false;
        }
        return true;
      }
    }

    VEC3 normal(const VEC3 &hit) const override
    {
      return (hit - center).normalized();
    }
    
  public:
    Sphere(const VEC3 &center, float radius, const VEC3 &color, Material material = Material::NORMAL)
      : center(center), radius(radius), Primitive(color, material) {}
    ~Sphere() = default;
};

class Triangle : public Primitive {
  private:
    VEC3 a;
    VEC3 b;
    VEC3 c;

    bool intersect(const Ray &r, VEC3 &hit) const override
    {
        VEC3 ab = b - a;
        VEC3 ac = c - a;
        VEC3 bc = c - b;
        VEC3 ca = a - c;

        VEC3 normal = this->normal(hit);

        if (abs(normal.dot(r.getDirection())) < 0.0001) { return false; }

        float d = -normal.dot(a);
        float t = -(normal.dot(r.getOrigin()) + d) / normal.dot(r.getDirection());
        if (t < 0) { return false; }

        hit = r.getOrigin() + t * r.getDirection();
    
        VEC3 ap = hit - a;
        if (normal.dot(ab.cross(ap)) < 0) { return false; }
    
        VEC3 bp = hit - b;
        if (normal.dot(bc.cross(bp)) < 0) { return false; }

        VEC3 cp = hit - c;
        if (normal.dot(ca.cross(cp)) < 0) { return false; }

        return true;
    }

    VEC3 normal(const VEC3 &hit) const override
    {
      return ((b - a).cross(c - a)).normalized();
    }

    bool operator==(const Primitive& other) const override
    {
        const Triangle* otherTriangle = dynamic_cast<const Triangle*>(&other);
        if (otherTriangle)
        {
            return (a == otherTriangle->a) &&
                   (b == otherTriangle->b) &&
                   (c == otherTriangle->c) &&
                   (color == otherTriangle->color) &&
                   (material == otherTriangle->material);
        }
        return false;
    }

  public:
    Triangle(const VEC3 &a, const VEC3 &b, const VEC3 &c, const VEC3 &color, Material material = Material::NORMAL)
      : a(a), b(b), c(c), Primitive(color, material) {}
    ~Triangle() = default;
    
    void rotateY(const VEC3 &center, float angle)
    {
      MATRIX3 rotation;
      rotation.setZero();
      rotation(0, 0) = cos(angle * M_PI / 180.0);
      rotation(1, 1) = 1.0;
      rotation(2, 2) = cos(angle * M_PI / 180.0);
      rotation(0, 2) = sin(angle * M_PI / 180.0);
      rotation(2, 0) = -sin(angle * M_PI / 180.0);

      a -= center;
      b -= center;
      c -= center;

      a = rotation * a;
      b = rotation * b;
      c = rotation * c;

      a += center;
      b += center;
      c += center;
    }
};

class Cylinder : public Primitive {
    private:
        VEC3 bot;
        VEC3 top;
        float radius;
    public:
        Cylinder(VEC3 bot, VEC3 top, float radius, VEC3 color, Material material = Material::NORMAL) : 
            bot(bot), top(top), radius(radius), Primitive(color, material) {}

        bool intersect(const Ray &r, VEC3 &hit) const override {
          VEC3 axis = bot - top;
          VEC3 oriTop = r.getOrigin() - top;
          float A = axis.dot(axis) - axis.dot(r.getDirection()) * axis.dot(r.getDirection());
          float B = axis.dot(axis) * oriTop.dot(r.getDirection()) - axis.dot(oriTop) * axis.dot(r.getDirection());
          float C = axis.dot(axis) * oriTop.dot(oriTop) - axis.dot(oriTop) * axis.dot(oriTop) - pow(radius, 2) * axis.dot(axis);
          float det = pow(B, 2) - A * C;
          if (det < 0.0) { return false; }
          det = sqrt(det);
          float t = (-B - det) / A;

          float y = axis.dot(oriTop) + t * axis.dot(r.getDirection());
          if (y > 0.0)
          {
            if (y < axis.dot(axis))
            {
              hit = ((oriTop + t * r.getDirection() - axis * y / axis.dot(axis)) / radius).normalized();
              return true;
            }
          }

          if (y < 0.0)
          {
            t = -axis.dot(oriTop) / axis.dot(r.getDirection());
          }
          else
          {
            t = (axis.dot(axis) - axis.dot(oriTop)) / axis.dot(r.getDirection());
          }

          if (abs(B + A * t) < det)
          {
            if (y > 0.0)
            {
              hit = axis / sqrt(axis.dot(axis));
            }
            else
            {
              hit = -axis / sqrt(axis.dot(axis));
            }
            return true;
          }
          return false;
        }
        
        VEC3 normal(const VEC3 &hit) const override {
            VEC3 point = hit - top;
            VEC3 axis = bot - top;
            float projection = point.dot(axis) / axis.dot(axis);
            VEC3 n = (point - axis * projection) / radius;
            return n.normalized();
        }

        bool operator==(const Primitive &other) const override {
          const Cylinder* otherCylinder = dynamic_cast<const Cylinder*>(&other);
          if (otherCylinder)
          {
              return (bot == otherCylinder->bot) &&
                    (top == otherCylinder->top) &&
                    (radius == otherCylinder->radius) &&
                    (color == otherCylinder->color) &&
                    (material == otherCylinder->material);
          }
          return false;
        }
};

#endif