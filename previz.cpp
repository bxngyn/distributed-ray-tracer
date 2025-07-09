//////////////////////////////////////////////////////////////////////////////////
// This is a front end for a set of viewer clases for the Carnegie Mellon
// Motion Capture Database: 
//    
//    http://mocap.cs.cmu.edu/
//
// The original viewer code was downloaded from:
//
//   http://graphics.cs.cmu.edu/software/mocapPlayer.zip
//
// where it is credited to James McCann (Adobe), Jernej Barbic (USC),
// and Yili Zhao (USC). There are also comments in it that suggest
// and Alla Safonova (UPenn) and Kiran Bhat (ILM) also had a hand in writing it.
//
//////////////////////////////////////////////////////////////////////////////////
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <float.h>
// #include <omp.h>
#include <random>
#include "SETTINGS.h"
#include "skeleton.h"
#include "displaySkeleton.h"
#include "motion.h"
#include "primitive.hpp"
#include "light.hpp"

using namespace std;

// Stick-man classes
DisplaySkeleton displayer;    
Skeleton* skeleton;
Motion* motion;

// VEC3 eye(-6, 0.5, 1);
// VEC3 lookingAt(5, 0.5, 1);
// VEC3 up(0,1,0);
int phong_exp = 10;

// scene geometry
vector<Primitive *> prims;
vector<Light *> lights;

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void writePPM(const string& filename, int& xRes, int& yRes, const float* values)
{
  int totalCells = xRes * yRes;
  unsigned char* pixels = new unsigned char[3 * totalCells];
  for (int i = 0; i < 3 * totalCells; i++)
    pixels[i] = values[i];

  FILE *fp;
  fp = fopen(filename.c_str(), "wb");
  if (fp == NULL)
  {
    cout << " Could not open file \"" << filename.c_str() << "\" for writing." << endl;
    cout << " Make sure you're not trying to write from a weird location or with a " << endl;
    cout << " strange filename. Bailing ... " << endl;
    exit(0);
  }

  fprintf(fp, "P6\n%d %d\n255\n", xRes, yRes);
  fwrite(pixels, 1, totalCells * 3, fp);
  fclose(fp);
  delete[] pixels;
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
Primitive *closest(const Ray &r)
{
  float minDist = FLT_MAX;
  Primitive *closestObject = nullptr;

  for (Primitive *prim : prims)
  {
      VEC3 hit;
      if (prim->intersect(r, hit))
      {
          float dist = (hit - r.getOrigin()).norm();
          if (dist < minDist)
          {
              minDist = dist;
              closestObject = prim;
          }
      }
  }

  return closestObject;
}

bool intersectScene(const Ray &r, const vector<Primitive*> &scn, const Primitive *ignore)
{
  for (const Primitive *obj : scn)
  {
    if (*obj == *ignore)
    {
      continue;
    }
    VEC3 tmp;
    if (obj->intersect(r, tmp))
    {
        return true;
    }
  }
  return false;
}


VEC3 rayColor(const Ray &r) 
{
  VEC3 color = VEC3(1,1,1);
  VEC3 hit = VEC3(0, 0, 0);
  Primitive *prim = closest(r);
  if (prim && prim->intersect(r, hit)) {
    color = prim->getColor();
  }
  return color;
}

VEC3 BRDF(Ray reflection, VEC3 normal, float theta, float phi)
{
  VEC3 dir = reflection.getDirection();
  VEC3 x = cos(theta) * sin(phi) * dir.cross(normal);
  VEC3 y = sin(theta) * sin(phi) * dir.cross(dir.cross(normal));
  VEC3 jitter = reflection.getDirection() + x + y;
  return jitter.normalized();
}

VEC3 rayColor(const Ray &r, int depth)
{
  if (depth > 5)
  {
    return VEC3(0.0, 0.0, 0.0);
  }
  
  const int numSamples = 48;
  VEC3 color = VEC3(0, 0, 0);
  VEC3 ambient = VEC3(0.5, 0.5, 0.5);
  VEC3 hit = VEC3(0, 0, 0);

  Primitive *prim = closest(r);      
  if (prim && prim->intersect(r, hit))
  {
    VEC3 normal = prim->normal(hit);
    VEC3 view = (r.getOrigin() - hit).normalized();
    if (prim->getMaterial() == Material::GLOSSY)
    {
      Ray reflection = Ray(hit + normal.cwiseProduct(offset),
                            -view + 2.0 * view.dot(normal) * normal); // reflection from view
      for (int i = 0; i < numSamples; i++)
      {
        double roughness = 0.99;
        double rand_theta = acos(pow((1 - (float(rand()) / RAND_MAX)), roughness));
        double rand_phi = 2.0 * M_PI * (float(rand()) / RAND_MAX);
        VEC3 dir = BRDF(reflection, normal, rand_theta, rand_phi);
        Ray jitter = Ray(reflection.getOrigin(), dir);
        if (jitter.getDirection().dot(normal) < 0.0) { continue; }
        color += rayColor(jitter, depth + 1);
      }
      color /= numSamples;
    }
    else if (prim->getMaterial() == Material::MIRROR)
    {
      Ray reflection = Ray(hit + normal.cwiseProduct(offset),
                            -view + 2.0 * view.dot(normal) * normal); // reflection from view
      color = rayColor(reflection, depth + 1);
    }
    else
    {
      for (Light *lamp : lights)
      {
        if (lamp->isAreaLight())
        {
          float visibility = 0.0;
          VEC3 diffuseContribution = VEC3(0.0, 0.0, 0.0);
          VEC3 specularContribution = VEC3(0.0, 0.0, 0.0);
          
          for (int i = 0; i < numSamples; i++)
          {
            VEC3 samplePoint = lamp->sampleLight(hit);
            VEC3 ell = (samplePoint - hit).normalized();
            Ray shadow = Ray(hit + normal.cwiseProduct(offset), ell);

            if (!intersectScene(shadow, prims, prim))
            {
              visibility += 1.0;
              diffuseContribution += lamp->getColor() * max(0.0, normal.dot(ell));
              
              VEC3 reflection = -ell + 2.0 * ell.dot(normal) * normal;
              specularContribution += lamp->getColor() * pow(max(0.0, reflection.dot(view)), phong_exp);
            }
          }
          visibility /= numSamples;
          color += visibility * prim->getColor().cwiseProduct(ambient + diffuseContribution / numSamples + specularContribution / numSamples);
        }
        else
        {
          VEC3 ell = lamp->getDirection(hit);
          Ray reflection = Ray(hit, -ell + 2.0 * ell.dot(normal) * normal); // reflection from light
          color += prim->getColor().cwiseProduct(ambient + lamp->getColor() * max(0.0, normal.dot(ell))
                                                         + lamp->getColor() * pow(max(0.0, reflection.getDirection().dot(view)),   phong_exp));
        }
      }
    }
  }
  return color;
}


//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
float clamp(float value)
{
  if (value < 0.0)      return 0.0;
  else if (value > 1.0) return 1.0;
  return value;
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void renderImage(int& xRes, int& yRes, const string& filename) 
{
  // allocate the final image
  const int totalCells = xRes * yRes;
  float* ppmOut = new float[3 * totalCells];

  // compute image plane
  const float halfY = (lookingAt - eye).norm() * tan(45.0f / 360.0f * M_PI);
  const float halfX = halfY * 4.0f / 3.0f;

  const VEC3 cameraZ = (lookingAt - eye).normalized();
  const VEC3 cameraX = up.cross(cameraZ).normalized();
  const VEC3 cameraY = cameraZ.cross(cameraX).normalized();

  for (int y = 0; y < yRes; y++) 
    for (int x = 0; x < xRes; x++) 
    {
      const float ratioX = 1.0f - x / float(xRes) * 2.0f;
      const float ratioY = 1.0f - y / float(yRes) * 2.0f;
      const VEC3 rayHitImage = lookingAt + 
                               ratioX * halfX * cameraX +
                               ratioY * halfY * cameraY;
      const VEC3 rayDir = (rayHitImage - eye).normalized();
      const Ray r = Ray(eye, rayDir);
      // get the color
      VEC3 color = rayColor(r, 0);

      // set, in final image
      ppmOut[3 * (y * xRes + x)] = clamp(color[0]) * 255.0f;
      ppmOut[3 * (y * xRes + x) + 1] = clamp(color[1]) * 255.0f;
      ppmOut[3 * (y * xRes + x) + 2] = clamp(color[2]) * 255.0f;
    }
  writePPM(filename, xRes, yRes, ppmOut);

  delete[] ppmOut;
}

// anti-aliasing
// void renderImage(int& xRes, int& yRes, const string& filename) 
// {
//   // allocate the final image
//   const int totalCells = xRes * yRes;
//   float* ppmOut = new float[3 * totalCells];

//   // compute image plane
//   const float halfY = (lookingAt - eye).norm() * tan(45.0f / 360.0f * M_PI);
//   const float halfX = halfY * 4.0f / 3.0f;

//   const VEC3 cameraZ = (lookingAt - eye).normalized();
//   const VEC3 cameraX = up.cross(cameraZ).normalized();
//   const VEC3 cameraY = cameraZ.cross(cameraX).normalized();
//   const int numSamples = 9;
//   const int gridSize = sqrt(numSamples);

//   for (int y = 0; y < yRes; y++) 
//     for (int x = 0; x < xRes; x++) 
//     {
//       VEC3 color = VEC3(0,0,0);
//       for (int i = 0; i < gridSize; i++) {
//         for (int j = 0; j < gridSize; j++) {
//           float subPixelX = (x + (i + float(rand()) / RAND_MAX) / gridSize);
//           float subPixelY = (y + (j + float(rand()) / RAND_MAX) / gridSize);
//           const float ratioX = 1.0f - subPixelX / float(xRes) * 2.0f;
//           const float ratioY = 1.0f - subPixelY / float(yRes) * 2.0f;
//           const VEC3 rayHitImage = lookingAt + 
//                                 ratioX * halfX * cameraX +
//                                 ratioY * halfY * cameraY;
//           const VEC3 rayDir = (rayHitImage - eye).normalized();
//           const Ray r = Ray(eye, rayDir);
//           color += rayColor(r, 0);
//         }
//       }
//       color /= numSamples;

//       ppmOut[3 * (y * xRes + x)] = clamp(color[0]) * 255.0f;
//       ppmOut[3 * (y * xRes + x) + 1] = clamp(color[1]) * 255.0f;
//       ppmOut[3 * (y * xRes + x) + 2] = clamp(color[2]) * 255.0f;
//     }
//   writePPM(filename, xRes, yRes, ppmOut);

//   delete[] ppmOut;
// }

//////////////////////////////////////////////////////////////////////////////////
// Load up a new motion captured frame
//////////////////////////////////////////////////////////////////////////////////
void setSkeletonsToSpecifiedFrame(int frameIndex)
{
  if (frameIndex < 0)
  {
    printf("Error in SetSkeletonsToSpecifiedFrame: frameIndex %d is illegal.\n", frameIndex);
    exit(0);
  }
  if (displayer.GetSkeletonMotion(0) != NULL)
  {
    int postureID;
    if (frameIndex >= displayer.GetSkeletonMotion(0)->GetNumFrames())
    {
      cout << " We hit the last frame! You might want to pick a different sequence. " << endl;
      postureID = displayer.GetSkeletonMotion(0)->GetNumFrames() - 1;
    }
    else 
      postureID = frameIndex;
    displayer.GetSkeleton(0)->setPosture(* (displayer.GetSkeletonMotion(0)->GetPosture(postureID)));
  }
}


void setLights()
{

  lights.push_back(new AreaLight(VEC3(1, 1, 1), VEC3(10, -11, -2), VEC3(5, 0, 0), VEC3(0, 0, 5)));
  lights.push_back(new AreaLight(VEC3(1, 1, 1), VEC3(-10, 11, 2), VEC3(5, 0, 0), VEC3(0, 0, 5)));
  // lights.push_back(new PointLight(VEC3(0, 100, 0), VEC3(1, 1, 1)));
  // lights.push_back(new AreaLight(VEC3(0.5, 0.5, 0.5), VEC3(3, 4, -3), VEC3(5, 0, 0), VEC3(0, 0, 5)));
  // lights.push_back(new AreaLight(VEC3(0.5, 0.5, 0.5), VEC3(-3, -4, 3), VEC3(5, 0, 0), VEC3(0, 0, 5)));
  // lights.push_back(new PointLight(VEC3(-1.75, 0.01, 1.75), VEC3(1, 1, 1)));
  // lights.push_back(new AreaLight(VEC3(0.25, 0.25, 0.25), VEC3(10, 10, 0), VEC3(5, 0, 0), VEC3(0, 0, 5)));
  // lights.push_back(new AreaLight(VEC3(0.25, 0.25, 0.25), VEC3(0, -10, -5), VEC3(5, 0, 0), VEC3(0, 0, 5)));
  // lights.push_back(new AreaLight(VEC3(0.25, 0.25, 0.25), VEC3(0, -10, 5), VEC3(5, 0, 0), VEC3(0, 0, 5)));
  // lights.push_back(new AreaLight(VEC3(0.25, 0.25, 0.25), VEC3(-10, 10, 0), VEC3(5, 0, 0), VEC3(0, 0, 5)));
 
  // lights.push_back(new PointLight(VEC3(5.75, 10, 1), VEC3(0.25, 0.25, 0.25)));
  // lights.push_back(new PointLight(VEC3(5.75, -5, 0), VEC3(0.25, 0.25, 0.25)));
  // lights.push_back(new PointLight(VEC3(-5.75, -5, 0), VEC3(0.25, 0.25, 0.25)));
  // lights.push_back(new PointLight(VEC3(-1, 5, 4.75), VEC3(0.25, 0.25, 0.25)));
  // lights.push_back(new PointLight(VEC3(-1, -5, -4.75), VEC3(0.25, 0.25, 0.25)));
  // lights.push_back(new PointLight(VEC3(5.75, -10, 1), VEC3(0.25, 0.25, 0.25)));
  // VEC3 v1 = VEC3(-1.75, 0, -1.75);
  // VEC3 v2 = VEC3(1.75, 0, -1.75);
  // VEC3 v3 = VEC3(1.75, 0, 1.75);
  // VEC3 v4 = VEC3(-1.75, 0, 1.75);

  
  // lights.push_back(new AreaLight(VEC3(1, 1, 1), VEC3(-1.75, 1, -1.75), VEC3(3.5, 0, 0), VEC3(0, 0, 3.5)));
  // lights.push_back(new AreaLight(VEC3(5.0, 5.0, 5.0), VEC3(-5, -1.0, -5), VEC3(15, 0, 0), VEC3(0, 0, 15)));
  // lights.push_back(new AreaLight(VEC3(5.0, 5.0, 5.0), VEC3(-1.75, 0.25, -1.75), VEC3(2.0, 0, 0), VEC3(0, 0, 2.0)));
  // lights.push_back(new AreaLight(VEC3(5.0, 5.0, 5.0), VEC3(1.75, 0.25, -1.75), VEC3(2.0, 0, 0), VEC3(0, 0, 2.0)));
  // lights.push_back(new AreaLight(VEC3(5.0, 5.0, 5.0), VEC3(0, 0.5, 0), VEC3(2.0, 0, 0), VEC3(0, 0, 2.0)));
  // lights.push_back(new AreaLight(VEC3(5.0, 5.0, 5.0), VEC3(1.75, 0.25, 1.75), VEC3(2.0, 0, 0), VEC3(0, 0, 2.0)));
  // lights.push_back(new AreaLight(VEC3(5.0, 5.0, 5.0), VEC3(-1.75, 0.25, 1.75), VEC3(2.0, 0, 0), VEC3(0, 0, 2.0)));

  // lights.push_back(new AreaLight(VEC3(5.0, 5.0, 5.0), VEC3(-2.25, -0.25, -2.25), VEC3(2.0, 0, 0), VEC3(0, 0, 2.0)));
  // lights.push_back(new AreaLight(VEC3(5.0, 5.0, 5.0), VEC3(2.25, -0.25, -2.25), VEC3(2.0, 0, 0), VEC3(0, 0, 2.0)));
  // // lights.push_back(new AreaLight(VEC3(5.0, 5.0, 5.0), VEC3(0, -0.5, 0), VEC3(2.0, 0, 0), VEC3(0, 0, 2.0)));
  // lights.push_back(new AreaLight(VEC3(5.0, 5.0, 5.0), VEC3(2.25, -0.25, 2.25), VEC3(2.0, 0, 0), VEC3(0, 0, 2.0)));
  // lights.push_back(new AreaLight(VEC3(5.0, 5.0, 5.0), VEC3(-2.25, -0.25, 2.25), VEC3(2.0, 0, 0), VEC3(0, 0, 2.0)));
}
//////////////////////////////////////////////////////////////////////////////////
// Build floor
//////////////////////////////////////////////////////////////////////////////////
void buildFloor()
{
// left: 0.420849 0.0940682 -0.390806  0.609194
// right: 0.534637 0.0519082 -0.394318  0.609194
  // y: up down
  // z: left right
  // x: forward back
  // top right, bottom right, bottom left 
  // top
  VEC3 v1 = VEC3(-1.75, 0, -1.75);
  VEC3 v2 = VEC3(1.75, 0, -1.75);
  VEC3 v3 = VEC3(1.75, 0, 1.75);
  VEC3 v4 = VEC3(-1.75, 0, 1.75);

  VEC3 v1_bottom = VEC3(v1[0], v1[1] - 5, v1[2]);
  VEC3 v2_bottom = VEC3(v2[0], v2[1] - 5, v2[2]);
  VEC3 v3_bottom = VEC3(v3[0], v3[1] - 5, v3[2]);
  VEC3 v4_bottom = VEC3(v4[0], v4[1] - 5, v4[2]);

  Primitive *side1_1 = new Triangle(v1, v2, v2_bottom, VEC3(0.25, 0.25, 1));
  Primitive *side1_2 = new Triangle(v1, v2_bottom, v1_bottom, VEC3(0.25, 0.25, 1));

  Primitive *side2_1 = new Triangle(v2, v3, v3_bottom, VEC3(0.25, 0.25, 1));
  Primitive *side2_2 = new Triangle(v2, v3_bottom, v2_bottom, VEC3(0.25, 0.25, 1));

  Primitive *side3_1 = new Triangle(v3, v4, v4_bottom, VEC3(0.25, 0.25, 1));
  Primitive *side3_2 = new Triangle(v3, v4_bottom, v3_bottom, VEC3(0.25, 0.25, 1));

  Primitive *side4_1 = new Triangle(v4, v1, v1_bottom, VEC3(0.25, 0.25, 1));
  Primitive *side4_2 = new Triangle(v4, v1_bottom, v4_bottom, VEC3(0.25, 0.25, 1));

  Primitive *triangle1 = new Triangle(v1, v2, v3, VEC3(0.25, 0.25, 1));
  Primitive *triangle2 = new Triangle(v1, v3, v4, VEC3(0.25, 0.25, 1));

  prims.push_back(triangle1);
  prims.push_back(triangle2);
  prims.push_back(side1_1);
  prims.push_back(side1_2);
  prims.push_back(side2_1);
  prims.push_back(side2_2);
  prims.push_back(side3_1);
  prims.push_back(side3_2);
  prims.push_back(side4_1);
  prims.push_back(side4_2);
  

  VEC3 v11 = VEC3(4.75, -1, -5.75);
  VEC3 v21 = VEC3(6.75, -1, -5.75);
  VEC3 v31 = VEC3(6.75, -1, -3.75);
  VEC3 v41 = VEC3(4.75, -1, -3.75);

  VEC3 v1_bottom1 = VEC3(v11[0], v11[1] - 20, v11[2]);
  VEC3 v2_bottom1 = VEC3(v21[0], v21[1] - 20, v21[2]);
  VEC3 v3_bottom1 = VEC3(v31[0], v31[1] - 20, v31[2]);
  VEC3 v4_bottom1 = VEC3(v41[0], v41[1] - 20, v41[2]);

  Primitive *side1_11 = new Triangle(v11, v21, v2_bottom1, VEC3(1.0, 0.25, 0.25));
  Primitive *side1_21 = new Triangle(v11, v2_bottom1, v1_bottom1, VEC3(1.0, 0.25, 0.25));

  Primitive *side2_11 = new Triangle(v21, v31, v3_bottom1, VEC3(1.0, 0.25, 0.25));
  Primitive *side2_21 = new Triangle(v21, v3_bottom1, v2_bottom1, VEC3(1.0, 0.25, 0.25));

  Primitive *side3_11 = new Triangle(v31, v41, v4_bottom1, VEC3(1.0, 0.25, 0.25));
  Primitive *side3_21 = new Triangle(v31, v4_bottom1, v3_bottom1, VEC3(1.0, 0.25, 0.25));

  Primitive *side4_11 = new Triangle(v41, v11, v1_bottom1, VEC3(1.0, 0.25, 0.25));
  Primitive *side4_21 = new Triangle(v41, v1_bottom1, v4_bottom1, VEC3(1.0, 0.25, 0.25));

  Primitive *triangle11 = new Triangle(v11, v21, v31, VEC3(1.0, 0.25, 0.25));
  Primitive *triangle21 = new Triangle(v11, v31, v41, VEC3(1.0, 0.25, 0.25));
  
  prims.push_back(triangle11);
  prims.push_back(triangle21);
  prims.push_back(side1_11);
  prims.push_back(side1_21);
  prims.push_back(side2_11);
  prims.push_back(side2_21);
  prims.push_back(side3_11);
  prims.push_back(side3_21);
  prims.push_back(side4_11);
  prims.push_back(side4_21);

  VEC3 v111 = VEC3(4.75, -1, 5.75);
  VEC3 v211 = VEC3(6.75, -1, 5.75);
  VEC3 v311 = VEC3(6.75, -1, 3.75);
  VEC3 v411 = VEC3(4.75, -1, 3.75);

  VEC3 v1_bottom11 = VEC3(v111[0], v111[1] - 20, v111[2]);
  VEC3 v2_bottom11 = VEC3(v211[0], v211[1] - 20, v211[2]);
  VEC3 v3_bottom11 = VEC3(v311[0], v311[1] - 20, v311[2]);
  VEC3 v4_bottom11 = VEC3(v411[0], v411[1] - 20, v411[2]);

  Primitive *side1_111 = new Triangle(v111, v211, v2_bottom11, VEC3(1.0, 0.25, 0.25));
  Primitive *side1_211 = new Triangle(v111, v2_bottom11, v1_bottom11,  VEC3(1.0, 0.25, 0.25));

  Primitive *side2_111 = new Triangle(v211, v311, v3_bottom11,  VEC3(1.0, 0.25, 0.25));
  Primitive *side2_211 = new Triangle(v211, v3_bottom11, v2_bottom11,  VEC3(1.0, 0.25, 0.25));

  Primitive *side3_111 = new Triangle(v311, v411, v4_bottom11,  VEC3(1.0, 0.25, 0.25));
  Primitive *side3_211 = new Triangle(v311, v4_bottom11, v3_bottom11,  VEC3(1.0, 0.25, 0.25));

  Primitive *side4_111 = new Triangle(v411, v111, v1_bottom11,  VEC3(1.0, 0.25, 0.25));
  Primitive *side4_211 = new Triangle(v411, v1_bottom11, v4_bottom11,  VEC3(1.0, 0.25, 0.25));

  Primitive *triangle111 = new Triangle(v111, v211, v311,  VEC3(1.0, 0.25, 0.25));
  Primitive *triangle211 = new Triangle(v111, v311, v411,  VEC3(1.0, 0.25, 0.25));
  
  prims.push_back(triangle111);
  prims.push_back(triangle211);
  prims.push_back(side1_111);
  prims.push_back(side1_211);
  prims.push_back(side2_111);
  prims.push_back(side2_211);
  prims.push_back(side3_111);
  prims.push_back(side3_211);
  prims.push_back(side4_111);
  prims.push_back(side4_211);

  VEC3 v1111 = VEC3(-4.75, -1, -5.75);
  VEC3 v2111 = VEC3(-6.75, -1, -5.75);
  VEC3 v3111 = VEC3(-6.75, -1, -3.75);
  VEC3 v4111 = VEC3(-4.75, -1, -3.75);

  VEC3 v1_bottom111 = VEC3(v1111[0], v1111[1] - 20, v1111[2]);
  VEC3 v2_bottom111 = VEC3(v2111[0], v2111[1] - 20, v2111[2]);
  VEC3 v3_bottom111 = VEC3(v3111[0], v3111[1] - 20, v3111[2]);
  VEC3 v4_bottom111 = VEC3(v4111[0], v4111[1] - 20, v4111[2]);

  Primitive *side1_1111 = new Triangle(v1111, v2111, v2_bottom111, VEC3(1, 0.25 ,0.25));
  Primitive *side1_2111 = new Triangle(v1111, v2_bottom111, v1_bottom111,  VEC3(1, 0.25 ,0.25));

  Primitive *side2_1111 = new Triangle(v2111, v3111, v3_bottom111,  VEC3(1, 0.25 ,0.25));
  Primitive *side2_2111 = new Triangle(v2111, v3_bottom111, v2_bottom111,  VEC3(1, 0.25 ,0.25));

  Primitive *side3_1111 = new Triangle(v3111, v4111, v4_bottom111,  VEC3(1, 0.25 ,0.25));
  Primitive *side3_2111 = new Triangle(v3111, v4_bottom111, v3_bottom111,  VEC3(1, 0.25 ,0.25));

  Primitive *side4_1111 = new Triangle(v4111, v1111, v1_bottom111,  VEC3(1, 0.25 ,0.25));
  Primitive *side4_2111 = new Triangle(v4111, v1_bottom111, v4_bottom111,  VEC3(1, 0.25 ,0.25));

  Primitive *triangle1111 = new Triangle(v1111, v2111, v3111,  VEC3(1, 0.25 ,0.25));
  Primitive *triangle2111 = new Triangle(v1111, v3111, v4111,  VEC3(1, 0.25 ,0.25));
  
  prims.push_back(triangle1111);
  prims.push_back(triangle2111);
  prims.push_back(side1_1111);
  prims.push_back(side1_2111);
  prims.push_back(side2_1111);
  prims.push_back(side2_2111);
  prims.push_back(side3_1111);
  prims.push_back(side3_2111);
  prims.push_back(side4_1111);
  prims.push_back(side4_2111);

  VEC3 v11111 = VEC3(-4.75, -1, 5.75);
  VEC3 v21111 = VEC3(-6.75, -1, 5.75);
  VEC3 v31111 = VEC3(-6.75, -1, 3.75);
  VEC3 v41111 = VEC3(-4.75, -1, 3.75);

  VEC3 v1_bottom1111 = VEC3(v11111[0], v11111[1] - 20, v11111[2]);
  VEC3 v2_bottom1111 = VEC3(v21111[0], v21111[1] - 20, v21111[2]);
  VEC3 v3_bottom1111 = VEC3(v31111[0], v31111[1] - 20, v31111[2]);
  VEC3 v4_bottom1111 = VEC3(v41111[0], v41111[1] - 20, v41111[2]);


  Primitive *side1_11111 = new Triangle(v11111, v21111, v2_bottom1111, VEC3(1, 0.25 ,0.25));
  Primitive *side1_21111 = new Triangle(v11111, v2_bottom1111, v1_bottom1111,  VEC3(1, 0.25 ,0.25));

  Primitive *side2_11111 = new Triangle(v21111, v31111, v3_bottom1111,  VEC3(1, 0.25 ,0.25));
  Primitive *side2_21111 = new Triangle(v21111, v3_bottom1111, v2_bottom1111,  VEC3(1, 0.25 ,0.25));

  Primitive *side3_11111 = new Triangle(v31111, v41111, v4_bottom1111,  VEC3(1, 0.25 ,0.25));
  Primitive *side3_21111 = new Triangle(v31111, v4_bottom1111, v3_bottom1111,  VEC3(1, 0.25 ,0.25));

  Primitive *side4_11111 = new Triangle(v41111, v11111, v1_bottom1111,  VEC3(1, 0.25 ,0.25));
  Primitive *side4_21111 = new Triangle(v41111, v1_bottom1111, v4_bottom1111,  VEC3(1, 0.25 ,0.25));

  Primitive *triangle11111 = new Triangle(v11111, v21111, v31111,  VEC3(1, 0.25 ,0.25));
  Primitive *triangle21111 = new Triangle(v11111, v31111, v41111,  VEC3(1, 0.25 ,0.25));
  
  prims.push_back(triangle11111);
  prims.push_back(triangle21111);
  prims.push_back(side1_11111);
  prims.push_back(side1_21111);
  prims.push_back(side2_11111);
  prims.push_back(side2_21111);
  prims.push_back(side3_11111);
  prims.push_back(side3_21111);
  prims.push_back(side4_11111);
  prims.push_back(side4_21111);

  

  return;
}

//////////////////////////////////////////////////////////////////////////////////
// Build objects
//////////////////////////////////////////////////////////////////////////////////
void buildObstacle()
{

  // y: up down
  // z: left right
  // x: forward back
  float radius = 2.5; 
  float yStart = -10.0;
  float yEnd = 7.5;
  int numSpheres = 75;  
  float turns = 5.0;

  for (int i = 0; i < numSpheres; ++i)
  {
      float t = float(i) / numSpheres; 
      float angle = 2.0 * M_PI * turns * t; 
      float y = yStart + (yEnd - yStart) * t; 

      float x = radius * cos(angle);
      float z = radius * sin(angle);

      if (i % 2 == 0)
      {
        z += 0.1;
      }
      else
      {
        z -= 0.1;
      }
      Primitive *ball = new Sphere(VEC3(x, y, z), 0.15, VEC3(1, 1, 1));
      // if (i % 3 == 0)
      // {
      //   ball = new Sphere(VEC3(x, y, z), 0.15, VEC3(1, 1, 1), Material::GLOSSY);
      // }
      // else
      // {
      //   ball = new Sphere(VEC3(x, y, z), 0.15, VEC3(1, 1, 1), Material::NORMAL);
      // }
  
      prims.push_back(ball);
  }

  Primitive *ball = new Sphere(VEC3(0, 3, 0), 1, VEC3(1, 1, 1), Material::GLOSSY);
  prims.push_back(ball);
//   Primitive *sun = new Sphere(VEC3(0, 3.0, 3.5), 0.15, VEC3(1.0, 0.047, 0.0));
//   Primitive *sun1 = new Sphere(VEC3(0, 3.0, 0.5), 0.15, VEC3(1.0, 0.447, 0.0));
//   Primitive *sun2 = new Sphere(VEC3(0, 3.0, -2.5), 0.15, VEC3(1.0, 0.847, 0.0));

// VEC3 rectCenterBase = VEC3(5, 2, -5);

// float halfWidth = 0.75f;
// float halfHeight = 0.1f;
// VEC3 color1 = VEC3(1.0, 0.25, 0.25); 
// VEC3 color2 = VEC3(0.25, 1.0, 0.25);

// for (int i = 0; i < 5; ++i) {
//     float zOffset = i * 3.0f;
//     VEC3 rectCenter = rectCenterBase + VEC3(0, 0, zOffset); 

//     VEC3 normal = (eye - rectCenter).normalized();

//     VEC3 tangent = up.cross(normal).normalized();
//     VEC3 bitangent = normal.cross(tangent).normalized();

//     VEC3 v1 = rectCenter + tangent * halfWidth + bitangent * halfHeight; 
//     VEC3 v2 = rectCenter - tangent * halfWidth + bitangent * halfHeight;
//     VEC3 v3 = rectCenter - tangent * halfWidth - bitangent * halfHeight;
//     VEC3 v4 = rectCenter + tangent * halfWidth - bitangent * halfHeight; 

//     Primitive *triangle1 = new Triangle(v1, v2, v3, (i % 2 == 0) ? color1 : color2);
//     Primitive *triangle2 = new Triangle(v1, v3, v4, (i % 2 == 0) ? color1 : color2);

//     prims.push_back(triangle1);
//     prims.push_back(triangle2);
// }

//   Primitive *ball = new Sphere(VEC3(8, 0, 0.5), 1, VEC3(0.0, 0.0, 0.0));
  
//   // Primitive *streak = new Triangle(VEC3())

//   prims.push_back(sun);
//   prims.push_back(sun1);
//   prims.push_back(sun2);
//   prims.push_back(ball);
  // prims.push_back(p1);
  // prims.push_back(p2);
  // prims.push_back(p3);
  
  return;
}


//////////////////////////////////////////////////////////////////////////////////
// Build stick man
//////////////////////////////////////////////////////////////////////////////////
void buildStickMan(float camAngle, float dist, float height)
{
  // prims.clear();
  displayer.ComputeBonePositions(DisplaySkeleton::BONES_AND_LOCAL_FRAMES);

  // retrieve all the bones of the skeleton
  vector<MATRIX4>& rotations = displayer.rotations();
  vector<MATRIX4>& scalings  = displayer.scalings();
  vector<VEC4>& translations = displayer.translations();
  VEC4 &headTranslation = translations[16];
  // cout << "head: " << headTranslation.transpose() << endl;
  vector<float>& lengths     = displayer.lengths();

  // build a sphere list, but skip the first bone, 
  // it's just the origin
  int totalBones = rotations.size();
  for (int x = 1; x < totalBones; x++)
  {
    // cout << lengths[x] << endl;
    MATRIX4& rotation = rotations[x];
    MATRIX4& scaling = scalings[x];
    VEC4& translation = translations[x];

    // get the endpoints of the cylinder
    VEC4 leftVertex(0,0,0,1);
    VEC4 rightVertex(0,0,lengths[x],1);

    leftVertex = rotation * scaling * leftVertex + translation;
    rightVertex = rotation * scaling * rightVertex + translation;
    // if (x == 4)  // left foot
    // {
    //   cout << "left:" << leftVertex.transpose() << endl;
    //   cout << "right:" << rightVertex.transpose() << endl;
    // }

    Primitive *p = new Cylinder(leftVertex.head<3>(), rightVertex.head<3>(), 0.045, VEC3(1, 1, 1));
    // Primitive *leftCap = new Sphere(leftVertex.head<3>(), 0.045, VEC3(1, 1, 1));
    // Primitive *rightCap = new Sphere(rightVertex.head<3>(), 0.045, VEC3(1, 1, 1));
    prims.push_back(p);
    // prims.push_back(leftCap);
    // prims.push_back(rightCap);
  }

  VEC3 center = headTranslation.head<3>();
  eye = VEC3(
    center.x() + dist * cos(camAngle),
    center.y() + height,
    center.z() + dist * sin(camAngle)
  );
  lookingAt = center;
}

//////////////////////////////////////////////////////////////////////////////////
// Build scene
//////////////////////////////////////////////////////////////////////////////////

void buildScene(float camAngle, float dist, float height)
{
  setLights();
  buildStickMan(camAngle, dist, height);
  buildFloor();
  buildObstacle();
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  string skeletonFilename("140.asf");
  string motionFilename("140_03.amc");
  
  // load up skeleton stuff
  skeleton = new Skeleton(skeletonFilename.c_str(), MOCAP_SCALE);
  skeleton->setBasePosture();
  displayer.LoadSkeleton(skeleton);

  // load up the motion
  motion = new Motion(motionFilename.c_str(), MOCAP_SCALE, skeleton);
  displayer.LoadMotion(motion);
  skeleton->setPosture(*(displayer.GetSkeletonMotion(0)->GetPosture(0)));


  // omp_set_num_threads(4);
  // not set num threads:
  // set_num_threads(4): ./previz  1362.78s user 5.19s system 393% cpu 5:48.00 total
  // #pragma omp parallel for 
  // for (int x = 0; x < 900; x += 3)
  // {
  //   #pragma omp critical
  //   {
  //     float height = 1 + x * 0.005f;
  //     float dist = 2 + x * 0.01f;
  //     float camAngle = x * 0.005f;
  //     prims.clear();
  //     lights.clear();
  //     char buffer[256];
  //     snprintf(buffer, 256, "./frames/frame.%04i.ppm", x / 3);
  //     setSkeletonsToSpecifiedFrame(x);
  //     buildScene(camAngle, dist, height);
  //     renderImage(xRes, yRes, buffer);
  //   }
  //     cout << "Thread " << omp_get_thread_num() << " rendered frame " + to_string(x / 3) << endl;
  // }

  int x = 3 * 250;
  prims.clear();
  lights.clear();
  float height = 1 + x * 0.005f;
  float dist = 2 + x * 0.01f;
  float camAngle = x * 0.005f;
  char buffer[256];
  snprintf(buffer, 256, "frame.%04i.ppm", x / 3);
  setSkeletonsToSpecifiedFrame(x);
  buildScene(camAngle, dist, height);
  renderImage(xRes, yRes, buffer);
  cout << "Rendered frame 250" << endl;

  return 0;
}

// at run
// atq