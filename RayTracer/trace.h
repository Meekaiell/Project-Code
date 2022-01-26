#ifndef TRACE_H
#define TRACE_H


//#include "slVector.H"
#include <vector>
#include <Eigen/Dense>

class Ray {
public:
  Eigen::Vector3d e;
  Eigen::Vector3d d;
  int depth;
  Ray(const Eigen::Vector3d &_e, const Eigen::Vector3d &_d, int _depth = 0) : e(_e), d(_d), depth(_depth) {};
};

class Fill {
public: 
  Eigen::Vector3d color;
  double kd, ks, shine, transmittance, ior;
};

class HitRecord {
public:
  double t, alpha, beta, gamma;
  Eigen::Vector3d p, n, v;
  Fill f;
  int raydepth;
};

class Light {
public:
  Eigen::Vector3d p, c;
};

class Surface {
public:
  virtual bool intersect(const Ray &r, double t0, double t1, HitRecord &hr) const = 0;
  //  virtual Eigen::Vector3d normal(const Eigen::Vector3d &x) const = 0;
  virtual ~Surface() {};
};

class Triangle : public Surface {
protected:
  Eigen::Vector3d a,b,c;
public:
  Triangle(const Eigen::Vector3d &_a, const Eigen::Vector3d &_b, const Eigen::Vector3d &_c) : a(_a), b(_b), c(_c) {};
  virtual bool intersect(const Ray &r, double t0, double t1, HitRecord &hr) const;
  //  virtual Eigen::Vector3d normal(const Eigen::Vector3d &x) const;
};

class TrianglePatch : public Triangle {
  Eigen::Vector3d n1, n2, n3;
public:
  TrianglePatch(const Eigen::Vector3d &_a, const Eigen::Vector3d &_b, const Eigen::Vector3d &_c,
	  const Eigen::Vector3d &_n1, const Eigen::Vector3d &_n2, const Eigen::Vector3d &_n3) 
	: Triangle(_a,_b,_c), n1(_n1), n2(_n2), n3(_n3) {};
  //  virtual Eigen::Vector3d normal(const Eigen::Vector3d &x) const;
};

class Poly : public Surface {
  std::vector<Eigen::Vector3d> verts;
public:
  Poly(const std::vector<Eigen::Vector3d> &_verts) : verts(_verts) {}; 
  virtual bool intersect(const Ray &r, double t0, double t1, HitRecord &hr) const;
  //  virtual Eigen::Vector3d normal(const Eigen::Vector3d &x) const;
};

class PolyPatch : public Poly {
  std::vector<Eigen::Vector3d> normals;
public:
  PolyPatch(const std::vector<Eigen::Vector3d> &_verts, const std::vector<Eigen::Vector3d> &_normals) : Poly(_verts), normals(_normals) {}; 
};

class Sphere : public Surface {
  Eigen::Vector3d c;
  double rad;
public:
  Sphere(const Eigen::Vector3d &_c, double _r) : c(_c), rad(_r) {};
  virtual bool intersect(const Ray &r, double t0, double t1, HitRecord &hr) const;
  //  virtual Eigen::Vector3d normal(const Eigen::Vector3d &x) const;
};

class Tracer {
  Eigen::Vector3d bcolor, eye, at, up;
  double angle, hither;
  unsigned int res[2];
  std::vector<std::pair<Surface *, Fill> > surfaces;
  std::vector<Light> lights;
  double shadowbias;
  
  Eigen::Vector3d *im;
public:
  Tracer(const std::string &fname);
  ~Tracer();
  void createImage();  
  Eigen::Vector3d castRay(const Ray &ray, double t0, double t1) const;
  Eigen::Vector3d shade(const HitRecord &hr) const;
  void writeImage(const std::string &fname);

  bool color;
  bool phong;
  bool stratified;
  bool reflections;
  bool shadows;
  bool verbose;
  int samples;
  double aperture;
  int maxraydepth;
};
	
#endif
