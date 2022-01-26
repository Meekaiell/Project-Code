#ifndef IO_H
#define IO_H
#include <vector>
#include <Eigen/Dense>

class Tri {
public:
  int indices[3];
  inline int &operator[](const unsigned int &i) { return indices[i];};
  inline int operator[](const unsigned int &i) const { return indices[i];};
  inline void set(int x, int y, int z) {indices[0] = x; indices[1] = y; indices[2] = z;};
  inline Tri(int x, int y, int z) {indices[0] = x; indices[1] = y; indices[2] = z;};
  inline Tri() {};
  inline Tri &operator=(const Tri &that);
};

inline Tri &Tri::operator=(const Tri &that) {
  this->indices[0] = that.indices[0];
  this->indices[1] = that.indices[1];
  this->indices[2] = that.indices[2];
  return (*this);
}

bool readObjFile(char *fname, std::vector<Eigen::Vector3d> &pts, std::vector<Tri> &triangles);

void writeObjFile(char *fname, const std::vector<Eigen::Vector3d> &meshPts, const std::vector<Tri> &triangles);

#endif
