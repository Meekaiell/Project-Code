#include "io.h"
#include <fstream>
#include <iostream>
#include <Eigen/Dense>

int main (int argc, char *argv[]) {
  std::vector<Tri> tris;
  std::vector<Eigen::Vector3d> pts;
  std::vector<double> m(pts.size());
  std::vector<Eigen::Vector3d> lap(pts.size());

  readObjFile(argv[1], pts, tris);

  for (int i = 0; i < atoi(argv[3]); i++) {
      for(unsigned int j = 0; j < pts.size(); j++) {
	m[tris[i][j]]   = 0.0;
	lap[tris[i][j]] = {0.0, 0.0, 0.0};
      }
      
      for (unsigned int t = 0; t < pts.size(); t++) {
	lap[tris[t][0]] += pts[tris[t][1]] - pts[tris[t][0]];
	lap[tris[t][0]] += pts[tris[t][2]] - pts[tris[t][0]];

	lap[tris[t][1]] += pts[tris[t][0]] - pts[tris[t][1]];
	lap[tris[t][1]] += pts[tris[t][2]] - pts[tris[t][1]];

	lap[tris[t][2]] += pts[tris[t][0]] - pts[tris[t][2]];
	lap[tris[t][2]] += pts[tris[t][1]] - pts[tris[t][2]];

	m[tris[t][0]] += 2;
	m[tris[t][1]] += 2;
	m[tris[t][2]] += 2;
      }
      
      for (unsigned int j = 0; j < pts.size(); j++) {
	pts[j] = pts[j] + atof(argv[4]) * (lap[j] / m[j]);
      }
  }
  writeObjFile(argv[2], pts, tris);
}
