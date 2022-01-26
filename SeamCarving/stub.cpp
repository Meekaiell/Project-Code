#define cimg_display 0
#include "CImg.h"
#include <Eigen/Dense>
using namespace cimg_library;

int main(int argc, char *argv[]) {
  CImg<double> input(argv[1]);
  CImg<double> lab = input.RGBtoLab();
  Eigen::Vector3d *image = new Eigen::Vector3d[input.width()*input.height()];
  for (unsigned int i=0; i<input.width(); i++) {
	for (unsigned int j=0; j<input.height(); j++) {
	  image[i*input.height()+j][0] = lab(i, j, 0);
	  image[i*input.height()+j][1] = lab(i, j, 1);
	  image[i*input.height()+j][2] = lab(i, j, 2);
	}
  }
  
  CImg<double> output(atoi(argv[3]), atoi(argv[4]), input.depth(), input.spectrum(), 0);
  for (unsigned int i=0; i<output.width(); i++) {
	for (unsigned int j=0; j<output.height(); j++) {
	  output(i, j, 0) = image[i*output.height()+j][0];
	  output(i, j, 1) = image[i*output.height()+j][1];
	  output(i, j, 2) = image[i*output.height()+j][2];
	}
  }
  CImg<double> rgb = output.LabtoRGB();
  if (strstr(argv[2], "png"))
	rgb.save_png(argv[2]);
  else if (strstr(argv[2], "jpg"))
	rgb.save_jpeg(argv[2]);

  delete [] image;
  return 0;
}
