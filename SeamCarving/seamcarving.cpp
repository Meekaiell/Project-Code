#define cimg_display 0
#include "CImg.h"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <limits>
using namespace cimg_library;


//compute energy function
void energy(Eigen::Vector3d* img, float* energy, int width, int height) {

	double deltaX, deltaY, eMax = 0;
	//the main loop of the image calculate the energy image pixel-by-pixel 
	for (unsigned int column = 0; column < height; column++) {
		for (unsigned int row = 0; row < width; row++) {
			if (row == 0 && column == 0) {
				deltaX = (img[(column * width + row) + 1] - img[(column * width + row)]).norm();
				deltaY = (img[(column * width + row) + width] - img[(column * width + row)]).norm();
				energy[column * width + row] = deltaX + deltaY;
				if (energy[column * width + row] > eMax)
					eMax = energy[column * width + row];
			}
			else if ((row > 0 && row < width - 1) && column == 0) {
				deltaX = (img[(column * width + row) + 1] - img[(column * width + row) - 1]).norm();
				deltaY = (img[(column * width + row) + width] - img[(column * width + row)]).norm();
				energy[column * width + row] = deltaX + deltaY;
				if (energy[column * width + row] > eMax)
					eMax = energy[column * width + row];
			}
			else if ((row == width - 1 && column == 0)) {
				deltaX = (img[(column * width + row)] - img[(column * width + row) - 1]).norm();
				deltaY = (img[(column * width + row) + width] - img[(column * width + row)]).norm();
				energy[column * width + row] = deltaX + deltaY;
				if (energy[column * width + row] > eMax)
					eMax = energy[column * width + row];
			}
			else if (row == 0 && (column != height - 1 && column != 0)) {
				deltaX = (img[(column * width + row) + 1] - img[(column * width + row)]).norm();
				deltaY = (img[(column * width + row) + width] - img[(column * width + row) - width]).norm();
				energy[column * width + row] = (deltaX + deltaY);
				if (energy[column * width + row] > eMax)
					eMax = energy[column * width + row];
			}
			else if (row == width - 1 && (column != height - 1 && column != 0)) {
				deltaX = (img[(column * width + row)] - img[(column * width + row) - 1]).norm();
				deltaY = (img[(column * width + row) + width] - img[(column * width + row) - width]).norm();
				energy[column * width + row] = deltaX + deltaY;
				if (energy[column * width + row] > eMax)
					eMax = energy[column * width + row];
			}
			else if (row == 0 && column == height - 1) {
				deltaX = (img[(column * width + row) + 1] - img[(column * width + row)]).norm();
				deltaY = (img[(column * width + row) - width] - img[(column * width + row)]).norm();
				energy[column * width + row] = deltaX + deltaY;
				if (energy[column * width + row] > eMax)
					eMax = energy[column * width + row];
			}
			else if ((row > 0 && row < width - 1) && column == height - 1) {
				deltaX = (img[(column * width + row) + 1] - img[(column * width + row) - 1]).norm();
				deltaY = (img[(column * width + row)] - img[(column * width + row) - width]).norm();
				energy[column * width + row] = deltaX + deltaY;
				if (energy[column * width + row] > eMax)
					eMax = energy[column * width + row];
			}
			else if (row == width - 1 && column == height - 1) {
				deltaX = (img[(column * width + row)] - img[(column * width + row) - 1]).norm();
				deltaY = (img[(column * width + row)] - img[(column * width + row) - width]).norm();
				energy[column * width + row] = deltaX + deltaY;
				if (energy[column * width + row] > eMax)
					eMax = energy[column * width + row];
			}
			else {
				deltaX = (img[(column * width + row) + 1] - img[(column * width + row) - 1]).norm();
				deltaY = (img[(column * width + row) + width] - img[(column * width + row) - width]).norm();
				energy[column * width + row] = deltaX + deltaY;
				if (energy[column * width + row] > eMax)
					eMax = energy[column * width + row];
			}
		}
	}
	//Conversion to greyscale
	for (unsigned int column = 0; column < height; column++) {
		for (unsigned int row = 0; row < width; row++) {
			img[column * width + row][0] = pow(energy[column * width + row] / eMax, 1.0 / 3.0) * 255;
			img[column * width + row][1] = pow(energy[column * width + row] / eMax, 1.0 / 3.0) * 255;
			img[column * width + row][2] = pow(energy[column * width + row] / eMax, 1.0 / 3.0) * 255;

		}
	}

}

//fuction to perform the removal of a vertical seam
void ripSeam(Eigen::Vector3d* img, float* e, int width, int height) {
	float* cost = new float[width * height];
	int* vertpath = new int[height];
	float vertleft, vertright, vertup, min_cost;

	for (int i = 0; i < width * height; i++)
		cost[i] = 0;

	for (unsigned int j = 0; j < height; j++) {
		for (unsigned int i = 0; i < width; i++) {
			//Top Row
			if (j == 0)
				cost[i] = e[i];
			//Left Column
			else if (i == 0 && j != 0 && j != height - 1) {
				vertup = cost[(j * width + i) - width];
				vertright = cost[(j * width + i) - width + 1];
				cost[j * width + i] = e[j * width + i] + std::min(vertup, vertright);
			}
			//Middle Area
			else if (i != width - 1 && i != 0 && j != height - 1 && j != 0)
			{
				vertleft = cost[(j * width + i) - width - 1];
				vertup = cost[(j * width + i) - width];
				vertright = cost[(j * width + i) - width + 1];
				cost[j * width + i] = e[j * width + i] + std::min({ vertleft, vertup, vertright });
			}
			//Right Column
			else if (i == width - 1 && j != 0 && j != height - 1) {
				vertleft = cost[(j * width + i) - width - 1];
				vertup = cost[(j * width + i) - width];
				cost[j * width + i] = e[j * width + i] + std::min(vertleft, vertup);
			}
			//Bottom Left
			else if (j == height - 1 && i == 0) {
				cost[j * width + i] = e[j * width + i] + std::min(cost[(j * width + i) - width], cost[(j * width + i) - width + 1]);
			}
			//Bottom Right
			else if (j == height - 1 && i == width - 1) {
				cost[j * width + i] = e[j * width + i] + std::min(cost[(j * width + i) - width], cost[(j * width + i) - width - 1]);
			}
			//Bottom Row
			else if (j == height - 1)
				cost[j * width + i] = e[j * width + i] + std::min({ cost[(j * width + i) - width], cost[(j * width + i) - width - 1], cost[(j * width + i) - width + 1] });
		}
	}


	for (unsigned int i = 0; i < width; i++) {
		unsigned int j = height - 1;
		if (i == 0) {
			min_cost = cost[j * width + i];
			vertpath[0] = j * width + i;
		}
		if (cost[j * width + i] < min_cost) {
			min_cost = cost[j * width + i];
			vertpath[0] = j * width + i;
		}
	}


	//Seam generation
	for (unsigned int i = 0; i < height - 1; i++) {

		//Bottom Right
		if ((vertpath[i] + 1) % width == 0) {
			if (cost[vertpath[i] - width - 1] < cost[vertpath[i] - width])
				vertpath[i + 1] = vertpath[i] - width - 1;
			else
				vertpath[i + 1] = vertpath[i] - width;
		}

		//Bottom Left
		else if ((vertpath[i]) % width == 0) {
			if (cost[vertpath[i] - width + 1] < cost[vertpath[i] - width])
				vertpath[i + 1] = vertpath[i] - width + 1;
			else
				vertpath[i + 1] = vertpath[i] - width;
		}
		//Middle Area
		else {
			if ((cost[vertpath[i] - width + 1] < cost[vertpath[i] - width]) && cost[vertpath[i] - width + 1] < cost[vertpath[i] - width - 1])
				vertpath[i + 1] = vertpath[i] - width + 1;
			else if ((cost[vertpath[i] - width] < cost[vertpath[i] - width + 1]) && cost[vertpath[i] - width] < cost[vertpath[i] - width - 1])
				vertpath[i + 1] = vertpath[i] - width;
			else
				vertpath[i + 1] = vertpath[i] - width - 1;

		}
	}

	//Temp img to store changes
	Eigen::Vector3d* tmpimage = new Eigen::Vector3d[(width - 1) * (height)];

	//Store Current Image
	int k = 0;
	bool check = true;
	for (unsigned int j = 0; j < height; j++) {
		for (unsigned int i = 0; i < width; i++) {
			for (unsigned int l = 0; l < height; l++) {
				if (img[vertpath[l]] != img[j * width + i])
					check = true;
				else
					check = false;
				if (check == false)
					break;
			}

			if (check == true) {
				tmpimage[k] = img[j * width + i];
				k++;
			}
		}
	}
	img = tmpimage;
}




int main(int argc, char* argv[]) {
	CImg<double> input(argv[1]);
	CImg<double> lab = input;
	int h = input.height();
	int w = input.width();
	Eigen::Vector3d* image = new Eigen::Vector3d[input.width() * input.height()];


	for (unsigned int j = 0; j < input.height(); j++) {
		for (unsigned int i = 0; i < input.width(); i++) {
			image[j * input.width() + i][0] = lab(i, j, 0);
			image[j * input.width() + i][1] = lab(i, j, 1);
			image[j * input.width() + i][2] = lab(i, j, 2);
		}
	}

	float* e = new float[w * h];
	energy(image, e, w, h);


	ripSeam(image, e, h, w);


	CImg<double> output(atoi(argv[3]), atoi(argv[4]), input.depth(), input.spectrum(), 0);
	for (unsigned int j = 0; j < output.height(); j++) {
		for (unsigned int i = 0; i < output.width(); i++) {
			output(i, j, 0) = image[j * output.width() + i][0];
			output(i, j, 1) = image[j * output.width() + i][1];
			output(i, j, 2) = image[j * output.width() + i][2];
		}
	}

	if (strstr(argv[2], "png"))
		output.save_png(argv[2]);
	else if (strstr(argv[2], "jpg"))
		output.save_jpeg(argv[2]);

	delete[] image;
	delete[] e;
	return 0;
}
