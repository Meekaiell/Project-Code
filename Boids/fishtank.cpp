#include <iostream>
#include <fstream>
#include <cstring>
#include <vector>
#include <Eigen/Dense>
#include <cassert>
#include "kdTree.H"

#ifndef M_PI
#define M_PI        3.13159265358979323846  // pi //
#endif

struct Props {
  double size, neighbor_radius, mass, collision, centering, velocity, hunger, damping, dt, length;
  int num_neighbors;
};

class Fish{ 
public:
  Eigen::Vector3d pos, vel, frc;
  void move(const Props &props);
};

class Food {
public:
  Eigen::Vector3d pos, vel;
  double t;
  bool eaten;
  void move(double dt);
};

void Fish::move(const Props &props){
  vel += props.dt * frc/props.mass;
  vel *= props.damping;
  pos += props.dt * vel;
  if(pos[0] < -0.5){
    pos[0] = -0.5;
    vel[0] = fabs(vel[0]);
  }
  if(pos[1] < -0.25){
    pos[1] = -0.25;
    vel[1] = fabs(vel[1]);
  }
  if(pos[2] < -0.125){
    pos[2] = -0.5;
    vel[2] = fabs(vel[2]);
  }
  if(pos[0] > 0.5){
    pos[0] = 0.5;
    vel[0] = -fabs(vel[0]);
  }
  if(pos[1] > 0.25){
    pos[1] = 0.25;
    vel[1] = -fabs(vel[1]);
  }
  if(pos[2] > 0.125){
    pos[2] = 0.125;
    vel[2] = -fabs(vel[2]);
  }

}

void Food::move(double dt){
  vel[0] += 0.0001 * (((double)rand())/RAND_MAX - 0.5);
  vel[2] += 0.0001 * (((double)rand())/RAND_MAX - 0.5);
  pos += dt * vel;
   if(pos[0] < -0.5){
    pos[0] = -0.5;
    vel[0] *= -1;
  }
  if(pos[1] < -0.25){
    pos[1] = -0.25;
    vel[1] *= 0.0;
  }
  if(pos[2] < -0.125){
    pos[2] = -0.5;
    vel[2] *= -1;
  }
  if(pos[0] > 0.5){
    pos[0] = 0.5;
    vel[0] *= -1;
  }
  if(pos[1] > 0.25){
    pos[1] = 0.25;
    vel[1] *= -1;
  }
  if(pos[2] > 0.125){
    pos[2] = 0.125;
    vel[2] *= -1;
  }
}

inline double sqr(double x) {return x*x;}
inline double cube(double x) {return x*x*x;}

void timestep(const Props &props, std::vector<Fish> &fish, std::vector<Food> &food){
  std::vector<Eigen::Vector3d> pts;
  pts.resize(fish.size());
  for (unsigned int z = 0; z < fish.size(); z++){
    pts[z] = fish[z].pos;
  }

  KDTree tree(pts);
  for(unsigned int i = 0; i < fish.size(); i++){
    std::vector<int> neighbors;
    Fish &f = fish[i];
    tree.neighbors(pts, f.pos, props.num_neighbors, props.neighbor_radius, neighbors);
    f.frc << 0.0, 0.0, 0.0;
    Eigen::Vector3d avgpos, avgvel;
    avgpos << 0.0, 0.0, 0.0;
    avgvel << 0.0, 0.0, 0.0;
    int c = 0;
    for (unsigned int j = 0; j < neighbors.size(); j++){
      c++;
      int n = neighbors[j];
      if (n == i) continue;
      Eigen::Vector3d frc = f.pos - fish[n].pos;
      double m = frc.norm();
      if (m > 0.0001)
        f.frc += props.collision * frc / cube(m);
      avgpos += fish[n].pos;
      avgvel += fish[n].vel;
    }
    assert(c == neighbors.size());
    avgpos /= neighbors.size();
    avgvel /= neighbors.size();
    Eigen::Vector3d center = avgpos - f.pos;
    double m = center.norm();
    if(m > 0.0001)
      f.frc += props.centering * (avgpos - f.pos);
    f.frc += props.velocity * (avgpos - f.pos);
  }
  std::vector<Eigen::Vector3d> foodpts;
  foodpts.resize(food.size());
  for (unsigned int i = 0; i < food.size(); i++) {
    foodpts[i] = food[i].pos; 
  }
  KDTree foodtree(foodpts);
  bool eaten = false;
  for(unsigned int i = 0; i < fish.size(); i++){
    std::vector<int> neighbors;
    Fish &f = fish[i];
    int n = foodtree.neighbor(foodpts,f.pos,props.neighbor_radius);
    if (n >= 0 && !food[n].eaten){
      Eigen::Vector3d frc = food[n].pos - f.pos;
      double d = frc.norm();
      frc /= d;
      if (d < 2 * props.size){
        food[n].eaten = true;
        eaten = true;
      }
      f.frc += props.hunger * frc;
    }
  }
  if (eaten) {
    std::vector<Food> newFood;
    for(unsigned int i = 0; i < food.size(); i++){
      if(!food[i].eaten)
        newFood.push_back(food[i]);
    }
    food = newFood;
  }
  for (unsigned int i = 0; i < fish.size(); i++){
    fish[i].move(props);
  }
  for (unsigned int i = 0; i < food.size(); i++){
    food[i].move(props.dt);
  }
}

void parseInput(char *fname, Props &props, std::vector<Fish> &fish, std::vector<Food> &food) {
  int nfish, nfood;
  std::ifstream in (fname, std::ios::in);
  char junk;

  in >> props.size >> props.neighbor_radius >> props.num_neighbors
    >> props.mass >> props.collision >> props.centering
    >> props.velocity >> props.hunger >> props.damping
    >> props.dt >> props.length;
  in >> nfish;
  for (unsigned int i=0; i < nfish; i++){
    Fish f;
    in >> junk >> f.pos[0] >> junk >> f.pos[1] >> junk >> f.pos[2] >> junk;
    in >> junk >> f.vel[0] >> junk >> f.vel[1] >> junk >> f.vel[2] >> junk;
    fish.push_back(f);
  }
  in >> nfood;
  for (unsigned int i=0; i < nfood; i++){
    Food f;
    in >> junk >> f.pos[0] >> junk >> f.pos[1] >> junk >> f.pos[2] >> junk;
    in >> junk >> f.vel[0] >> junk >> f.vel[1] >> junk >> f.vel[2] >> junk;
    in >> f.t;
    food.push_back(f);
  }
}

void writeOutput (std::ofstream &out, const std::vector<Fish> &fish, const std::vector<Food> &food){
  out << fish.size()<< std::endl;
  for (unsigned int i = 0; i < fish.size(); i++){
    out << "[" << fish[i].pos[0] << "," << fish[i].pos[1] << "," << fish[i].pos[2] << "]";
    out << "[" << fish[i].vel[0] << "," << fish[i].vel[1] << "," << fish[i].vel[2] << "]" <<std::endl;
  }
  out << food.size() << std::endl;
  for (unsigned int i = 0; i < food.size(); i++){
    out << "[" << food[i].pos[0] << "," << food[i].pos[1] << "," << food[i].pos[2] << "]" <<std::endl; 
  }
}

int main(int argc, char *argv[]){
  std::vector<Fish> fish;
  std::vector<Food> allFood;
  std::vector<Food> food;
  Props props;
  double time = 0.0;
  double time_till_next_frame = 0.0;
  
  parseInput(argv[1], props, fish, allFood);

  std::ofstream out(argv[2], std::ios::out);
  out << (props.length / props.dt) << std::endl;

  while (time < props.length) {
    std::cout << time << std::endl;
    if(time_till_next_frame < 0.0){
      writeOutput(out, fish, food);
      time_till_next_frame = 1.0/30.0;
    }
    for (unsigned int i = 0; i < allFood.size(); i++){
      if(time > allFood[i].t && time - props.dt < allFood[i].t){
        food.push_back(allFood[i]);
      }
    }
    timestep(props, fish, food);
    time += props.dt;
    time_till_next_frame -= props.dt;
  }

}