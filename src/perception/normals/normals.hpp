#include <KDTreeVectorOfVectorsAdaptor.h>
#include <iostream>
#include <eigen3/Eigen/Dense>

std::vector<double> estimate_nomal(std::vector<std::vector<double>> points);
std::vector<std::vector<double>> compute_normals(std::vector<std::vector<double>> point_cloud, int max_neighbors, double max_distance);
