#ifndef MERGE_MESHES_H
#define MERGE_MESHES_H

#include <igl/repmat.h>

void merge_meshes(Eigen::MatrixXd& V, Eigen::MatrixXi& F, Eigen::MatrixXd& C, std::vector<Eigen::MatrixXd> &Vs, std::vector<Eigen::MatrixXi> &Fs, std::vector<Eigen::MatrixXd> &Cs);
void merge_edges(Eigen::MatrixXd& P, Eigen::MatrixXi& E, Eigen::MatrixXd& C, std::vector<Eigen::MatrixXd> &Ps, std::vector<Eigen::MatrixXi> &Es, std::vector<Eigen::MatrixXd> &Cs);

#endif
