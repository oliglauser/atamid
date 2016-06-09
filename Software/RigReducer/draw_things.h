#ifndef DRAW_THINGS_H
#define DRAW_THINGS_H

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include "MeshMerger.h"

#include <igl/readSTL.h>
#include <igl/readPLY.h>
#include <igl/readOBJ.h>

void load_S(std::vector<Eigen::MatrixXd> &Vs, std::vector<Eigen::MatrixXi> &Fs, std::vector<Eigen::MatrixXd> &Cs, std::string path, Eigen::MatrixXd &C);
void draw_S(Eigen::MatrixXd& V, Eigen::MatrixXi& F, Eigen::MatrixXd& C, std::vector<Eigen::MatrixXd> &Vo, std::vector<Eigen::MatrixXi> &Fo, std::vector<Eigen::MatrixXd> &Co, Eigen::MatrixXd& Outs);

void load_TUT(std::vector<Eigen::MatrixXd> &Vs, std::vector<Eigen::MatrixXi> &Fs, std::vector<Eigen::MatrixXd> &Cs, std::string path, Eigen::MatrixXd &C);
void draw_TUT(Eigen::MatrixXd& V, Eigen::MatrixXi& F, Eigen::MatrixXd& C, std::vector<Eigen::MatrixXd> &Vo, std::vector<Eigen::MatrixXi> &Fo, std::vector<Eigen::MatrixXd> &Co, float angle1, float angle2, float angle3, float angle4);

void draw_bone(Eigen::MatrixXd& PO, Eigen::MatrixXi& EO, Eigen::MatrixXd& CO, Eigen::MatrixXd& O, Eigen::MatrixXd& RPY, Eigen::MatrixXd& CC, double length, float size);
void draw_skeleton(Eigen::MatrixXd& PO, Eigen::MatrixXi& EO, Eigen::MatrixXd& CO, Eigen::MatrixXd& P, Eigen::MatrixXi& BE, Eigen::MatrixXd& C, float size);

void draw_puppet(std::vector<Eigen::MatrixXd> &Vs, std::vector<Eigen::MatrixXi> &Fs, std::vector<Eigen::MatrixXd> &Cs, Eigen::MatrixXi &Edges, int rootID, Eigen::VectorXi &SplitterMap, std::vector<Eigen::MatrixXd> &Splitters, Eigen::MatrixXd &Cos, Eigen::MatrixXd &Restposes, Eigen::MatrixXd &JC);


#endif
