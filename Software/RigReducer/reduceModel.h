#ifndef REDUCEMODEL_H
#define REDUCEMODEL_H

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <igl/readPLY.h>
#include <igl/readOFF.h>
#include <igl/readOBJ.h>
#include <igl/readTGF.h>
#include <igl/readDMAT.h>
#include <igl/bone_parents.h>
#include <igl/file_exists.h>
#include <igl/lbs_matrix.h>
#include <igl/REDRUM.h>
#include <igl/repmat.h>
#include <igl/vertex_triangle_adjacency.h>
#include <igl/doublearea.h>
#include <iomanip>
#include <iostream>

class ReduceModel
{
public:
  Eigen::MatrixXd Nodes;
  Eigen::MatrixXi Edges;

  std::vector<Eigen::MatrixXd> Poses;
  std::vector<Eigen::MatrixXd> PosesN;

  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  Eigen::MatrixXd C;
  Eigen::MatrixXd M;

  double scale_factor;

  Eigen::MatrixXi P;

  Eigen::MatrixXi PN;

  Eigen::MatrixXd W;

  Eigen::VectorXd I;

  int bone_root_id;

  Eigen::MatrixXd WC;
  Eigen::MatrixXd MC;

  bool maya;

  bool normalise;

  //Eigen::MatrixXd move_root_to_zero_matrix;

  int root_node_id;

  bool load_poses();
  bool load_model(std::string path);
  static void vertex_area(Eigen::VectorXd& AV, Eigen::MatrixXd& V, Eigen::MatrixXi& F);

  std::string basepath;
};

#endif
