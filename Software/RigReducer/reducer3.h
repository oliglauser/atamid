#ifndef REDUCER3_H
#define REDUCER3_H

#include <vector>
#include <iostream>
#include <math.h>
#include <chrono>
#include <igl/lbs_matrix.h>
#include <igl/writeDMAT.h>
#include <igl/writeTGF.h>
#include <igl/jet.h>
#include <gurobi_c++.h>
#include "draw_things.h"
#include "reduceModel.h"
#include <thread>

#include <unsupported/Eigen/MatrixFunctions>

using namespace std::chrono;

struct splitter_match{
  int library_id;
  int splitter_id;
  int node_id;
  double cost = 0;
  Eigen::MatrixXi branch_ass;
  double angle_error = 0;
  double rel_rot_error = 0;
  double theta;
  Eigen::MatrixXd opt_rot;
  int covered = 0;
  Eigen::VectorXi keep;
  Eigen::VectorXi out_id;
  double gain = 0;
};

struct Reduction
{
  Eigen::SparseMatrix<double> M;
  Eigen::MatrixXi E;
  std::vector<Eigen::MatrixXd> R;
  Eigen::VectorXi RN;
};

class Reducer3
{
  public:
    Reducer3(ReduceModel *model_in);
    bool ReductionStep();

    std::vector<Eigen::MatrixXd> PVs;
    std::vector<Eigen::MatrixXi> PFs;
    std::vector<Eigen::MatrixXd> PCs;

    Eigen::VectorXi rigid_nodes_n;

    void AddFlexibleNode(int id);
    void SetJointImportance(int id, double w);

    Eigen::SparseMatrix<double> K;
    Eigen::SparseMatrix<double> KV;

    std::vector<Eigen::MatrixXd> R;

    std::vector<std::vector<Eigen::MatrixXd > > RC;

    Eigen::VectorXd e;

    Eigen::SparseMatrix<double> M;

    Eigen::VectorXi PNR;

    Eigen::VectorXd n, v, v2;

    double last_reduction_error;

    Eigen::SparseMatrix<double> IM;

    Eigen::MatrixXd VNodes, VNodes2;

    Eigen::MatrixXd vstack, vstack2;

    Eigen::VectorXi rigid_nodes;

    Eigen::MatrixXi SEdges;

    std::vector<splitter_match> splitter_best;

    std::vector<Eigen::MatrixXd> R_best;

    int n_complex;

    std::vector<int> reduction_order;

    int joints_left;

    Eigen::MatrixXd RMP;

    double normalizef;

    Eigen::MatrixXd JI;

    // splitters
    std::vector<Eigen::MatrixXd> splitter_library_geometry;
    std::vector<int> splitter_library_n;

    void FindSplitters();
    double FindRotations(Reduction r, std::vector<Eigen::MatrixXd> &Ropt, double & cost);
    void ExportSetup();

    std::vector<Eigen::RowVector3d> CPoints;
    std::vector<Eigen::RowVector3d> CPointsC;

    // some splitter display
    std::vector<Eigen::MatrixXi> SLines;
    std::vector<Eigen::MatrixXd> SPoints;
    std::vector<Eigen::MatrixXd> SColors;

    std::vector<Eigen::MatrixXi> SLines_S;
    std::vector<Eigen::MatrixXd> SPoints_S;
    std::vector<Eigen::MatrixXd> SColors_S;

    int n_TUT;

    Reduction cr;

  private:
    ReduceModel *model;
    bool update_hessian(Eigen::MatrixXd &H, int p, Reduction r, Eigen::VectorXd& n, Eigen::VectorXd& vn, Eigen::VectorXd& hn);
    bool update_gradient(Eigen::VectorXd &g, int p, Reduction r, Eigen::VectorXd& n, Eigen::VectorXd& vn, Eigen::VectorXd& hn);
    void setup_variables(Eigen::VectorXi& rigid_nodes_new, Reduction &r);

    int n_poses;
    int n_simple;
    Eigen::MatrixXd colors;
    Eigen::MatrixXi EdgesCopy;
    Eigen::MatrixXd PosedNodes;
    Eigen::MatrixXd VPosedNodes;
    Eigen::MatrixXd VPosedNodes2;
    double rl;

    Eigen::VectorXi rigid_nodes_f;
    std::vector<Eigen::MatrixXd> J;
};

#endif
