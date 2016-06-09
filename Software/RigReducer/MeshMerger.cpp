#include "MeshMerger.h"

void merge_meshes(Eigen::MatrixXd& V, Eigen::MatrixXi& F, Eigen::MatrixXd& C, std::vector<Eigen::MatrixXd> &Vs, std::vector<Eigen::MatrixXi> &Fs, std::vector<Eigen::MatrixXd> &Cs)
{
    assert(Vs.size() == Fs.size());

    int V_rows = 0;
    int F_rows = 0;

    for(int i=0; i < Vs.size(); i++)
    {
      V_rows+=Vs.at(i).rows();
      F_rows+=Fs.at(i).rows();
    }

    F.resize(F_rows,3);
    V.resize(V_rows,3);
    C.resize(V_rows,3);

    int NCOL = Fs.at(0).cols();

    int V_index = 0;
    int F_index = 0;

    Eigen::MatrixXd COLORS;

    for(int i=0; i < Vs.size(); i++)
    {
      //igl::repmat(Cs[i], Fs[i].rows(), 1, COLORS);

      if(Cs[i].rows()==1)
      {
        igl::repmat(Cs[i], Vs.at(i).rows(), 1, COLORS);
      }
      else
      {
        COLORS = Cs[i];
      }

      C.block(V_index, 0, Vs.at(i).rows(), 3) = COLORS;

      V.block(V_index, 0, Vs.at(i).rows(), 3) = Vs.at(i);
      F.block(F_index, 0, Fs.at(i).rows(), NCOL) = Fs.at(i) + V_index*Eigen::MatrixXi::Ones(Fs.at(i).rows(), Fs.at(i).cols());

      V_index += Vs.at(i).rows();
      F_index += Fs.at(i).rows();
    }
}

void merge_edges(Eigen::MatrixXd& P, Eigen::MatrixXi& E, Eigen::MatrixXd& C, std::vector<Eigen::MatrixXd> &Ps, std::vector<Eigen::MatrixXi> &Es, std::vector<Eigen::MatrixXd> &Cs)
{
  //assert(Ps.size() == Es.size());

  int P_rows = 0;
  int E_rows = 0;

  for(int i=0; i < Ps.size(); i++)
  {
    P_rows+=Ps.at(i).rows();
    E_rows+=Es.at(i).rows();
  }

  int NCOL = Es.at(0).cols();

  E.resize(E_rows,NCOL);
  P.resize(P_rows,3);
  C.resize(E_rows,3);

  int P_index = 0;
  int E_index = 0;

  Eigen::MatrixXd COLORS;

  for(int i=0; i < Ps.size(); i++)
  {
    //std::cout << "1" << std::endl;

    if(Cs[i].rows()==1)
    {
      igl::repmat(Cs[i], Es.at(i).rows(), 1, COLORS);
    }
    else
    {
      COLORS = Cs[i];
    }

    /*std::cout << "2" << std::endl;

    std::cout << Cs[i] << std::endl;
    std::cout << COLORS.rows() << std::endl;
    std::cout << Es.at(i).rows() << std::endl;*/

    C.block(E_index, 0, Es.at(i).rows(), 3) = COLORS;

    //std::cout << "3" << std::endl;
    P.block(P_index, 0, Ps.at(i).rows(), 3) = Ps.at(i);

    //std::cout << "4" << std::endl;
    E.block(E_index, 0, Es.at(i).rows(), NCOL) = Es.at(i) + P_index*Eigen::MatrixXi::Ones(Es.at(i).rows(), NCOL);

    P_index += Ps.at(i).rows();
    E_index += Es.at(i).rows();
  }
}
