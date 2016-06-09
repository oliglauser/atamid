#include <igl/bone_parents.h>

#include "MessageParser.h"

// read in new topology
void update_topology(Eigen::MatrixXi& Edges, Eigen::MatrixXd& Lengths, std::vector<Eigen::Matrix3d>& dR, MessageParser &mp)
{
  using namespace Eigen;

  std::vector<Eigen::Vector2i> vedges;

  dR.clear();

  int max_id = mp.nodes.size();

  Eigen::Matrix3d dRL;

  for (unsigned i = 0; i < mp.nodes.size(); ++i)
  {
    const Node& n = mp.nodes[i];

    Eigen::Vector2i edge;

    bool active = true;
    for (unsigned j=0;j<n.cs.size();++j)
      if (!n.cs.at(j).initialized)
        active = false;

    if (!active && n.cs.size() != 1)
      continue;

    // add node position --- how to get that
    for (unsigned j=0; j<n.cs.size(); ++j)
    {
      if (n.cs.at(j).child_id != -1)
      {
        edge << i,  mp.id2pos.at(n.cs.at(j).child_id);
        vedges.push_back(edge);
      }
      else
      {
        edge << i, max_id++;
        vedges.push_back(edge);
      }

      const Connection& c = n.cs.at(j);

      dRL = AngleAxisd(M_PI*c.child_angle1/180.0, Vector3d::UnitX())*
          AngleAxisd(M_PI*c.child_angle2/180.0, Vector3d::UnitY())*
          AngleAxisd(M_PI*c.child_angle3/180.0, Vector3d::UnitX());

      dR.push_back(dRL);

      //std::cout << dRL << std::endl;
    }
  }

  Edges.resize(vedges.size(), 2);
  Lengths.resize(vedges.size(),1);

  for(int i=0; i<vedges.size(); i++)
  {
    Edges(i,0) = vedges.at(i)[0];
    Edges(i,1) = vedges.at(i)[1];
    Lengths(i,0) = 50;
  }
}

// basically kind of forward kinematics
void update_geometry(Eigen::MatrixXd& Nodes, Eigen::MatrixXi& Edges, std::vector<Eigen::Matrix3d>& dR, Eigen::MatrixXd& Lengths, MessageParser &mp)
{
  using namespace Eigen;

  if(Edges.rows()>0)
  {
    Eigen::MatrixXi Parent;
    igl::bone_parents(Edges, Parent);

    //std::cout << Parent << std::endl;

    std::vector<Eigen::Matrix3d> dRR;
    std::vector<Eigen::Vector3d> vT;
    std::vector<Eigen::Matrix3d> vQ;

    for(int i=0; i<Edges.rows(); i++)
    {
      //std::cout << Edges(i,0) << std::endl;

      const Node& n = mp.nodes[Edges(i,0)];
      Eigen::Matrix3d ROT4;

      // rotate first part
      if(n.angle1!=0 & n.angle2!=0 & n.angle3!=0 & n.angle4!=0)
      {
        ROT4 = AngleAxisd((n.angle1*M_PI)/180.0, Vector3d::UnitX())*AngleAxisd(((90.0-n.angle2)*M_PI)/180.0, Vector3d::UnitZ())*AngleAxisd(((n.angle3-90.0)*M_PI)/180.0, Vector3d::UnitY())*AngleAxisd((n.angle4*M_PI)/180.0, Vector3d::UnitX());
      }
      else
      {
        ROT4 = Eigen::Matrix3d::Identity();
      }

      //std::cout << ROT4 << std::endl;

      dRR.push_back(dR[i]*ROT4);

      vQ.push_back(Eigen::Matrix3d::Zero());
      vT.push_back(Eigen::Vector3d::Zero());
      //std::cout << dRR[i] << std::endl;
    }

    std::vector<bool> computed(Edges.rows(),false);

    // calculate and display new skeleton
    Nodes.resize(Edges.rows()+1,3);

    std::function<void (int) > fk_helper = [&] (int b)
    {
      //std::cout << b << std::endl;

      if(!computed[b])
      {
        if(Parent(b) < 0)
        {
          vQ[b] = dRR[b];
          const Vector3d r = Lengths(b,0)*Vector3d::UnitX();
          vT[b] = r;

          //std::cout << vQ[b] << std::endl;
          //std::cout << vT[b] << std::endl;

          Nodes.row(Edges(b,0)) = (Eigen::Vector3d::Zero()).transpose();
          Nodes.row(Edges(b,1)) = vT[b].transpose();

          //std::cout << Nodes << std::endl;
        }else
        {
          // Otherwise first compute parent's
          const int p = Parent(b);
          fk_helper(p);
          vQ[b] = vQ[p] * dRR[b];

          //std::cout << vQ[b] << std::endl;

          const Vector3d r = Lengths(b,0)*Vector3d::UnitX();
          vT[b] = vT[p] + vQ[b]*r;
          Nodes.row(Edges(b,1)) = vT[b].transpose();
        }
        computed[b] = true;
      }
    };

    for(int b = 0; b < Edges.rows(); b++)
    {
      fk_helper(b);
    }
  }
}
