#include "reduceModel.h"

void ReduceModel::vertex_area(Eigen::VectorXd& AV, Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
  std::vector<std::vector<int> > VF;
  std::vector<std::vector<int> > VFi;
  igl::vertex_triangle_adjacency(V, F, VF, VFi);

  Eigen::VectorXd AF;
  igl::doublearea(V,F, AF);

  AV.resize(V.rows());

  for(int i=0; i < VF.size(); i++)
  {
    AV[i] = 0;

    for(int j=0; j < VF[i].size(); j++)
    {
      AV[i] += AF(VF[i][j])/2;
    }
  }
}

bool ReduceModel::load_poses()
{
  int f=0;
  std::string posepath = basepath + "-pose-0000.dmat";

  // empty from recent poses
  Poses.clear();
  PosesN.clear();

  Eigen::MatrixXd TM;

  while(igl::file_exists(posepath))
  {
    igl::readDMAT(posepath, TM);

    Eigen::MatrixXd TM2;

    if(maya == false)
    {
      TM2.resize(TM.rows()+4, TM.cols());

      for(int i=0; i < Edges.rows(); i++)
      {
        TM2.block(4*Edges(i,1),0,4,3) = TM.block(4*i, 0,4,3);
      }

      TM2.block(0,0,4,3) << 1,0,0,
                            0,1,0,
                            0,0,1,
                            0,0,0;

      std::cout << TM2 << std::endl;
    }
    else
    {
      TM2 = TM;
    }

    for(int i=0; i<TM2.rows()/4; i++)
    {
      TM2.row(4*i) = scale_factor*TM2.row(4*i);
    }

    PosesN.push_back(TM2);
    Poses.push_back(TM);

    // next pose
    f++;
    std::stringstream n;
    n << std::setfill('0') << std::setw(4) << std::to_string(f);
    posepath = basepath + "-pose-" + n.str() + ".dmat";
  }

  // TODO: add rest pose? we should

  std::cout << BLUERUM(std::to_string(f) + " poses found and loaded") << std::endl;

  return true;
}

bool ReduceModel::load_model(std::string path)
{
  normalise = false;

  int pos = path.find_last_of('.');
  basepath = path.substr(0, pos);
  std::string format = path.substr(pos+1, path.length()-pos-1);

  std::cout << format << std::endl;

  if(format == "ply" || format == "PLY")
  {
    igl::readPLY(path, V, F);
  }
  else if(format == "off" || format == "OFF")
  {
    igl::readOFF(path, V,F);
  }
  else if(format == "OBJ" || format == "obj")
  {
    igl::readOBJ(path, V,F);
  }
  else
  {
    std::cout << "no readable mesh file ending" << std::endl;
  }

  scale_factor = 1;

  // question is how to normalise? bounding box diagonal => 1
  if(normalise)
  {
    Eigen::Vector3d m = V.colwise().minCoeff();
    Eigen::Vector3d M = V.colwise().maxCoeff();

    scale_factor = 1/(M-m).norm();

    V = scale_factor*V;
  }

  //std::cout << V << std::endl;

  //std::cout << F << std::endl;

  // read skeleton
  std::string skelpath = basepath + ".tgf";
  igl::readTGF(skelpath, Nodes, Edges);
  igl::bone_parents(Edges,P);

  Nodes = scale_factor*Nodes;

  // find NodeParents
  PN = Eigen::MatrixXi(Nodes.rows(),1);

  for(int i=0; i<Nodes.rows(); i++)
  {
    int j;

    for(j=0; j<Edges.rows(); j++)
    {
      if(Edges(j,1) == i)
      {
        PN(i) = Edges(j,0);
        break;
      }
    }

    if(j==Edges.rows())
    {
      PN(i) = -1;
      root_node_id = i;
    }
  }

  //std::cout << PN << std::endl;

  // read weights
  std::string weightpath = basepath + "-weights.dmat";
  igl::readDMAT(weightpath, W);

  // calculate Lbs matrix
  igl::lbs_matrix(V,W,M);

  // some colors
  C = Eigen::MatrixXd::Constant(F.rows(),3,1);

  if(W.cols() < Nodes.rows())
  {
    // calculate lbs matrix to transform skeleton
    WC = Eigen::MatrixXd::Zero(Nodes.rows(), Nodes.rows());

    for(int i=0; i < Nodes.rows(); i++)
    {
      WC(i,i) = 1;
      //WC(Edges(i,1),i+1) = 1;
    }
    maya = false;
  }
  else
  {
    maya = true;

    Eigen::VectorXi isendnode = Eigen::VectorXi::Constant(Nodes.rows(), 1);
    int n_endnodes = Nodes.rows();

    for(int j=0; j < Edges.rows(); j++)
    {
      if(isendnode(Edges(j,0))!=0)
      {
        isendnode(Edges(j,0))=0;
        n_endnodes--;
      }
    }

    // resize Node and Edge
    Eigen::MatrixXd ENodes(Nodes.rows() + n_endnodes, Nodes.cols());
    Eigen::MatrixXi EEdges(Edges.rows() + n_endnodes, Edges.cols());

    ENodes.block(0,0, Nodes.rows(), Nodes.cols()) = Nodes;
    EEdges.block(0,0, Edges.rows(), Edges.cols()) = Edges;

    int c=0;

    for(int i=0; i < Nodes.rows(); i++)
    {
      if(isendnode(i)) // add node and edge
      {
        for(int j=0; j < Edges.rows(); j++)
        {
          if(Edges(j,1) == i)
          {
            ENodes.row(Nodes.rows() + c) = 0.2*(Nodes.row(Edges(j,1)) - Nodes.row(Edges(j,0))) + Nodes.row(Edges(j,1));
            EEdges(Edges.rows() + c, 0) = i;
            EEdges(Edges.rows() + c, 1) = Nodes.rows() + c;
            c++;
            break;
          }
        }
      }
    }

    Nodes = ENodes;
    Edges = EEdges;

    // calculate lbs matrix to transform skeleton
    WC = Eigen::MatrixXd::Zero(Nodes.rows(), Nodes.rows()-n_endnodes);

    for(int i=0; i < Edges.rows(); i++)
    {
      WC(Edges(i,1), Edges(i,0)) = 1;
    }

    WC(root_node_id, root_node_id) = 1;
  }

  //WC(root_node_id,0) = 1;

  // std::cout << WC << std::endl;
  // std::cout << Nodes << std::endl;

  igl::lbs_matrix(Nodes, WC, MC);

  //igl::repmat(Nodes.row(root_node_id), Nodes.rows(), 1, move_root_to_zero_matrix);

  // caculate importance of each node (just by vertex area)
  Eigen::VectorXd AV;
  vertex_area(AV, V, F);

  if(maya == false)
  {
    Eigen::MatrixXd II;

    II = AV.transpose()*W;

    I = Eigen::VectorXd(II.cols()+1);

    //std::cout << II << std::endl;
    //std::cout << I << std::endl;

    I(0) = 0;
    I.block(1, 0, II.cols(), 1) = II.transpose();
  }
  else
  {
    // I = Eigen::RowVectorXd::Constant(W.cols(), W.colwise().sum().mean());
    //
    // Eigen::VectorXd WCOL = W.transpose()*AV;
    //
    // for(int i=0; i < WCOL.rows(); i++)
    // {
    //   if(WCOL(i) < 100)
    //     I(i) = WCOL(i);
    // }
    //
    // //I = WC*I;
    //
    // //I = (WC*W.transpose()*AV).transpose();

    //std::cout << AV << std::endl;

    //AV = Eigen::VectorXd::Constant(AV.rows(),1);

    I = (WC*W.transpose()*AV).transpose();

    // for(int i=0; i < I.rows(); i++)
    // {
    //   I(i) = std::log(I(i));
    // }
  }

  //std::cout << I << std::endl;

  //I << 0, II;

  load_poses();

  return true;
}
