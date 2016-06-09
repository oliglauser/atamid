#include "reducer3.h"

double rotation_angle_from_directions(const Eigen::Matrix<double, 3, 1> v0,
                                                                     const Eigen::Matrix<double, 3, 1> v1)
{
  Eigen::Matrix<double, 3, 1> v0N = v0;
  Eigen::Matrix<double, 3, 1> v1N = v1;

  if(v0.norm() != 0 )
  {
   v0N = v0.normalized();
  }

  if(v1.norm() != 0 )
  {
    v1N = v1.normalized();
  }

  Eigen::VectorXd ax = v0N.cross(v1N);

  double a = std::atan2(ax.norm(), v0N.dot(v1N));

  double no = ax.norm();

  if(no!=0)
  {
    return a;
  }
  else
  {
    return 0;
  }

}

Eigen::MatrixXd rotation_mat_from_directions(const Eigen::Matrix<double, 3, 1> v0,
                                                                     const Eigen::Matrix<double, 3, 1> v1)
{
  Eigen::Matrix<double, 3, 1> v0N = v0;
  Eigen::Matrix<double, 3, 1> v1N = v1;

  if(v0.norm() != 0 )
  {
   v0N = v0.normalized();
  }

  if(v1.norm() != 0 )
  {
    v1N = v1.normalized();
  }

  Eigen::VectorXd ax = v0N.cross(v1N);

  double a = std::atan2(ax.norm(), v0N.dot(v1N));

  double no = ax.norm();

  Eigen::Matrix3d mat;
  if(no!=0)
  {
    mat = Eigen::AngleAxisd(a, (1/no)*ax);
  }
  else
  {
    mat = Eigen::MatrixXd::Identity(3,3);
  }

  return mat;
}

void Reducer3::AddFlexibleNode(int id)
{
  std::cout << rigid_nodes << std::endl;

  rigid_nodes(id) = 0;
  rigid_nodes_n = rigid_nodes;

  std::cout << "----" << std::endl;

  std::cout << rigid_nodes << std::endl;

  std::cout << "****" << std::endl;

  Reduction r;
  setup_variables(rigid_nodes_n, r);

  SEdges = r.E;
  M = r.M;
  R_best = r.R;
}

void Reducer3::SetJointImportance(int id, double w)
{
  std::cout << "added node " <<  id << " as very important" << std::endl;

  JI(id) = w;

  IM = Eigen::SparseMatrix<double>(3*n_complex, 3*n_complex);

  IM.setZero();

  for(int c=0; c < n_complex; c++) // add up all the complex pose position (errors)
  {
    IM.insert(3*c, 3*c) = JI(c)*model->I(c);
    IM.insert(3*c+1, 3*c+1) = JI(c)*model->I(c);
    IM.insert(3*c+2, 3*c+2) = JI(c)*model->I(c);

    //IM.block(3*c,3*c,3,3) = model->I(c)*Eigen::MatrixXd::Identity(3,3);
  }
}

Reducer3::Reducer3(ReduceModel *model_in)
{
  n_TUT = -1;

  colors.resize(12,3);

  colors << 1,0,0,
            0,1,0,
            0,0,1,
            1,1,0,
            0,1,1,
            1,0,1,
            0.5,0,0,
            0.5,0.5,0,
            0,0.5,0,
            0.5,0,0.5,
            0,0.5,0.5,
            0,0,0.5;

  rl = 0.001;

  last_reduction_error = 0;

  Eigen::MatrixXd temp(3,3);

  temp << 0, 0, 0,
          0, 0, -1,
          0, 1, 0;

  J.push_back(temp);

  temp << 0, 0, 1,
          0, 0, 0,
          -1, 0, 0;

  J.push_back(temp);

  temp << 0, -1, 0,
          1, 0, 0,
          0, 0, 0;

  J.push_back(temp);

  model = model_in;

  n_complex = model->Nodes.rows();
  n_poses = model->PosesN.size();

  splitter_library_geometry.push_back(Eigen::RowVector3d(0,0,0));
  splitter_library_n.push_back(n_complex);

  joints_left = 0;

  Eigen::MatrixXi line(0,2);
  Eigen::MatrixXd points(0,3);

  SLines.push_back(line);
  SPoints.push_back(points);
  SColors.push_back(points);

  rigid_nodes = Eigen::VectorXi::Constant(n_complex, 1);
  rigid_nodes_f = Eigen::VectorXi::Constant(n_complex, 0);

  if(model->maya)
  {
    // first count kids
    Eigen::VectorXi Nkids = Eigen::VectorXi::Constant(model->Nodes.rows(),0);

    for(int j=0; j < model->Edges.rows(); j++)
    {
      Nkids(model->Edges(j,0)) = Nkids(model->Edges(j,0)) + 1;
    }

    for(int j=0; j < model->Nodes.rows(); j++)
    {
      if(Nkids(j) > 1)
      {
        for(int i=0; i < model->Edges.rows(); i++)
        {
          if(model->Edges(i,0) == j)
          {
            rigid_nodes_f(model->Edges(i,1)) = 1;
          }
        }
      }
    }
  }

  rigid_nodes(model->root_node_id) = 0;
  rigid_nodes_f(model->root_node_id) = 1;

  rigid_nodes_n  = rigid_nodes;

  JI = Eigen::VectorXd::Constant(n_complex,1);

  n =  Eigen::VectorXd::Constant(3*n_complex, 0);
  IM = Eigen::SparseMatrix<double>(3*n_complex, 3*n_complex);

  IM.setZero();

  for(int c=0; c < n_complex; c++) // add up all the complex pose position (errors)
  {
    IM.insert(3*c, 3*c) = JI(c)*model->I(c);
    IM.insert(3*c+1, 3*c+1) = JI(c)*model->I(c);
    IM.insert(3*c+2, 3*c+2) = JI(c)*model->I(c);

    //IM.block(3*c,3*c,3,3) = model->I(c)*Eigen::MatrixXd::Identity(3,3);
  }

  // std::cout << IM << std::endl;

  e = Eigen::VectorXd::Constant(3*n_complex, 0);

  double average_e = 0;

  for(int j=0; j < model->Edges.rows(); j++)
  {
    e.block(3*model->Edges(j,1), 0, 3, 1) = (model->Nodes.row(model->Edges(j,1)) - model->Nodes.row(model->Edges(j,0))).transpose();
    average_e+=e.block(3*j, 0, 3, 1).norm();
  }

  e.block(3*model->root_node_id, 0, 3, 1) = model->Nodes.row(model->root_node_id).transpose() - model->Nodes.row(model->root_node_id).transpose();
  average_e+=e.block(3*model->root_node_id, 0, 3, 1).norm();

  average_e = average_e / model->Edges.rows();

  VNodes = Eigen::MatrixXd::Constant(n_complex, 3, 0);
  vstack = Eigen::VectorXd::Constant(3*n_complex, 0);

  VNodes2 = Eigen::MatrixXd::Constant(n_complex, 3, 0);
  vstack2 = Eigen::VectorXd::Constant(3*n_complex, 0);

  Eigen::Vector3d a,b,c, d;

  for(int j=0; j < model->Nodes.rows(); j++)
  {
    if(e.block(3*j, 0, 3, 1).norm() == 0)
    {
      VNodes.row(j) = model->Nodes.row(j) + 0.5*average_e*Eigen::RowVector3d(1,0,0);
      VNodes2.row(j) = model->Nodes.row(j) + 0.5*average_e*Eigen::RowVector3d(0,1,0);
    }
    else
    {
      a = e.block(3*j, 0, 3, 1);
      b = Eigen::Vector3d(1, 0, 0);
      c = a.cross(b);

      if(c.norm()==0)
      {
        b = Eigen::Vector3d(0, 1, 0);
        c = a.cross(b);
      }

      d = a.cross(c);

      VNodes.row(j) = model->Nodes.row(j) + 0.5*average_e*c.normalized().transpose();
      VNodes2.row(j) = model->Nodes.row(j) + 0.5*average_e*d.normalized().transpose();
    }

    vstack.block(3*j, 0, 3, 1) = VNodes.row(j).transpose() - model->Nodes.row(model->root_node_id).transpose();
    vstack2.block(3*j, 0, 3, 1) = VNodes2.row(j).transpose() -  model->Nodes.row(model->root_node_id).transpose();
  }

  Eigen::SparseMatrix<double> K_init(3*n_complex, 3*n_complex); // = Eigen::MatrixXd::Constant(3*n_complex, 3*n_complex, 0);

  for(int j=0; j<n_complex; j++)
  {
    int p = -1;

    for(int k=0; k < model->Edges.rows(); k++)
    {
      if(model->Edges(k,1) == j)
      {
        p = model->Edges(k,0);
        break;
      }
    }

    if(p == -1)
    {
      continue;
    }

    while(p != model->root_node_id)
    {
      K_init.insert(3*j, 3*p) = 1;
      K_init.insert(3*j+1, 3*p+1) = 1;
      K_init.insert(3*j+2, 3*p+2) = 1;

      //.block(3*j, 3*p, 3, 3) = Eigen::MatrixXd::Identity(3, 3);

      for(int k=0; k < model->Edges.rows(); k++)
      {
        if(model->Edges(k,1) == p)
        {
          p = model->Edges(k,0);
          break;
        }
      }
    }

    K_init.insert(3*j, 3*model->root_node_id) = 1;
    K_init.insert(3*j+1, 3*model->root_node_id+1) = 1;
    K_init.insert(3*j+2, 3*model->root_node_id+2) = 1;
    //K_init.block(3*j, 3*(model->root_node_id), 3, 3) = Eigen::MatrixXd::Identity(3, 3);
  }

  Eigen::SparseMatrix<double> I_K(K_init.rows(), K_init .cols());
  I_K.setIdentity();

  K.setZero();
  KV.setZero();

  K = K_init + I_K;

  KV = K_init; //K - Eigen::MatrixXd::Identity(K_init.rows(), K_init.cols());

  // std::cout << K << std::endl;
  // std::cout << KV << std::endl;

  Reduction r;
  setup_variables(rigid_nodes_n, r);

  SEdges = r.E;
  M = r.M;
  R_best = r.R;

  std::vector<Eigen::MatrixXd> ROPT;
  double cost;

  Eigen::Vector3d m = model->V.colwise().minCoeff();
  Eigen::Vector3d L = model->V.colwise().maxCoeff();

  normalizef = 1/(L-m).norm();

    std::cout << normalizef << std::endl;

  last_reduction_error = FindRotations(r, ROPT, cost);

  //normalizef = 10*normalizef;
}

void Reducer3::ExportSetup()
{
  // here all information of the reduced skeleton will be exported, so that it
  // can drive a puppet in maya

  // calculate rotation matrix for each joint in local COS, in the same get
  // local cooridnate system in global

  // start from root and push COS and restpose rotation through whole skeleton

  Eigen::MatrixXd JLCOS = Eigen::MatrixXd::Constant(3*model->Nodes.rows(), 3, 0);
  Eigen::MatrixXd JRP = Eigen::MatrixXd::Constant(3*model->Nodes.rows(), 3, 0);
  Eigen::MatrixXd JRPL = Eigen::MatrixXd::Constant(3*model->Nodes.rows(), 3, 0);

  int rid = model->root_node_id;

  // 2 identify splitters in reduced skeleton
  std::vector<std::vector<Eigen::RowVector3d> > no_b; // all outgoing branches in reduced restpose
  std::vector<std::vector<int> > no_id; // id of first outgoing branch in reduced restpose
  std::vector<int> ni_id; // id of first outgoing branch in reduced restpose
  Eigen::MatrixXd ni_b = Eigen::MatrixXd::Zero(model->Nodes.rows(), 3); // incoming branch for each node in reduced restpose
  Eigen::MatrixXd ni_b_c = Eigen::MatrixXd::Zero(model->Nodes.rows(), 3); // incoming branch for each node in reduced restpose

  // setup with empty vectors
  for(int j=0; j < model->Nodes.rows(); j++)
  {
    std::vector<Eigen::RowVector3d> bout;
    std::vector<int> iout;

    no_b.push_back(bout);
    no_id.push_back(iout);
    ni_id.push_back(-1);
  }

  for(int j=0; j < model->Edges.rows(); j++)
  {
    Eigen::RowVector3d bout = model->Nodes.row(model->Edges(j,1)) - model->Nodes.row(model->Edges(j,0));
    ni_b_c.row(model->Edges(j,1)) = bout.normalized();
  }


  // go through reduced skeleton to populate node_o, node_o_id, node_i
  for(int j=0; j < SEdges.rows(); j++)
  {
    Eigen::RowVector3d bout = model->Nodes.row(SEdges(j,1)) - model->Nodes.row(SEdges(j,0));

    no_b[SEdges(j,0)].push_back(bout.normalized());
    no_id[SEdges(j,0)].push_back(SEdges(j,1));
    ni_id[SEdges(j,1)] = SEdges(j,0);
    ni_b.row(SEdges(j,1)) = bout.normalized();
  }

  Eigen::VectorXi CP = Eigen::VectorXi::Constant(model->Nodes.rows(), -1);

  for(int i=0; i < model->Edges.rows(); i++)
  {
    CP(model->Edges(i,1)) = model->Edges(i,0);
  }

  Eigen::VectorXi nid2bc = Eigen::VectorXi::Constant(model->Nodes.rows(), -1);

  int n_splitter_branches = 0;

  for(int i=0; i<splitter_best.size(); i++)
  {
    nid2bc(splitter_best[i].node_id) = i;

    if(splitter_best[i].covered > 1)
    {
      n_splitter_branches += splitter_library_geometry[splitter_best[i].library_id].rows();
    }
  }

  Eigen::MatrixXd SOUT;
  SOUT = Eigen::MatrixXd::Constant(n_splitter_branches, 5, 0);

  int line_counter = 0;
  int not_connected = 0;

  for(int i=0; i < splitter_best.size(); i++)
  {
    if(splitter_best[i].covered > 1)
    {
      for(int j=0; j < splitter_library_geometry[splitter_best[i].library_id].rows(); j++)
      {
        if(CP(splitter_best[i].node_id) == -1)
          SOUT(line_counter,0) = model->root_node_id;
        else
          SOUT(line_counter,0) = CP(splitter_best[i].node_id);

        if(splitter_best[i].out_id(j)!=-1)
          SOUT(line_counter,1) = CP(splitter_best[i].out_id(j));
        else
        {
          SOUT(line_counter,1) = -1;
          not_connected++;
        }

        SOUT.block(line_counter, 2, 1, 3) = splitter_library_geometry[splitter_best[i].library_id].row(j);

        line_counter++;
      }
    }
  }

  SOUT.conservativeResize(SEdges.rows()+not_connected, 5);

  for(int i=0; i < SEdges.rows(); i++)
  {
    if(no_b[SEdges(i,0)].size() == 1)
    {
      if(CP(SEdges(i,0))==-1)
        SOUT(line_counter,0) = rid;
      else
        SOUT(line_counter,0) = CP(SEdges(i,0));

      SOUT(line_counter,1) = CP(SEdges(i,1));
      SOUT(line_counter,2) = 0;
      SOUT(line_counter,3) = 0;
      SOUT(line_counter,4) = 0;

      line_counter++;
    }
  }

  std::cout << SOUT << std::endl;

  // first, set local root coordinate system and then continue from there
  Eigen::Matrix<double, 3, 1> a,b;
  Eigen::MatrixXd toX, toB;

  b = Eigen::Vector3d(1,0,0);

  if(no_b[rid].size() == 1) // root is connected to joint
  {
    a = ni_b_c.row(no_id[rid][0]).transpose();
    toX = rotation_mat_from_directions(b, a);

    ni_b_c.row(rid) = ni_b_c.row(no_id[rid][0]).transpose();
    JLCOS.block(3*rid, 0, 3, 3) = toX;
  }
  else if(no_b[rid].size() > 1) // root is connected to splitter
  {
    ni_b_c.row(rid) = (splitter_best[nid2bc(rid)].opt_rot.transpose()*Eigen::Vector3d(1, 0, 0)).transpose();
    JLCOS.block(3*rid, 0, 3, 3) = splitter_best[nid2bc(rid)].opt_rot.transpose();
  }
  else
  {
    std::cout << "disconnected root" << std::endl;
  }

  JRP.block(3*rid, 0, 3, 3) = Eigen::MatrixXd::Identity(3,3);

  std::function<void (int) > push_cos = [&] (int nid)
  {
    Eigen::Matrix3d SROT;

    if(no_b[nid].size() > 1 && rid!=nid)
    {
      double A=0;
      double B=0;

      for(int j=0; j < splitter_best[nid2bc(nid)].branch_ass.cols(); j++)
      {
        if(splitter_best[nid2bc(nid)].out_id(j)!=-1)
        {
          //Eigen::Vector3d skelout = JLCOS.block(3*nid, 0, 3, 3).transpose()*JRP.block(3*nid, 0, 3, 3).transpose()*ni_b_c.row(no_id[nid][splitter_best[nid2bc(nid)].branch_ass(0,j)]).transpose();

          Eigen::Vector3d skelout = JLCOS.block(3*nid, 0, 3, 3).transpose()*
                                    JRP.block(3*nid, 0, 3, 3).transpose()*
                                    ni_b_c.row(splitter_best[nid2bc(nid)].out_id(j)).transpose();

          Eigen::Vector3d libout = Eigen::AngleAxisd(splitter_library_geometry[splitter_best[nid2bc(nid)].library_id](j, 2), Eigen::Vector3d::UnitZ())*
                 Eigen::AngleAxisd(splitter_library_geometry[splitter_best[nid2bc(nid)].library_id](j, 1), Eigen::Vector3d::UnitY())*
                 Eigen::AngleAxisd(splitter_library_geometry[splitter_best[nid2bc(nid)].library_id](j, 0), Eigen::Vector3d::UnitX())*
                 Eigen::Vector3d(1,0,0);

          A += libout(2)*skelout(1) - libout(1)*skelout(2);
          B += libout(1)*skelout(1) + libout(2)*skelout(2);
        }
      }

      double theta = std::atan2(A,B);
      Eigen::Matrix3d RADD;
      RADD = Eigen::AngleAxisd(-theta, Eigen::Vector3d::UnitX());

      JRP.block(3*nid, 0, 3, 3) = JRP.block(3*nid, 0, 3, 3)*
                                  JLCOS.block(3*nid, 0, 3, 3)*
                                  RADD*
                                  JLCOS.block(3*nid, 0, 3, 3).transpose();
    }

    for(int i=0; i < no_b[nid].size(); i++)
    {
      if(no_b[nid].size() > 1)
      {
        int j;

        for(j=0; j < splitter_best[nid2bc(nid)].branch_ass.cols(); j++)
        {
          if(splitter_best[nid2bc(nid)].out_id(j) == no_id[nid][i])
            break;
        }

        SROT = Eigen::AngleAxisd(splitter_library_geometry[splitter_best[nid2bc(nid)].library_id](j, 2), Eigen::Vector3d::UnitZ())*
               Eigen::AngleAxisd(splitter_library_geometry[splitter_best[nid2bc(nid)].library_id](j, 1), Eigen::Vector3d::UnitY())*
               Eigen::AngleAxisd(splitter_library_geometry[splitter_best[nid2bc(nid)].library_id](j, 0), Eigen::Vector3d::UnitX());

        b = JRP.block(3*nid, 0, 3, 3)*JLCOS.block(3*nid, 0, 3, 3)*SROT*Eigen::Vector3d(1,0,0);
        a = ni_b_c.row(no_id[nid][i]).transpose();

        toB = rotation_mat_from_directions(b, a); // this is also the restpose

        JLCOS.block(3*no_id[nid][i], 0, 3, 3) = JRP.block(3*nid, 0, 3, 3)*JLCOS.block(3*nid, 0, 3, 3)*SROT;
        JRP.block(3*no_id[nid][i], 0, 3, 3) = toB;
      }
      else
      {
        b = ni_b_c.row(nid).transpose();
        a = ni_b_c.row(no_id[nid][i]).transpose();

        toB = rotation_mat_from_directions(b, a); // this is also the restpose // plus some rotation (theta) for the joints ahead of splitters

        JLCOS.block(3*no_id[nid][i], 0, 3, 3) = JRP.block(3*nid, 0, 3, 3)*JLCOS.block(3*nid, 0, 3, 3);
        JRP.block(3*no_id[nid][i], 0, 3, 3) = toB;
      }

      push_cos(no_id[nid][i]);
    }
  };

  push_cos(rid);

  Eigen::MatrixXd TUTCOLORS(3,3);

  TUTCOLORS << 0.8,0.2,0.2,
               0.5,0.8,0.2,
               0.12,0.5,0.8;

  Eigen::MatrixXd joint_colors = Eigen::MatrixXd::Constant(model->Nodes.rows(), 3, 0);

  std::function<void (int, int) > push_colors = [&] (int nid, int cid)
  {
    joint_colors.row(nid) = TUTCOLORS.row(cid);

    for(int i=0; i < no_id[nid].size(); i++)
    {
      cid = (cid + 1)%TUTCOLORS.rows();

      push_colors(no_id[nid][i], cid);
    }
  };

  push_colors(rid, 0);

  // std::cout << joint_colors << std::endl;

  // std::cout << JLCOS << std::endl;
  // std::cout << "*-*-*-*-*-*-*-*" << std::endl;
  // std::cout << JRP << std::endl;

  // store JLCOS (dmat), JRP(dmat) and SEdges(tgf) in files

  Eigen::MatrixXd JCLOSR(JRP.rows(), JRP.cols());

  // setup with empty vectors
  for(int j=0; j < model->Nodes.rows(); j++)
  {
    JCLOSR.block(3*j, 0, 3, 3) = JRP.block(3*j, 0, 3, 3)*JLCOS.block(3*j, 0, 3, 3);
    JRPL.block(3*j, 0, 3, 3) = JCLOSR.block(3*j, 0, 3, 3)*JRP.block(3*j, 0, 3, 3)*JCLOSR.block(3*j, 0, 3, 3).transpose();
  }

  Eigen::MatrixXd CO(JRP.rows(), JRP.cols());
  Eigen::MatrixXd RO(JRP.rows(), JRP.cols());

  for(int j=0; j < model->Edges.rows(); j++)
  {
    CO.block(3*model->Edges(j,0),0, 3, 3) = JCLOSR.block(3*model->Edges(j,1),0, 3, 3);
    RO.block(3*model->Edges(j,0),0, 3, 3) = JRP.block(3*model->Edges(j,1),0, 3, 3);
  }

  // std::cout << "*-*-*-*-*-*-*-*" << std::endl;
  // std::cout << JRPL << std::endl;

  bool write = true;

  if(write)
  {
    igl::writeDMAT(model->basepath + "-localcos.dmat", CO);
    //igl::writeDMAT(model->basepath + "-SEdges.dmat", SEdges);
    igl::writeDMAT(model->basepath + "-restpose.dmat", RO);
    igl::writeDMAT(model->basepath + "-splitgeo.dmat", SOUT);
  }

  // ------------------------------------------------------

  std::vector<Eigen::MatrixXd> DSplitters;

  for(int i=0; i < splitter_best.size(); i++)
  {
    DSplitters.push_back(splitter_library_geometry[splitter_best[i].library_id]);
  }

  PVs.clear();
  PFs.clear();
  PCs.clear();

  draw_puppet(PVs, PFs, PCs, SEdges, model->root_node_id, nid2bc, DSplitters, JLCOS, JRP, joint_colors);

  // ------------------------------------------------------

  bool print_cos = false;

  if(print_cos)
  {
    float bone_size = (1/20.0)*(model->V.colwise().maxCoeff() - model->V.colwise().minCoeff()).norm();

    Eigen::MatrixXd points(4,3);
    Eigen::MatrixXi lines(3,2);
    Eigen::MatrixXd colores(3,3);

    /*SPoints.clear();
    SLines.clear();
    SColors.clear();*/

    points.row(0) = model->Nodes.row(rid) - model->Nodes.row(rid);
    points.row(1) = model->Nodes.row(rid) + bone_size*(JRP.block(3*rid, 0, 3, 3)*JLCOS.block(3*rid, 0, 3, 1)).transpose() - model->Nodes.row(rid);
    points.row(2) = model->Nodes.row(rid) + bone_size*(JRP.block(3*rid, 0, 3, 3)*JLCOS.block(3*rid, 1, 3, 1)).transpose() - model->Nodes.row(rid);
    points.row(3) = model->Nodes.row(rid) + bone_size*(JRP.block(3*rid, 0, 3, 3)*JLCOS.block(3*rid, 2, 3, 1)).transpose() - model->Nodes.row(rid);

    lines(0,0) = 0;
    lines(0,1) = 1;
    lines(1,0) = 0;
    lines(1,1) = 2;
    lines(2,0) = 0;
    lines(2,1) = 3;

    colores.row(0) = Eigen::RowVector3d(1,0,0);
    colores.row(1) = Eigen::RowVector3d(0,1,0);
    colores.row(2) = Eigen::RowVector3d(0,0,1);

    SPoints.push_back(points);
    SLines.push_back(lines);
    SColors.push_back(colores);

    // display
    for(int i=0; i < SEdges.rows(); i++)
    {
      points.row(0) = model->Nodes.row(SEdges(i,1)) - model->Nodes.row(rid);
      points.row(1) = model->Nodes.row(SEdges(i,1)) + bone_size*(JRP.block(3*SEdges(i,1), 0, 3, 3)*JLCOS.block(3*SEdges(i,1), 0, 3, 1)).transpose() - model->Nodes.row(rid);
      points.row(2) = model->Nodes.row(SEdges(i,1)) + bone_size*(JRP.block(3*SEdges(i,1), 0, 3, 3)*JLCOS.block(3*SEdges(i,1), 1, 3, 1)).transpose() - model->Nodes.row(rid);
      points.row(3) = model->Nodes.row(SEdges(i,1)) + bone_size*(JRP.block(3*SEdges(i,1), 0, 3, 3)*JLCOS.block(3*SEdges(i,1), 2, 3, 1)).transpose() - model->Nodes.row(rid);

      lines(0,0) = 0;
      lines(0,1) = 1;
      lines(1,0) = 0;
      lines(1,1) = 2;
      lines(2,0) = 0;
      lines(2,1) = 3;

      colores.row(0) = 0.6*Eigen::RowVector3d(1,0,0);
      colores.row(1) = 0.6*Eigen::RowVector3d(0,1,0);
      colores.row(2) = 0.6*Eigen::RowVector3d(0,0,1);

      SPoints.push_back(points);
      SLines.push_back(lines);
      SColors.push_back(colores);

      points.row(1) = model->Nodes.row(SEdges(i,1)) + bone_size*JLCOS.block(3*SEdges(i,1), 0, 3, 1).transpose() - model->Nodes.row(rid);
      points.row(2) = model->Nodes.row(SEdges(i,1)) + bone_size*JLCOS.block(3*SEdges(i,1), 1, 3, 1).transpose() - model->Nodes.row(rid);
      points.row(3) = model->Nodes.row(SEdges(i,1)) + bone_size*JLCOS.block(3*SEdges(i,1), 2, 3, 1).transpose() - model->Nodes.row(rid);

      lines(0,0) = 0;
      lines(0,1) = 1;
      lines(1,0) = 0;
      lines(1,1) = 2;
      lines(2,0) = 0;
      lines(2,1) = 3;

      colores.row(0) = 0.8*Eigen::RowVector3d(1,0,0);
      colores.row(1) = 0.8*Eigen::RowVector3d(0,1,0);
      colores.row(2) = 0.8*Eigen::RowVector3d(0,0,1);

      SPoints.push_back(points);
      SLines.push_back(lines);
      SColors.push_back(colores);
    }
  }
}

int factorial(int n)
{
  return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
}

void Reducer3::FindSplitters()
{
  static int recycled_joints = 0;

  // line plotting variables
  SLines.clear();
  SPoints.clear();
  SColors.clear();

  Eigen::MatrixXi line;
  Eigen::MatrixXd points;

  splitter_best.clear();

  // find the best suiting splitters from the splitter libarary for the current
  // reduced skeleton (in rigid_nodes)

  std::cout << "START FIND SPLITTERS" << std::endl;

  if(n_TUT == -1)
  {
    n_TUT = rigid_nodes_n.rows() - (rigid_nodes_n.sum() + 1);
  }

  int c = 0;

  // 1 setup variables
  Reduction r;
  setup_variables(rigid_nodes_n, r);

  SEdges = r.E;
  M = r.M;
  R_best = r.R;

  // 2 identify splitters in reduced skeleton
  std::vector<std::vector<Eigen::RowVector3d> > node_o; // all outgoing branches in reduced restpose
  std::vector<std::vector<int> > node_o_id; // id of first outgoing branch in reduced restpose
  std::vector<std::vector<int> > node_o_idc; // id of first outgoing branch in reduced restpose
  Eigen::MatrixXd node_i(model->Nodes.rows(), 3); // incoming branch for each node in reduced restpose
  Eigen::MatrixXd node_i_c(model->Nodes.rows(), 3); // incoming branch for each node in reduced restpose
  Eigen::MatrixXd node_i_sc(model->Nodes.rows(), 3); // connection from node to next higher splitter

  // setup with empty vectors
  for(int j=0; j < model->Nodes.rows(); j++)
  {
    std::vector<Eigen::RowVector3d> bout;
    std::vector<int> iout;

    node_o.push_back(bout);
    node_o_id.push_back(iout);
    node_o_idc.push_back(iout);
  }

  for(int j=0; j < model->Edges.rows(); j++)
  {
    Eigen::RowVector3d bout = model->Nodes.row(model->Edges(j,1)) - model->Nodes.row(model->Edges(j,0));

    node_i_c.row(model->Edges(j,1)) = bout;
  }

  // go through reduced skeleton to populate node_o, node_o_id, node_i
  for(int j=0; j < SEdges.rows(); j++)
  {
    Eigen::RowVector3d bout = model->Nodes.row(SEdges(j,1)) - model->Nodes.row(SEdges(j,0));

    node_o[SEdges(j,0)].push_back(bout);
    node_o_id[SEdges(j,0)].push_back(SEdges(j,1));
    node_i.row(SEdges(j,1)) = bout;
  }

  // go through reduced skeleton to populate node_o, node_o_id, node_i
  for(int j=0; j < model->Edges.rows(); j++)
  {
    node_o_idc[model->Edges(j,0)].push_back(model->Edges(j,1));
  }

  // find connections from node to next splitter up in the hierarchy
  for(int j=0; j < node_o_idc.size(); j++)
  {
    if(node_o_idc[j].size() > 1)
    {
      Eigen::RowVector3d start = model->Nodes.row(j);

      for(int i=0; i < node_o_idc[j].size(); i++)
      {
        int k =  node_o_idc[j][i];

        node_i_sc.row(k) = model->Nodes.row(k) - start;

        while(node_o_idc[k].size() == 1)
        {
          k = node_o_idc[k][0];
          node_i_sc.row(k) = model->Nodes.row(k) - start;
        }
      }
    }
  }

  // transform out going branches to coordinate system that aligns in branch
  // with x-axis = incoming branch
  Eigen::Matrix<double, 3, 1> a,b;
  Eigen::MatrixXd toX;

  b << 1, 0, 0;

  std::vector<Eigen::MatrixXd> splitter_skeleton_geometry; // to store all "aligned" outgoing branches
  std::vector<int> splitter_skeleton_node; // store the id of the splitter nodes

  for(int j=0; j < node_o.size(); j++)
  {
    if(node_o[j].size() > 1) // check if mor than one out => splitter
    {
      if(j == model->root_node_id) // special treatment of root
      {
        toX = Eigen::MatrixXd::Identity(3,3);
      }
      else // otherwise get rotation matrix to align in-branch with x-axis
      {
        a = node_i.row(j).transpose();
        toX = rotation_mat_from_directions(a, b);
      }

      Eigen::MatrixXd splitter(node_o[j].size(), 3);

      for(int k=0; k < node_o[j].size(); k++) // rotate outgoing branches
      {
        splitter.row(k) = (node_i_sc.row(node_o_id[j][k])*toX.transpose()).normalized();
      }

      splitter_skeleton_node.push_back(j);
      splitter_skeleton_geometry.push_back(splitter);
    }
  }

  // not sure what T is doing
  std::vector<std::vector<int> > T;
  std::vector<int> Tf;

  for(int j=0; j < node_o.size(); j++)
  {
    if(node_o[j].size() > 1) // check if more than one out => splitter
    {
      T.push_back(Tf);

      for(int i=0; i < node_o[j].size(); i++)
      {
        int is_s = node_o_id[j][i];

        while(node_o[is_s].size() == 1)
        {
          is_s = node_o_id[is_s][0];
        }

        if(node_o[is_s].size() > 1)
        {
          // push back node id
          for(int k=0; k < splitter_skeleton_node.size(); k++)
          {
            if(splitter_skeleton_node[k]==is_s)
            {
              T[T.size()-1].push_back(k);
              break;
            }
          }
        }
        else
        {
          T[T.size()-1].push_back(-1);
          // push back -1
        }
      }
    }
  }

  // at this moment we have splitter_skeleton_geometry (with all the outgoing
  // branches of the skeleton) and splitter_library_geometry (with all the
  // splitter variants available)
  // now for each in splitter skeleton geometry we have to find on how well the
  // library variants match

  // 1 | how well can the poses still be reached if certain branches are removed (min cost)
  // 2 | how many branches have to be removed (max covered)
  // 3 | what's the angle error over all branches (min error)
  // 4 | how many splitter outputs are not used

  int l,s,p;

  std::vector<std::vector<std::vector<splitter_match> > > combos; // store all matching information

  // go through all splitting nodes
  for(int si = 0; si < splitter_skeleton_geometry.size(); si++)
  {
    // will store all combos of this spligging node
    std::vector < std::vector <splitter_match> > si_combos;

    // number of outgoing branches of current node
    s = splitter_skeleton_geometry[si].rows();

    // number of out assignments of current node
    int ths = std::pow(2,s);

    // keep book of what outgoing branch combinations have been
    // caclulated already
    Eigen::MatrixXi bookkeep = Eigen::MatrixXi::Constant(ths, s, 0);
    Eigen::MatrixXd bookcost = Eigen::VectorXd::Constant(ths, 0);
    int bookcounter = 0;

    // go through all splitter geometries
    for(int li = 0; li < splitter_library_geometry.size(); li++)
    {
      std::vector <splitter_match> li_combos;

      // number of outgoing branches of current library splitter
      l = splitter_library_geometry[li].rows();

      Eigen::MatrixXd lib_branch(l,3);

      for(int i=0; i < l; i++)
      {
        lib_branch.row(i) = (Eigen::AngleAxisd(splitter_library_geometry[li](i,2), Eigen::Vector3d::UnitZ())*
                             Eigen::AngleAxisd(splitter_library_geometry[li](i,1), Eigen::Vector3d::UnitY())*
                             Eigen::AngleAxisd(splitter_library_geometry[li](i,0), Eigen::Vector3d::UnitX())*
                             Eigen::Vector3d(1,0,0)).transpose();
      }

      // create matrix of possible permutations
      p = std::max(s,l);

      int perms[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};

      Eigen::MatrixXi P(factorial(p),p);
      int counter = 0;

      std::sort(perms, perms+p);

      for(int m=0; m < p; m++)
        P(counter, m) = perms[m];

      counter++;

      while (std::next_permutation(perms, perms+p))
      {
        for(int m=0; m < p; m++)
          P(counter, m) = perms[m];
        counter++;
      }

      Eigen::MatrixXi E = P.block(0,0,P.rows(),l);

      std::chrono::time_point<std::chrono::steady_clock> start,end;

      //std::cout << P << std::endl;

      for(int o=0; o < P.rows(); o++)
      {
        start = std::chrono::steady_clock::now();

        bool done = false;

        for(int i=0; i < o; i++)
        {
          if((E.row(o)-E.row(i)).norm()==0)
          {
            done=true;
            //end = std::chrono::steady_clock::now();
            //std::cout << "b " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << std::endl;
            break;
          }
        }

        if(done)
          continue;

        start = std::chrono::steady_clock::now();

        splitter_match new_combo;

        new_combo.library_id = li;
        new_combo.splitter_id = si;
        new_combo.node_id = splitter_skeleton_node[si];

        //std::cout << P.row(o) << std::endl;
        double A=0;
        double B=0;

        Eigen::Matrix3d Rtheta;
        Eigen::MatrixXd H = Eigen::MatrixXd::Constant(3,3,0);

        // get optimal rotation, by using svd
        for(int p = 0; p < l; p++)
        {
          if(P(o,p) < s)
          {
            if(splitter_skeleton_node[si] == model->root_node_id)
            {
              H += splitter_skeleton_geometry[si].row(P(o,p)).transpose()*lib_branch.row(p);
            }
            else
            {
              A += lib_branch(p,2)*splitter_skeleton_geometry[si](P(o,p), 1) - lib_branch(p,1)*splitter_skeleton_geometry[si](P(o,p), 2);
              B += lib_branch(p,1)*splitter_skeleton_geometry[si](P(o,p), 1) + lib_branch(p,2)*splitter_skeleton_geometry[si](P(o,p), 2);
            }
          }
        }

        if(splitter_skeleton_node[si] == model->root_node_id)
        {
          Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
          Rtheta = svd.matrixV()*svd.matrixU().transpose();

          if(Rtheta.determinant() < 0)
              Rtheta.col(2) = -Rtheta.col(2);

          new_combo.theta = 0;
        }
        else
        {
          double theta = std::atan2(A,B);
          new_combo.theta = theta;
          Rtheta = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitX());
        }

        double omega;

        new_combo.opt_rot = Rtheta;

        // first find the ones to that have to be rigidified
        Eigen::VectorXi keep = Eigen::VectorXi::Constant(s,0);

        Eigen::VectorXi oid = Eigen::VectorXi::Constant(l,-1);

        // go thrugh the "existing" (<l) splitter positions and compare the angle
        // between them and the assigned skeleton branch
        for(int m=0; m < l; m++)
        {
          if(P(o,m) < s)
          {
            a = lib_branch.row(m).transpose();
            b = Rtheta*splitter_skeleton_geometry[si].row(P(o,m)).transpose();

            omega = acos(a.dot(b)); //rotation_angle_from_directions(a,b);

            if(omega!=omega)
            {
              if(a.dot(b) < -1.0)
              {
                omega = M_PI;
              }
              else if(a.dot(b) > 1.0)
              {
                omega = 0;
              }
            }

            // if(li==0)
            // {
            //   std::cout << a.transpose() << std::endl;
            //   std::cout << splitter_skeleton_geometry[si].row(P(o,m)) << std::endl;
            //   std::cout << Rtheta << std::endl;
            //   std::cout << b.transpose() << std::endl;
            //   std::cout << omega << std::endl;
            //   std::cout << "* * * * * * * *" << std::endl;
            // }

            if(omega < M_PI/2.0)
            {
              //std::cout << omega << std::endl;
              new_combo.angle_error += omega*omega;
              keep(P(o,m)) = 1;
              oid(m) = node_o_id[splitter_skeleton_node[si]][P(o,m)];
              new_combo.covered += 1;
              continue;
            }
          }

          P(o,m) = -1;
        }

        new_combo.out_id = oid;

        if(new_combo.covered==0)
          continue;

        // -  // -  // -  //

        for(int m=l; m < p; m++)
        {
          P(o,m) = -1;
        }

        new_combo.rel_rot_error = new_combo.angle_error;

        // std::cout << "----------" << std::endl;
        // std::cout << si << " : " << li << std::endl;
        // std::cout << new_combo.angle_error << std::endl;

        new_combo.keep = keep;

        bool bookkeeperfound = false;

        // test here if it's the book already
        for(int m=0; m < bookcounter; m++)
        {
          if((bookkeep.row(m)-keep.transpose()).norm()==0)
          {
            new_combo.cost = bookcost(m);
            bookkeeperfound = true;
            break;
          }
        }

        if(!bookkeeperfound)
        {
          // now rigidify the not needed branches and optimise :)
          // get node from here: splitter_skeleton_node[si]
          // order can be derived from SEdges, should be the same
          // all nodes in the not to keep branches are simply set rigid in
          // rigid_nodes_n

          rigid_nodes_n = rigid_nodes;

          setup_variables(rigid_nodes_n, r);

          SEdges = r.E;
          M = r.M;
          R_best = r.R;

          std::function<void (int) > rigidier = [&] (int node)
          {
            rigid_nodes_n(node)=1;
            for(int i=0; i<node_o_id[node].size(); i++)
            {
              rigidier(node_o_id[node][i]);
            }
          };

          for(int m=0; m < s; m++)
          {
            if(keep(m)==0)
            {
              // rigidify all child nodes of this branch:
              rigidier(node_o_id[splitter_skeleton_node[si]][m]);
            }
            else
            {
              // only start rigidfiying after first child splitter
              int n = node_o_id[splitter_skeleton_node[si]][m];

              while(node_o_id[n].size())
              {
                n = node_o_id[n][0];

                if(node_o_id[n].size() == 0)
                {
                  break;
                }

                if(node_o_id[n].size() > 1)
                {
                  for(int i=0; i<node_o_id[n].size(); i++)
                  {
                    rigidier(node_o_id[n][i]);
                  }
                  break;
                }
              }
            }
          }

          //std::cout << "Rigid nodes n" << std::endl << rigid_nodes_n << std::endl;

          // now run optimisation .. guess has to be split first to do that

          setup_variables(rigid_nodes_n, r);

          SEdges = r.E;
          M = r.M;
          R_best = r.R;

          std::vector<Eigen::MatrixXd> Ropt;

          double cost;
          new_combo.cost = FindRotations(r, Ropt, cost);

          bookkeep.row(bookcounter) = keep;
          bookcost(bookcounter) = new_combo.cost;
          bookcounter++;
        }

        std::cout << new_combo.cost << std::endl;
        new_combo.branch_ass = P.row(o);

        li_combos.push_back(new_combo);

        // std::cout << "branching node: " << si << std::endl;
        // std::cout << "splitter type: " << li << std::endl;
        // std::cout << "out id: " << new_combo.out_id << std::endl;
        // std::cout << "covered: " << new_combo.covered << std::endl;
        // std::cout << "branch ass: " << new_combo.branch_ass << std::endl;
        // std::cout << "gain: " << new_combo.gain << std::endl;
        // std::cout << "angle_error: " << new_combo.angle_error << std::endl;
        // std::cout << "------------------------" << std::endl;

        //end = std::chrono::steady_clock::now();
        //std::cout << "A " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << std::endl;
      }

      si_combos.push_back(li_combos);
    }

    combos.push_back(si_combos);
  }

  std::function<void (int) > rigidier = [&] (int node)
  {
    rigid_nodes_n(node)=1;
    for(int i=0; i<node_o_id[node].size(); i++)
    {
      rigidier(node_o_id[node][i]);
    }
  };

  Eigen::VectorXd ref_cost(splitter_skeleton_geometry.size());

  for(int si=0; si < splitter_skeleton_geometry.size(); si++)
  {
    rigid_nodes_n  = rigid_nodes;

    setup_variables(rigid_nodes_n, r);

    SEdges = r.E;
    M = r.M;
    R_best = r.R;

    for(int m=0; m < node_o_id[splitter_skeleton_node[si]].size(); m++)
    {
        // rigidify all child nodes of this branch:
        rigidier(node_o_id[splitter_skeleton_node[si]][m]);
    }

    setup_variables(rigid_nodes_n, r);

    SEdges = r.E;
    M = r.M;
    R_best = r.R;

    std::vector<Eigen::MatrixXd> Ropt;

    double cost;
    ref_cost(si) = FindRotations(r, Ropt, cost);
  }

  rigid_nodes_n  = rigid_nodes;

  setup_variables(rigid_nodes_n, r);

  SEdges = r.E;
  M = r.M;
  R_best = r.R;

  Eigen::MatrixXd colores;

  std::vector<std::vector  <splitter_match> > bc;

  splitter_match worst_combo;
  worst_combo.cost = 10000000;

  std::vector<splitter_match> ev;

  // now find the best:
  for(int si=0; si < splitter_skeleton_geometry.size(); si++)
  {
    bc.push_back(ev);

    for(int li=0; li < splitter_library_geometry.size(); li++)
    {
      bc[si].push_back(worst_combo);
    }
  }

  splitter_match cc;

  std::vector<std::vector<std::vector<splitter_match> > > C;

  std::vector<std::vector<std::vector<GRBVar> > > X;

  std::vector<std::vector<GRBVar > > Xf;
  std::vector<GRBVar> Xff;

  std::vector<std::vector<splitter_match > > Cf;
  std::vector<splitter_match> Cff;

  for(int si=0; si < splitter_skeleton_geometry.size(); si++)
  {
     C.push_back(Cf);
     X.push_back(Xf);

    for(int li=0; li < splitter_library_geometry.size(); li++)
    {
      C[si].push_back(Cff);
      X[si].push_back(Xff);
    }
  }

  // now find the best (can this be made nicer?)
  for(int si=0; si < splitter_skeleton_geometry.size(); si++)
  {
    for(int li=0; li < splitter_library_geometry.size(); li++)
    {
      for(int i=0; i < combos[si][li].size(); i++)
      {
        cc = combos[si][li][i];

        cc.gain = ref_cost(si) - cc.cost;

        if(cc.gain < 0.0)
        {
          cc.gain = 0;
        }

        //fill C

        // std::cout << "----------------------------"  << std::endl;
        //
        // std::cout << si << ":" << li << ":" << i << std::endl;
        // std::cout << cc.keep.transpose() << std::endl;
        // std::cout << cc.branch_ass << std::endl;
        // std::cout << cc.rel_rot_error << std::endl;
        // std::cout << cc.angle_error << std::endl;

        int j = 0;

        for(j=0; j < C[si][li].size(); j++)
        {
          if( ((C[si][li][j].keep - cc.keep).norm() == 0) )
          {
            //if( (cc.angle_error < C[si][li][j].angle_error) )
            if( (cc.rel_rot_error < C[si][li][j].rel_rot_error) )
            {
              // std::cout << "----------------------------"  << std::endl;
              //
              // std::cout << li << ":" << j << std::endl;
              // std::cout << cc.keep.transpose() << ":" << C[si][li][j].keep.transpose() << std::endl;
              // std::cout << cc.rel_rot_error << ":" << C[si][li][j].rel_rot_error << std::endl;
              // std::cout << cc.angle_error << ":" << C[si][li][j].angle_error << std::endl;
              C[si][li][j] = cc;
            }
            break;
          }
        }

        if(j == C[si][li].size())
        {
          C[si][li].push_back(cc);
          //std::cout << "----------------------------"  << std::endl;

          // std::cout << li << ":" << j << std::endl;
          // std::cout << cc.keep.transpose() << ":" << C[si][li][j].keep.transpose() << std::endl;
          // std::cout << cc.rel_rot_error << ":" << C[si][li][j].rel_rot_error << std::endl;
          // std::cout << cc.angle_error << ":" << C[si][li][j].angle_error << std::endl;
        }
      }
    }
  }

  // int lib_size = 0;
  //
  // for(int i=0; i < splitter_library_n.size(); i++)
  // {
  //   lib_size += splitter_library_n[i];
  // }
  //
  // int skel_size = splitter_skeleton_geometry.size();
  //
  // p = std::max(lib_size, skel_size);

  GRBEnv denv = GRBEnv();
  denv.set(GRB_IntParam_OutputFlag, 0);
  GRBModel dmodel = GRBModel(denv);

  // now find the best (can this be made nicer?)
  for(int si=0; si < splitter_skeleton_geometry.size(); si++)
  {
    for(int li=0; li < splitter_library_geometry.size(); li++)
    {
      for(int k=0; k < C[si][li].size(); k++)
      {
        std::cout << "branching node: " << si << std::endl;
        std::cout << "splitter type: " << li << std::endl;
        std::cout << "out id: " << C[si][li][k].out_id << std::endl;
        std::cout << "covered: " << C[si][li][k].covered << std::endl;
        std::cout << "branch ass: " << C[si][li][k].branch_ass << std::endl;
        std::cout << "gain: " << C[si][li][k].gain << std::endl;
        std::cout << "angle_error: " << C[si][li][k].angle_error << std::endl;
        std::cout << "------------------------" << std::endl;

        std::stringstream nn;
        nn << "x_" << si << "_" << li << "_" << k;
        GRBVar v = dmodel.addVar(0.0, 1.0, 0.0, GRB_BINARY, nn.str());
        X[si][li].push_back(v);
      }
    }
  }

  dmodel.update();

  // snode constraints
  for(int si=0; si < splitter_skeleton_geometry.size(); si++)
  {
    GRBLinExpr lhs = 0;
    for(int li=0; li < splitter_library_geometry.size(); li++)
    {
      for(int k=0; k < X[si][li].size(); k++)
      {
        lhs += X[si][li][k];
      }
    }
    dmodel.addConstr(lhs, GRB_LESS_EQUAL, 1.0);
  }

  // splitter constraints
  for(int li=0; li < splitter_library_geometry.size(); li++)
  {
    GRBLinExpr lhs = 0;
    for(int si=0; si < splitter_skeleton_geometry.size(); si++)
    {
      for(int k=0; k < X[si][li].size(); k++)
      {
        lhs += X[si][li][k];
      }
    }
    dmodel.addConstr(lhs, GRB_LESS_EQUAL, splitter_library_n[li]);
  }

  // hierarchy constraints
  for(int si=0; si < splitter_skeleton_geometry.size(); si++)
  {
    for(int m=0; m < splitter_skeleton_geometry[si].rows(); m++)
    {
      if(T[si][m]!=-1)
      {
        GRBLinExpr rhs = 0;
        for(int li=0; li < splitter_library_geometry.size(); li++)
        {
          for(int k=0; k < X[si][li].size(); k++)
          {
            rhs += C[si][li][k].keep(m)*X[si][li][k];
          }
        }

        GRBLinExpr lhs = 0;
        for(int li=0; li < splitter_library_geometry.size(); li++)
        {
          for(int k=0; k < X[T[si][m]][li].size(); k++)
          {
            lhs += X[T[si][m]][li][k];
          }
        }

        dmodel.addConstr(lhs-rhs, GRB_LESS_EQUAL, 0);
      }
    }
  }

  GRBLinExpr objd;

  // now find the best (can this be made nicer?)
  for(int si=0; si < splitter_skeleton_geometry.size(); si++)
  {
    for(int li=0; li < splitter_library_geometry.size(); li++)
    {
      for(int k=0; k < X[si][li].size(); k++)
      {
        // TODO: enforce more or less splitters here? now more
        objd += X[si][li][k]*(C[si][li][k].gain + 1);
        ///std::cout << C[si][li][k].gain << std::endl;
      }
    }
  }

  dmodel.setObjective(objd, GRB_MAXIMIZE);

  std::cout << "GUROBI" << std::endl;

  dmodel.update();
  dmodel.optimize();

  double nval, oval;
  oval = dmodel.get(GRB_DoubleAttr_ObjVal);

  std::vector<Eigen::MatrixXi > solutions;

  do {
    Eigen::MatrixXi sol = Eigen::MatrixXi::Constant(splitter_skeleton_geometry.size(),2,-1);

    GRBLinExpr lhs = 0;
    int c=0;
    // now find the best (can this be made nicer?)
    for(int si=0; si < splitter_skeleton_geometry.size(); si++)
    {
      for(int li=0; li < splitter_library_geometry.size(); li++)
      {
        for(int k=0; k < X[si][li].size(); k++)
        {
          //std::cout << X[si][li][k].get(GRB_DoubleAttr_X);

          if(X[si][li][k].get(GRB_DoubleAttr_X)==1)
          {
            lhs += X[si][li][k];
            sol(si,0) = li;
            sol(si,1) = k;
            c++;
          }
        }
      }
    }

    std::cout << "SOLUTION " << solutions.size() << "  - -" << std::endl;
    std::cout << sol << std::endl;
    std::cout << "- - - - - - - -" << std::endl;

    solutions.push_back(sol);

    dmodel.addConstr(lhs, GRB_LESS_EQUAL, c-1);
    dmodel.update();
    dmodel.reset();
    dmodel.optimize();

    nval = dmodel.get(GRB_DoubleAttr_ObjVal);
  }
  while(nval==oval);

  double min_angle_error = 1000000;
  int max_covered = 0;
  int best_id = 0;

  for(int i=0; i < solutions.size(); i++)
  {
    double angle_error = 0;
    int covered = 0;

    for(int j=0; j < solutions[i].rows(); j++)
    {
      if(solutions[i](j,0)!=-1)
      {
        std::cout << j << " : " << solutions[i](j,0) << " : " << solutions[i](j,1)  << " : " << C[j][solutions[i](j,0)][solutions[i](j,1)].angle_error << std::endl;
        angle_error += C[j][solutions[i](j,0)][solutions[i](j,1)].angle_error/C[j][solutions[i](j,0)][solutions[i](j,1)].covered;
        covered += C[j][solutions[i](j,0)][solutions[i](j,1)].covered;
      }
    }
    //
    std::cout << angle_error << " | " << covered << std::endl;
    std::cout << "- - - - " << std::endl;

    if(covered > max_covered || (angle_error < min_angle_error && covered == max_covered) )
    {
      min_angle_error = angle_error;
      max_covered = covered;
      best_id = i;
    }
  }

  splitter_match ns;

  ns.library_id=-1;

  for(int j=0; j < solutions[best_id].rows(); j++)
  {
    if(solutions[best_id](j,0)!=-1)
      splitter_best.push_back(C[j][solutions[best_id](j,0)][solutions[best_id](j,1)]);
    else
      splitter_best.push_back(ns);
  }

  int splitted_joints = 0;

  std::vector<int> wasted_joints;

  std::function<void (int) > rigidierE = [&] (int node)
  {
    if(rigid_nodes_n(node) != 1)
      splitted_joints++;

    rigid_nodes_n(node)=1;
    wasted_joints.push_back(node);

    for(int i=0; i<node_o_id[node].size(); i++)
    {
      rigidierE(node_o_id[node][i]);
    }
  };

  rigid_nodes_n  = rigid_nodes;

  setup_variables(rigid_nodes_n, r);

  SEdges = r.E;
  M = r.M;
  R_best = r.R;

  for(int si=0; si < splitter_skeleton_geometry.size(); si++)
  {
    if(splitter_best[si].library_id!=-1)
    {
       // rigidfy the ones not to keep
       for(int m=0; m < node_o_id[splitter_skeleton_node[si]].size(); m++)
       {
         if(splitter_best[si].keep(m)==0)
         {
           // rigidify!!
           rigidierE(node_o_id[splitter_skeleton_node[si]][m]);
         }
       }
    }
    else
    {
      // rigidfy all if no splitter set
      for(int m=0; m < node_o_id[splitter_skeleton_node[si]].size(); m++)
      {
          // rigidify!!
          rigidierE(node_o_id[splitter_skeleton_node[si]][m]);
      }
    }
  }

  std::cout << "Wasted Joints: " << splitted_joints << std::endl;
  std::cout << rigid_nodes_n.sum() + 1 - rigid_nodes_f.sum() << std::endl;

  std::cout << (rigid_nodes_n.rows() - (rigid_nodes_n.sum() + 1)) << std::endl;

  std::cout << n_TUT << std::endl;

  if((n_TUT > (rigid_nodes_n.rows() - (rigid_nodes_n.sum() + 1))) && (rigid_nodes_n.sum() + 1 > rigid_nodes_f.sum()))
  {
    recycled_joints = n_TUT - (rigid_nodes_n.rows() - (rigid_nodes_n.sum() + 1));

    if(recycled_joints > rigid_nodes_n.sum() + 1 - rigid_nodes_f.sum())
      recycled_joints = rigid_nodes_n.sum() + 1 - rigid_nodes_f.sum();

    //std::cout << recycled_joints << std::endl;

    for(int i=0; i < recycled_joints; i++)
    {
      ReductionStep();
      //std::cout << reduction_order.size()-1-i << std::endl;
      //std::cout << reduction_order[reduction_order.size()-1-i] << std::endl;
      //rigid_nodes(reduction_order[reduction_order.size()-1-i]) = 0;
    }

    rigid_nodes_n = rigid_nodes;

    setup_variables(rigid_nodes_n, r);

    SEdges = r.E;
    M = r.M;
    R_best = r.R;

    FindSplitters();
  }
  else
  {
    n_TUT = -1;

    rigid_nodes  = rigid_nodes_n;

    setup_variables(rigid_nodes_n, r);

    SEdges = r.E;
    M = r.M;

    std::vector<Eigen::MatrixXd> Ropt;
    double cost;
    FindRotations(r, Ropt, cost);

    R_best = Ropt;

    float axis_size = (1/10.0)*(model->V.colwise().maxCoeff() - model->V.colwise().minCoeff()).norm();

    // print them
    for(int si=0; si < splitter_skeleton_geometry.size(); si++)
    {
      if(splitter_best[si].library_id==-1)
        continue;

      //std::cout << splitter_best[si].opt_rot << std::endl;
      //std::cout << splitter_best[si].theta << std::endl;

      Eigen::MatrixXd slb = splitter_library_geometry[splitter_best[si].library_id];

      // std::cout << slb << std::endl;

      points.resize(slb.rows()+2,3);
      line.resize(slb.rows()+1,2);
      colores.resize(slb.rows()+1,3);

      Eigen::MatrixXd a,b,c, toX;

      c = Eigen::Vector3d(1,0,0);

      points.row(0) = model->Nodes.row(splitter_skeleton_node[si]) - model->Nodes.row(model->root_node_id);

      if(splitter_best[si].node_id!=model->root_node_id)
      {
        b = node_i.row(splitter_best[si].node_id).transpose();
        toX = rotation_mat_from_directions(c, b);
      }
      else
      {
        toX = Eigen::MatrixXd::Identity(3,3);
      }

      for(int i=0; i < slb.rows(); i++)
      {
        a = (toX*splitter_best[si].opt_rot.transpose()*
             Eigen::AngleAxisd(slb(i,2), Eigen::Vector3d::UnitZ())*
             Eigen::AngleAxisd(slb(i,1), Eigen::Vector3d::UnitY())*
             Eigen::AngleAxisd(slb(i,0), Eigen::Vector3d::UnitX())*
             Eigen::Vector3d(1,0,0)).transpose();

        points.row(i+1) = model->Nodes.row(splitter_best[si].node_id) + axis_size*a - model->Nodes.row(model->root_node_id);
        line(i,0) = 0;
        line(i,1) = i+1;

        if(splitter_best[si].branch_ass(i)==-1)
          colores.row(i) = colors.row(5);
        else
          colores.row(i) = colors.row(6);
      }

      a = (toX*splitter_best[si].opt_rot.transpose()*Eigen::Vector3d(-1,0,0)).transpose();
      points.row(slb.rows()+1) = model->Nodes.row(splitter_best[si].node_id) + axis_size*a - model->Nodes.row(model->root_node_id);
      line(slb.rows(),0) = 0;
      line(slb.rows(),1) = slb.rows()+1;
      colores.row(slb.rows()) = colors.row(7);

      SPoints.push_back(points);
      SLines.push_back(line);
      SColors.push_back(colores);
    }
  }
}


void Reducer3::setup_variables(Eigen::VectorXi& rigid_nodes_new, Reduction &r)
{
  r.RN = rigid_nodes_new;

  // till here it's the same for every simplification
  //Eigen::MatrixXd M_init = Eigen::MatrixXd::Identity(3*n_complex, 3*n_complex);
  //r.M = Eigen::MatrixXd::Constant(3*n_complex, 3*n_complex, 0);

  r.M = Eigen::SparseMatrix <double> (3*n_complex, 3*n_complex);
  r.M.setZero();

  for(int j=0; j<n_complex; j++)
  {
    int p = j;

    if(r.RN(j))
    {
      for(int k=0; k < model->Edges.rows(); k++)
      {
        if(model->Edges(k,1) == j)
        {
          p = model->Edges(k,0);
          break;
        }
      }

      while(r.RN(p))
      {
        for(int k=0; k < model->Edges.rows(); k++)
        {
          if(model->Edges(k,1) == p)
          {
            p = model->Edges(k,0);
            break;
          }
        }
      }
    }

    r.M.insert(3*j, 3*p) = 1;
    r.M.insert(3*j+1, 3*p+1) = 1;
    r.M.insert(3*j+2, 3*p+2) = 1;

    // //K.block(3*j, 0, 3, 3*n_complex) = K_init.block(3*p, 0, 3, 3*n_complex);
    // r.M.block(3*j, 0, 3, 3*n_complex) = M_init.block(3*p, 0, 3, 3*n_complex);
  }

  // ======== get "simple"

  // first count kids
  Eigen::MatrixXi PEdges = model->Edges;
  Eigen::VectorXi Nkids = Eigen::VectorXi::Constant(model->Nodes.rows(),0);

  for(int j=0; j < model->Edges.rows(); j++)
  {
    Nkids(model->Edges(j,0)) = Nkids(model->Edges(j,0)) + 1;
  }

  int bs = PEdges.rows();

  for(int j=0; j < PEdges.rows(); j++)
  {
    if(r.RN(PEdges(j,1)))
    {
      int k;

      for(k=0; k < PEdges.rows(); k++)
      {
        if(PEdges(k,0) == PEdges(j,1))
        {
          PEdges(k,0) = PEdges(j,0);
        }
      }

      PEdges(j,0) = -1;
      PEdges(j,1) = -1;
      bs--;
    }
  }

  r.E = Eigen::MatrixXi::Constant(bs,2,0);

  int counter = 0;

  for(int j=0; j < PEdges.rows(); j++)
  {
    if(PEdges(j,0)!=-1)
    {
      r.E.row(counter) = PEdges.row(j);
      counter++;
    }
  }

  Eigen::Matrix<double, 3, 1> a,b;

  Eigen::MatrixXd PN;

  // now calculate R init for each pose!
  for(int k=0; k < model->Poses.size(); k++)
  {
    PN = model->MC*model->PosesN[k];// - model->Nodes.row(model->root_node_id);
    Eigen::MatrixXd Rk = Eigen::MatrixXd::Constant(3*n_complex, 3, 0);

    for(int j=0; j < model->Edges.rows(); j++)
    {
      if(r.RN(model->Edges(j,1)) == 1)
      {
        Rk.block(3*model->Edges(j,1), 0, 3, 3) = Eigen::MatrixXd::Identity(3,3);
      }
      else
      {
        a = (model->Nodes.row(model->Edges(j,1)) - model->Nodes.row(model->Edges(j,0))).transpose(); // - model->Nodes.row(model->root_node_id).transpose();
        b = (PN.row(model->Edges(j,1)) - PN.row(model->Edges(j,0))).transpose(); //- PosedNodes.row(model->root_node_id).transpose();

        Rk.block(3*model->Edges(j,1), 0, 3, 3) =  rotation_mat_from_directions(a, b); // Eigen::MatrixXd::Identity(3,3); //
      }
    }

    Rk.block(3*model->root_node_id, 0, 3, 3) = Eigen::MatrixXd::Identity(3,3); //rotation_mat_from_directions(a, b);

    r.R.push_back(Rk);
  }
}

bool Reducer3::ReductionStep()
{
  RC.clear();
  CPoints.clear();
  CPointsC.clear();

  //if(n_complex - rigid_nodes.sum() - 1 == 0)
  if(rigid_nodes.sum() == rigid_nodes_f.sum()-1)
  {
    std::cout << "OVER: no more joints to add" << std::endl;
    return false;
  }

  // generate vector of possible reductions (which joints to rigidify)
  std::vector<int> reduction_candidate_nodes;
  std::vector<double> reduction_candidate_cost;
  std::vector<std::vector<Eigen::MatrixXd> > Ropt;

  std::vector<Eigen::MatrixXd> Roptt;

  for(int i=0; i < n_complex; i++)
  {
    if(rigid_nodes(i)!=0)
    {
      if(rigid_nodes_f(i)==0)
      {
        reduction_candidate_nodes.push_back(i);
        reduction_candidate_cost.push_back(-1);
        Ropt.push_back(Roptt);
      }
    }
  }

  // add the new one :)

  Eigen::VectorXd reduction_candidate_error = Eigen::VectorXd::Constant(reduction_candidate_nodes.size(), -1);

  double ERROR_MIN = 1000000;

  std::thread *t = new std::thread[reduction_candidate_nodes.size()];

  std::chrono::time_point<std::chrono::steady_clock> start,end;
  start = std::chrono::steady_clock::now();

  std::cout << "FIND NEXT REDUCTION" << std::endl;

  // go through candidates and find best
  for(int i=0; i < reduction_candidate_nodes.size(); i++)
  {
    Reduction r;

    rigid_nodes_n = rigid_nodes;

    rigid_nodes_n(reduction_candidate_nodes[i]) = 0;

    setup_variables(rigid_nodes_n, r);

    // std::bind binds it to the current instance: this
    // std::ref passes them by reference

    //std::cout << r.M << std::endl;

    std::cout << "start try node " << reduction_candidate_nodes[i] << std::endl;

    t[i] = std::thread(std::bind(&Reducer3::FindRotations,this, r, std::ref(Ropt[i]), std::ref(reduction_candidate_cost[i])));
  }

  for(int i=0; i < reduction_candidate_nodes.size(); i++)
  {
    t[i].join();
    //std::cout << "finish try node " << reduction_candidate_nodes[i] << " :" << std::endl;
    //std::cout << reduction_candidate_cost[i] << std::endl;

    reduction_candidate_error(i) = reduction_candidate_cost[i];
    RC.push_back(Ropt[i]);

    if(reduction_candidate_error(i) < ERROR_MIN)
    {
      R_best = Ropt[i];
      ERROR_MIN = reduction_candidate_error(i);
    }
  }

  std::cout << std::endl;

  end = std::chrono::steady_clock::now();
  std::cout << "TIME: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << std::endl;

  int minindex;

  reduction_candidate_error.minCoeff(&minindex);
  reduction_order.push_back(reduction_candidate_nodes[minindex]);
  rigid_nodes(reduction_candidate_nodes[minindex]) = 0;

  // Eigen::MatrixXd ErrorColors;
  //
  // //Eigen::MatrixXd normer = (reduction_candidate_error.array() - reduction_candidate_error.minCoeff())/(reduction_candidate_error.maxCoeff() - reduction_candidate_error.minCoeff());
  //
  // Eigen::MatrixXd normer = last_reduction_error - reduction_candidate_error.array();
  //
  // igl::jet(model->I, true, ErrorColors);
  //
  // //std::cout << model->I << std::endl;
  //
  // for(int i=0; i<reduction_candidate_error.rows(); i++)
  // {
  //   std::cout << reduction_candidate_nodes[i] << ":" << reduction_candidate_error(i) << std::endl;
  //   CPoints.push_back(model->Nodes.row(reduction_candidate_nodes[i]) - model->Nodes.row(model->root_node_id));
  //   CPointsC.push_back(ErrorColors.row(reduction_candidate_nodes[i]));
  // }

  last_reduction_error = ERROR_MIN;

  std::cout << "Node " << reduction_candidate_nodes[minindex] << " is non-rigid now!" << std::endl;
  joints_left = rigid_nodes.rows() - rigid_nodes.sum();
  std::cout << rigid_nodes.rows() - rigid_nodes.sum()-1 << " joints are distributed now" << std::endl;

  std::cout << ERROR_MIN << std::endl;

  // set everything back to best simplified solution
  rigid_nodes_n = rigid_nodes;

  Reduction r;
  setup_variables(rigid_nodes_n, r);

  SEdges = r.E;
  M = r.M;

  return true;
}

double Reducer3::FindRotations(Reduction r, std::vector<Eigen::MatrixXd> &Ropt, double &cost)
{
  Eigen::MatrixXd H;
  Eigen::VectorXd g;

  Eigen::MatrixXd VMC;
  Eigen::MatrixXd VMC2;

  igl::lbs_matrix(VNodes, model->WC, VMC);
  igl::lbs_matrix(VNodes2, model->WC, VMC2);

  double total_error = 0;
  double start_error = 0;

  Ropt = r.R;

  // now loop through poses and find best rotations for each of them!
  for(int k=0; k < model->Poses.size(); k++)
  {
    double ERROR = 0;
    double ERROR_before = 0;

    Eigen::MatrixXd PN = model->MC*model->PosesN[k];
    Eigen::MatrixXd VPN = VMC*model->PosesN[k];
    Eigen::MatrixXd HPN = VMC2*model->PosesN[k];

    Eigen::VectorXd n =  Eigen::VectorXd::Constant(3*n_complex,0);
    Eigen::VectorXd vn =  Eigen::VectorXd::Constant(3*n_complex,0);
    Eigen::VectorXd hn =  Eigen::VectorXd::Constant(3*n_complex,0);

    for(int c=0; c < n_complex; c++) // add up all the complex pose position (errors)
    {
      n.block(3*c, 0, 3 , 1) = PN.row(c).transpose() - PN.row(model->root_node_id).transpose();
    }

    for(int c=0; c < n_complex; c++) // add up all the complex pose position (errors)
    {
      vn.block(3*c, 0, 3 , 1) = VPN.row(c).transpose() - PN.row(model->root_node_id).transpose();
      hn.block(3*c, 0, 3 , 1) = HPN.row(c).transpose() - PN.row(model->root_node_id).transpose();
    }

    g = Eigen::VectorXd::Constant(3*n_complex,1);

    Eigen::MatrixXd mdmsfd;

    Eigen::SparseMatrix<double> RR(3*n_complex, 3*n_complex);

    Eigen::MatrixXd PR(3*n_complex, 3);

    Eigen::MatrixXd last_factor_R(3*n_complex, 3);

    RR.setZero();

    PR = r.M*r.R[k];

    for(int o=0; o < n_complex; o++)
    {
      for(int i=0; i < 3; i++)
      {
        for(int j=0; j < 3; j++)
        {
          RR.insert(3*o+i, 3*o+j) = PR(3*o+i, j);
        }
      }
    }

    mdmsfd = (n - K*(RR)*e).transpose()*IM*(n - K*(RR)*e)
              + rl*(vn - (RR*(vstack - KV*e) + KV*RR*e)).transpose()*IM*(vn - (RR*(vstack - KV*e) + KV*RR*e))
              + rl*(hn - (RR*(vstack2 - KV*e) + KV*RR*e)).transpose()*IM*(hn - (RR*(vstack2 - KV*e) + KV*RR*e));

    ERROR = normalizef*normalizef*normalizef*normalizef*mdmsfd(0,0);

    std::cout << ERROR << std::endl;

    int max_iters = 50*(r.RN.rows() - r.RN.sum()-1);

    Eigen::VectorXd d;

    std::chrono::time_point<std::chrono::steady_clock> start,end;

    bool show_time = false;
    bool show_info = false;

    int iters = 0;

    double factor = 1;

    double last_iteration_ERROR = ERROR;

    int direction = 0;

    while(iters < max_iters)
    {
      start = std::chrono::steady_clock::now();

      iters++;

      // get new direction
      update_gradient(g, k, r, n, vn, hn);

      g = normalizef*normalizef*normalizef*normalizef*g;

      if(g.norm() < 0.001)
      {
        break;
      }

      end = std::chrono::steady_clock::now();
      if(show_time)
        std::cout << "G " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << std::endl;
      start = std::chrono::steady_clock::now();

      // Eigen::FullPivHouseholderQR<Eigen::MatrixXd> qr(g.rows(), g.rows());
      // qr.compute(ERROR*ERROR*g*g.transpose());
      //       d = qr.solve(-g);

      Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(g.rows(), g.rows());
      qr.compute(ERROR*ERROR*g*g.transpose());
            d = qr.solve(-g);

      // Eigen::MatrixXd H = ERROR*ERROR*g*g.transpose();
      // //
      // // d = H.ldlt().solve(-g);
      //
      // d = H.householderQr().solve(-g);

      double last_factor_ERROR = last_iteration_ERROR;

      Eigen::MatrixXd Jn(3,3);
      Eigen::MatrixXd Rp1;

      direction = 0;

      end = std::chrono::steady_clock::now();
      if(show_time)
        std::cout << "D " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << std::endl;

      while(true)
      {
        Rp1 = r.R[k];

        int after_root_r = 0;

        // get new ERROR

        start = std::chrono::steady_clock::now();

        if(factor*d.norm() < 0.0001)
        {
        //  std::cout << "SD" << std::endl;
          iters = max_iters;
          break;
        }

        for(int m=0; m<n_complex; m++)
        {
          if(m == model->root_node_id || r.RN(m) == 1)
          {
            after_root_r++;
          }
          else
          {
            Jn.resize(3,3);

            Jn << 0, -d(3*(m-after_root_r)+2), d(3*(m-after_root_r)+1),
                  d(3*(m-after_root_r)+2), 0, -d(3*(m-after_root_r)),
                  -d(3*(m-after_root_r)+1), d(3*(m-after_root_r)), 0;

            Jn = factor*Jn;

            Rp1.block(3*m,0,3,3) = Rp1.block(3*m,0,3,3)*Jn.exp();
          }
        }

        ERROR = 0;

        RR.setZero();

        PR = r.M*Rp1;

        for(int o=0; o < n_complex; o++)
        {
          for(int i=0; i < 3; i++)
          {
            for(int j=0; j < 3; j++)
            {
              RR.insert(3*o+i, 3*o+j) = PR(3*o+i, j);
            }
          }
        }

        end = std::chrono::steady_clock::now();
        if(show_time)
          std::cout << "PARAS " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << std::endl;
        start = std::chrono::steady_clock::now();

        mdmsfd = (n - K*RR*e).transpose()*IM*(n - K*RR*e)
                  + rl*(vn - (RR*(vstack - KV*e) + KV*RR*e)).transpose()*IM*(vn - (RR*(vstack - KV*e) + KV*RR*e))
                  + rl*(hn - (RR*(vstack2 - KV*e) + KV*RR*e)).transpose()*IM*(hn - (RR*(vstack2 - KV*e) + KV*RR*e));
        ERROR = normalizef*normalizef*normalizef*normalizef*mdmsfd(0,0);

        if(ERROR!=ERROR)
        {
          std::cout << "NAN" << std::endl;
          iters = max_iters;
          break;
        }

        end = std::chrono::steady_clock::now();
        if(show_time)
          std::cout << "ERROR " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << std::endl;

        if(show_info)
          std::cout << k << " " << iters << " " << ERROR << " | " << " : " << factor << " : " << g.norm() << std::endl;

        if(direction==0)
        {
          if(ERROR > last_iteration_ERROR)
          {
            direction = -1;
          }
          else if(ERROR < last_iteration_ERROR)
          {
            direction = 1;
            last_factor_ERROR = last_iteration_ERROR;
          }
          else
          {
            break;
          }
        }

        if(direction==-1)
        {
          if(ERROR > last_iteration_ERROR)
          {
            factor = 0.1*factor;
          }
          else
          {
            last_iteration_ERROR = ERROR;
            r.R[k] = Rp1;
            Ropt[k] = Rp1;
            break;
          }
        }

        if(direction==1)
        {
          if(ERROR < last_factor_ERROR)
          {
            factor = 10*factor;
          }
          else
          {
            last_iteration_ERROR = last_factor_ERROR;
            r.R[k] = last_factor_R;
            Ropt[k] = last_factor_R;
            factor = 0.1*factor;
            break;
          }
        }

        last_factor_ERROR = ERROR;
        last_factor_R = Rp1;
      }

      if(show_info)
        std::cout <<  "- - - - - - - - - - - - - - - - - - - - - - " << std::endl;
    }

    if(iters == max_iters)
    {
      std::cout << "N" << std::endl;
    }
    else
    {
      //std::cout << "Y: " << iters << std::endl;
    }

    total_error += last_iteration_ERROR;

    if(show_info)
      std::cout <<  "----------------------------------------------" << std::endl;
  }

  std::cout << "FINISH" << std::endl;

  // for referenced cost
  cost = total_error;

  return total_error;
}

bool Reducer3::update_gradient(Eigen::VectorXd &g, int p, Reduction r, Eigen::VectorXd& n, Eigen::VectorXd& vn, Eigen::VectorXd& hn)
{
  // std::chrono::time_point<std::chrono::steady_clock> start,end;
  // start = std::chrono::steady_clock::now();

  g = Eigen::VectorXd::Constant(3*(n_complex - r.RN.sum() - 1), 0);

  Eigen::MatrixXd A(3*n_complex,1), B(3*n_complex,1), O(3*n_complex,1), AX(3*n_complex,1), BX(3*n_complex,1), AY(3*n_complex,1), BY(3*n_complex,1), dRp(3*n_complex,3);

  Eigen::MatrixXd mdmsf;

  int after_root_s = 0;

  Eigen::SparseMatrix<double> RR(3*n_complex, 3*n_complex);
  RR.setZero();
  Eigen::MatrixXd PR(3*n_complex, 3);

  // end = std::chrono::steady_clock::now();
  // std::cout << "A " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << std::endl;
  // start = std::chrono::steady_clock::now();

  PR = r.M*r.R[p];

  // end = std::chrono::steady_clock::now();
  // std::cout << "B " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << std::endl;
  // start = std::chrono::steady_clock::now();

  for(int o=0; o < n_complex; o++)
  {
    for(int i=0; i < 3; i++)
    {
      for(int j=0; j < 3; j++)
      {
        RR.insert(3*o+i, 3*o+j) = PR(3*o+i, j);
      }
    }
  }

  // end = std::chrono::steady_clock::now();
  // std::cout << "C " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << std::endl;
  // start = std::chrono::steady_clock::now();

  A = n - K*RR*e;
  O = KV*e;
  AX = vn - (RR*(vstack - O) + KV*RR*e);
  AY = hn - (RR*(vstack2 - O) + KV*RR*e);

  // A = n - K*(RR)*e;
  // AX = (vn - (RR*(vstack - KV*e) + KV*RR*e));
  // AY = (hn - (RR*(vstack2 - KV*e) + KV*RR*e));

  after_root_s = 0;

  // end = std::chrono::steady_clock::now();
  // std::cout << "D " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << std::endl;
  // start = std::chrono::steady_clock::now();

  for(int s=0; s < n_complex; s++) // through all R (simple bones)
  {
    if(s == model->root_node_id  || r.RN(s) == 1)
    {
      after_root_s++;
      continue;
    }

    for(int i=0; i < 3; i++) // through x, y, z part of R
    {
      dRp = Eigen::MatrixXd::Constant(3*n_complex, 3, 0);
      dRp.block(3*s, 0, 3, 3) = r.R[p].block(3*s, 0, 3, 3)*J[i];

      RR.setZero();

      PR = r.M*dRp;

      for(int o=0; o < n_complex; o++)
      {
        for(int k=0; k < 3; k++)
        {
          for(int l=0; l < 3; l++)
          {
            RR.insert(3*o+k, 3*o+l) = PR(3*o+k, l);
          }
        }
      }

      // for(int o=0; o < n_complex; o++)
      //   RR.block(3*o, 3*o, 3, 3) = r.M.block(3*o, 0, 3, 3*n_complex)*dRp;

      B = -K*RR*e;
      O = KV*e;
      BX = -(RR*(vstack - O) + KV*RR*e);
      BY = -(RR*(vstack2 - O) + KV*RR*e);

      // B = -K*(RR)*e;
      // BX = -(RR*(vstack - KV*e) + KV*RR*e);
      // BY = -(RR*(vstack2 - KV*e) + KV*RR*e);

      mdmsf = 2*A.transpose()*IM*B + rl*(2*AX.transpose()*IM*BX + 2*AY.transpose()*IM*AY);

      g(3*(s-after_root_s)+i) = mdmsf(0,0);
    }
  }

  // end = std::chrono::steady_clock::now();
  // std::cout <<  "E " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << std::endl;

  return true;
}

bool Reducer3::update_hessian(Eigen::MatrixXd &H, int p, Reduction r, Eigen::VectorXd& n, Eigen::VectorXd& vn, Eigen::VectorXd& hn)
{
  H = Eigen::MatrixXd::Constant(3*(n_complex - r.RN.sum() - 1), 3*(n_complex - r.RN.sum() - 1), 0);

  Eigen::MatrixXd A(3*n_complex,1), B1(3*n_complex,1), B2(3*n_complex,1), AX(3*n_complex,1), B1X(3*n_complex,1), B2X(3*n_complex,1), AY(3*n_complex,1), B1Y(3*n_complex,1), B2Y(3*n_complex,1),  C(3*n_complex,1),CX(3*n_complex,1), CY(3*n_complex,1),  dRp(3*n_complex, 3), ddRp(3*n_complex, 3);

  Eigen::MatrixXd mdmsf;

  Eigen::MatrixXd HD = Eigen::MatrixXd::Constant(H.rows(),H.cols(),0);

  int after_root_s = 0;
  int after_root_t = 0;

  Eigen::SparseMatrix<double> RR(3*n_complex, 3*n_complex);
  RR.setZero();
  Eigen::MatrixXd PR(3*n_complex, 3);

  PR = r.M*r.R[p];

  for(int o=0; o < n_complex; o++)
  {
    for(int i=0; i < 3; i++)
    {
      for(int j=0; j < 3; j++)
      {
        RR.insert(3*o+i, 3*o+j) = PR(3*o+i, j);
      }
    }
  }

  A = n - K*(RR)*e;
  AX = (vn - (RR*(vstack - KV*e) + KV*RR*e));
  AY = (hn - (RR*(vstack2 - KV*e) + KV*RR*e));

  after_root_s = 0;

  for(int s=0; s < n_complex; s++) // through all R (simple pones)
  {
    if(s == model->root_node_id  || rigid_nodes_n(s) == 1)
    {
      after_root_s++;
      continue;
    }

    for(int i=0; i < 3; i++) // through x, y, z part of R
    {
      dRp = Eigen::MatrixXd::Constant(3*n_complex, 3, 0);
      dRp.block(3*s, 0, 3, 3) = r.R[p].block(3*s, 0, 3, 3)*J[i];

      // RR = Eigen::MatrixXd::Constant(3*n_complex, 3*n_complex, 0);
      //
      // for(int o=0; o < n_complex; o++)
      //   RR.block(3*o, 3*o, 3, 3) = M.block(3*o, 0, 3, 3*n_complex)*dRp;

      RR.setZero();

      PR = r.M*dRp;

      for(int o=0; o < n_complex; o++)
      {
        for(int k=0; k < 3; k++)
        {
          for(int l=0; l < 3; l++)
          {
            RR.insert(3*o+k, 3*o+l) = PR(3*o+k, l);
          }
        }
      }

      B1 = -K*(RR)*e;
      B1X = -(RR*(vstack - KV*e) + KV*RR*e);
      B1Y = -(RR*(vstack2 - KV*e) + KV*RR*e);

      after_root_t = 0;

      for(int t=0; t <= s /* n_simple*/; t++) // through all R (simple pones)
      {
        if(t == model->root_node_id  || rigid_nodes_n(t) == 1)
        {
          after_root_t++;
          continue;
        }

        for(int j=0; j <= i /* 3*/; j++) // through x, y, z part of R
        {
          dRp = Eigen::MatrixXd::Constant(3*n_complex,3, 0);
          dRp.block(3*t, 0, 3, 3) = r.R[p].block(3*t, 0, 3, 3)*J[j];

          // RR = Eigen::MatrixXd::Constant(3*n_complex, 3*n_complex, 0);
          //
          // for(int o=0; o < n_complex; o++)
          //   RR.block(3*o, 3*o, 3, 3) = M.block(3*o, 0, 3, 3*n_complex)*dRp;

          RR.setZero();

          PR = r.M*dRp;

          for(int o=0; o < n_complex; o++)
          {
            for(int k=0; k < 3; k++)
            {
              for(int l=0; l < 3; l++)
              {
                RR.insert(3*o+k, 3*o+l) = PR(3*o+k, l);
              }
            }
          }

          B2 = -K*(RR)*e;
          B2X = -(RR*(vstack - KV*e) + KV*RR*e);
          B2Y = -(RR*(vstack2 - KV*e) + KV*RR*e);

          if(s==t) // it was not true;
          {
            ddRp = Eigen::MatrixXd::Constant(3*n_complex, 3 ,0);
            ddRp.block(3*s, 0, 3, 3) = r.R[p].block(3*s, 0, 3, 3)*(0.5*(J[i]*J[j] + J[j]*J[i])); // + (1.0/3)*(J[i]*J[j]*J[i]) + (1.0/6)*(J[i]*J[i]*J[j]+J[j]*J[i]*J[i]));

            // RR = Eigen::MatrixXd::Constant(3*n_complex, 3*n_complex, 0);
            //
            // for(int o=0; o < n_complex; o++)
            //   RR.block(3*o, 3*o, 3, 3) = M.block(3*o, 0, 3, 3*n_complex)*ddRp;

            RR.setZero();

            PR = r.M*ddRp;

            for(int o=0; o < n_complex; o++)
            {
              for(int k=0; k < 3; k++)
              {
                for(int l=0; l < 3; l++)
                {
                  RR.insert(3*o+k, 3*o+l) = PR(3*o+k, l);
                }
              }
            }

            C = -K*(RR)*e;
            CX = -(RR*(vstack - KV*e) + KV*RR*e);
            CY = -(RR*(vstack2 - KV*e) + KV*RR*e);

            mdmsf =  2*B1.transpose()*IM*B2 + 2*A.transpose()*IM*C +
                     rl*(2*B1X.transpose()*IM*B2X + 2*AX.transpose()*IM*CX +
                     2*B1Y.transpose()*IM*B2Y + 2*AY.transpose()*IM*CY); //-2*(A.transpose()*IM*C - B1.transpose()*IM*B2);
          }
          else
          {
            mdmsf =  2*B1.transpose()*IM*B2 + rl*(2*B1X.transpose()*IM*B2X + 2*B1Y.transpose()*IM*B2Y);
          }

          if(s==t && i==j)
            HD(3*(s-after_root_s)+i, 3*(t-after_root_t)+j) = 0.5*mdmsf(0,0);
          else
            HD(3*(s-after_root_s)+i, 3*(t-after_root_t)+j) = mdmsf(0,0);
        }
      }
    }
  }

  H = HD + HD.transpose(); // + 10*Eigen::MatrixXd::Identity(HD.rows(), HD.cols());

  //std::cout << H << std::endl;

  return true;
}
