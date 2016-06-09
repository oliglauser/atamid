#include "draw_things.h"

void draw_puppet(std::vector<Eigen::MatrixXd> &Vs, std::vector<Eigen::MatrixXi> &Fs, std::vector<Eigen::MatrixXd> &Cs, Eigen::MatrixXi &Edges, int rootID, Eigen::VectorXi &SplitterMap, std::vector<Eigen::MatrixXd> &Splitters, Eigen::MatrixXd &Cos, Eigen::MatrixXd &Restposes, Eigen::MatrixXd &JC)
{
  using namespace Eigen;

  Eigen::MatrixXd TUTCOLORS(5,3);

  TUTCOLORS << 1,0,0,
               1,1,0,
               0,1,0,
               0,1,1,
               0,0,1;

  std::vector<Eigen::MatrixXd> CR;
  CR.push_back(Eigen::RowVector3d(0.8,0,0));
  std::vector<Eigen::MatrixXd> CB;
  CB.push_back(Eigen::RowVector3d(0,0,0.8));
  std::vector<Eigen::MatrixXd> CG;
  CG.push_back(Eigen::RowVector3d(0,0.8,0));

  std::vector<std::vector<Eigen::MatrixXd> > CCC;
  CCC.push_back(CR);
  CCC.push_back(CG);
  CCC.push_back(CB);

  Eigen::MatrixXd SCOLORS(3,3);

  SCOLORS <<  0.3,0.3,0.3,
              0.3,0.3,0.3,
              0.3,0.3,0.3;

  std::vector<Eigen::MatrixXd> Vo_S;
  std::vector<Eigen::MatrixXi> Fo_S;
  std::vector<Eigen::MatrixXd> Co_S;

  std::vector<Eigen::MatrixXd> Vo_TUT;
  std::vector<Eigen::MatrixXi> Fo_TUT;
  std::vector<Eigen::MatrixXd> Co_TUT;

  load_TUT(Vo_TUT, Fo_TUT, Co_TUT, "../STL", TUTCOLORS);
  load_S(Vo_S, Fo_S, Co_S, "../STL", SCOLORS);

  Eigen::VectorXi pushed = Eigen::VectorXi::Constant(SplitterMap.rows(), 0);
  Eigen::MatrixXd S = Eigen::MatrixXd::Constant(SplitterMap.rows(), 3, 0);

  std::function<void (Eigen::Vector2i, int) > push_joint = [&] (Eigen::Vector2i E, int color)
  {
    if(pushed(E(1))==0)
    {
      Eigen::MatrixXd Vi;
      Eigen::MatrixXi Fi;
      Eigen::MatrixXd Ci;

      Eigen::Matrix3d r = Cos.block(3*E(1), 0, 3, 3).transpose()*Restposes.block(3*E(1), 0, 3, 3)*Cos.block(3*E(1), 0, 3, 3);

      // TODO : to this here correct .. but I think the rest is correct :)
      Eigen::MatrixXd rT = r;

      std::vector<Eigen::MatrixXd> SHITC;
      SHITC.push_back(JC.row(E(1)));

      draw_TUT(Vi, Fi, Ci, Vo_TUT, Fo_TUT, SHITC, atan2(rT(1,0), -rT(2,0)), 0, atan2(sqrt(rT(0,1)*rT(0,1)+rT(0,2)*rT(0,2)), rT(0,0)), atan2(rT(0,1), rT(0,2)));

      Eigen::MatrixXd Vc = Vi*Cos.block(3*E(1), 0, 3, 3).transpose(); //*Restposes.block(3*E(1), 0, 3, 3).transpose();

      if(pushed(E(0)) == 0 && E(0) != rootID)
      {
        for(int j=0; j < Edges.rows(); j++)
        {
          if(Edges(j,1) == E(0))
          {
            push_joint(Edges.row(j), (color+1)%2);
            break;
          }
        }
      }

      if(E(0) != rootID)
        Vc = Vc.rowwise() + S.row(E(0));

      if( SplitterMap( E(0) ) != -1)
      {
        Vc = Vc.rowwise() + (Cos.block(3*E(1), 0, 3, 3)*Eigen::Vector3d(19,0,0)).transpose();

        S.row(E(1)) = S.row(E(0)) + (Cos.block(3*E(1), 0, 3, 3)*(Eigen::Vector3d(19,0,0) + Eigen::Vector3d(34.6,0,0) + r*Eigen::Vector3d(31.6,0,0))).transpose();
      }
      else
      {
        S.row(E(1)) = S.row(E(0)) + (Cos.block(3*E(1), 0, 3, 3)*(Eigen::Vector3d(34.6,0,0) + r*Eigen::Vector3d(31.6,0,0))).transpose();
      }

      Vs.push_back(Vc);
      Fs.push_back(Fi);
      Cs.push_back(Ci);

      // is it a splitter
      if( SplitterMap( E(1) ) != -1)
      {
        //std::cout << "splitter detected" << std::endl;

        draw_S(Vi, Fi, Ci, Vo_S, Fo_S, Co_S, Splitters[SplitterMap( E(1) ) ]);

        Eigen::MatrixXd Vd = Vi*Cos.block(3*E(1), 0, 3, 3).transpose()*Restposes.block(3*E(1), 0, 3, 3).transpose();//Vi*r*Cos.block(3*E(1), 0, 3, 3).transpose();

        S.row(E(1)) += (Cos.block(3*E(1), 0, 3, 3)*r*Eigen::Vector3d(21.5, 0,0)).transpose();

        Vd = Vd.rowwise() + S.row(E(1));

        Vs.push_back(Vd);
        Fs.push_back(Fi);
        Cs.push_back(Ci);
      }

      pushed(E(1)) = 1;
    }
  };

  if(SplitterMap(rootID) != -1)
  {
    Eigen::MatrixXd Vi;
    Eigen::MatrixXi Fi;
    Eigen::MatrixXd Ci;

    draw_S(Vi, Fi, Ci, Vo_S, Fo_S, Co_S, Splitters[SplitterMap( rootID ) ]);

    Vi = Vi*Cos.block(3*rootID, 0, 3, 3).transpose();

    Vs.push_back(Vi);
    Fs.push_back(Fi);
    Cs.push_back(Ci);
  }

  for(int i=0; i < Edges.rows(); i++)
  {
      push_joint(Edges.row(i), 0);
  }

  // we get the coordinate system
  // we get the rest pose
  // we need branch out the splitters

  //draw_S(Vi, Fi, Ci, Vo_So, Fo_S, Co_S, Eigen::MatrixXd& Outs);
  // V_combo.push_back(Vi);
  // F_combo.push_back(Fi);
  // C_combo.push_back(Ci);
}

void load_S(std::vector<Eigen::MatrixXd> &Vs, std::vector<Eigen::MatrixXi> &Fs, std::vector<Eigen::MatrixXd> &Cs, std::string path, Eigen::MatrixXd &C)
{
  using namespace Eigen;

  Eigen::MatrixXd V,N;
  Eigen::MatrixXi F;

  // Eigen::Matrix3d PREROT;
  // PREROT = AngleAxisd(-M_PI/2, Vector3d::UnitX())*AngleAxisd(-M_PI/2, Vector3d::UnitY());

  igl::readSTL(path + "/s_splitter_in_II.stl",V,F,N);
  // V.rowwise() += 19.5606*Vector3d::UnitZ().transpose();
  // V.rowwise() += 2.6340*Vector3d::UnitY().transpose();
  V.rowwise() -= 21.7*Vector3d::UnitX().transpose();
  //V = V*PREROT.transpose();
  Vs.push_back(V);
  Fs.push_back(F);

  igl::readSTL(path + "/s_splitter_body.stl",V,F,N);
  Vs.push_back(V);
  Fs.push_back(F);

  igl::readSTL(path + "/s_splitter_out_II.stl",V,F,N);
  // V.rowwise() += 19.5606*Vector3d::UnitZ().transpose();
  // V.rowwise() += 2.6340*Vector3d::UnitY().transpose();
  V.rowwise() += 18.8*Vector3d::UnitX().transpose();
  //V = V*PREROT.transpose();
  Vs.push_back(V);
  Fs.push_back(F);

  if(C.rows()==1)
  {
    for(int i=0; i<3; i++)
    {
      Cs.push_back(C);
    }
  }
  else if(C.rows()==3)
  {
    for(int i=0; i<3; i++)
    {
      Cs.push_back(C.row(i));
    }
  }
  else
  {
    Eigen::MatrixXd grey(1,3);
    grey << 0.5, 0.5, 0.5;

    for(int i=0; i<3; i++)
    {
      Cs.push_back(grey);
    }
  }
}

void draw_S(Eigen::MatrixXd& V, Eigen::MatrixXi& F, Eigen::MatrixXd& C, std::vector<Eigen::MatrixXd> &Vo, std::vector<Eigen::MatrixXi> &Fo, std::vector<Eigen::MatrixXd> &Co, Eigen::MatrixXd& Outs)
{
  using namespace Eigen;

  // create a copy of the original parts
  std::vector<Eigen::MatrixXd> Vs;
  std::vector<Eigen::MatrixXi> Fs;
  std::vector<Eigen::MatrixXd> Cs;

  // in and body don't need to be rotated
  Vs.push_back(Vo.at(0));
  Vs.push_back(Vo.at(1));

  Fs.push_back(Fo.at(0));
  Fs.push_back(Fo.at(1));

  Cs.push_back(Co.at(0));
  Cs.push_back(Co.at(1));

  Eigen::Matrix3d dRL;

  for(int i=0; i<Outs.rows(); i++)
  {
    dRL = AngleAxisd(Outs(i,2), Vector3d::UnitZ())*
        AngleAxisd(Outs(i,1), Vector3d::UnitY())*
        AngleAxisd(Outs(i,0), Vector3d::UnitX());

    Vs.push_back(Vo.at(2)*dRL.transpose());
    Fs.push_back(Fo.at(2));
    Cs.push_back(Co.at(2));
  }

  //std::cout << "ready to merge" << std::endl;

  merge_meshes(V,F,C,Vs,Fs,Cs);
}

void load_TUT(std::vector<Eigen::MatrixXd> &Vs, std::vector<Eigen::MatrixXi> &Fs, std::vector<Eigen::MatrixXd> &Cs, std::string path, Eigen::MatrixXd &C)
{
  using namespace Eigen;

  Eigen::MatrixXd V,N;
  Eigen::MatrixXi F;

  igl::readSTL(path + "/tut_joint_one_II.stl",V,F,N);
  Vs.push_back(V);
  Fs.push_back(F);

  igl::readSTL(path + "/tut_joint_two_II.stl",V,F,N);
  Vs.push_back(V);
  Fs.push_back(F);

  igl::readSTL(path + "/tut_joint_three_II.stl",V,F,N);
  Vs.push_back(V);
  Fs.push_back(F);

  igl::readSTL(path + "/tut_joint_four_II.stl",V,F,N);
  Vs.push_back(V);
  Fs.push_back(F);

  igl::readSTL(path + "/tut_joint_five_II.stl",V,F,N);
  Vs.push_back(V);
  Fs.push_back(F);

  if(C.rows()==1)
  {
    for(int i=0; i<5; i++)
    {
      Cs.push_back(C);
    }
  }
  else if(C.rows()==5)
  {
    for(int i=0; i<5; i++)
    {
      Cs.push_back(C.row(i));
    }
  }
  else
  {
    Eigen::MatrixXd grey(3,1);
    grey << 0.5, 0.5, 0.5;

    for(int i=0; i<5; i++)
    {
      Cs.push_back(grey);
    }
  }
}

void draw_TUT(Eigen::MatrixXd& V, Eigen::MatrixXi& F, Eigen::MatrixXd& C, std::vector<Eigen::MatrixXd> &Vo, std::vector<Eigen::MatrixXi> &Fo, std::vector<Eigen::MatrixXd> &Co, float angle1, float angle2, float angle3, float angle4)
{
  using namespace Eigen;
  // that's a mess
  // yes :: TODO: make better and then it can be reused :D

  // create a copy of the original parts
  std::vector<Eigen::MatrixXd> Vs;
  std::vector<Eigen::MatrixXd> Cs;

  Vs.push_back(Vo.at(0));
  Vs.push_back(Vo.at(1));
  Vs.push_back(Vo.at(2));
  Vs.push_back(Vo.at(3));
  Vs.push_back(Vo.at(4));

  // move them al to zero to be correctly rotated afterwards
  Vs.at(2).rowwise() -= 34.6*Vector3d::UnitX().transpose();
  Vs.at(3).rowwise() -= 34.6*Vector3d::UnitX().transpose();
  Vs.at(4).rowwise() -= 34.6*Vector3d::UnitX().transpose();

  Eigen::Matrix3d m,n,o,p;

  // rotate first part
  m = AngleAxisd(angle1, Vector3d::UnitX());
  Vs.at(1) = Vs.at(1)*m.transpose();

  // rotate ball
  n = AngleAxisd(angle3, Vector3d::UnitY());
  Vs.at(2) = Vs.at(2)*n.transpose()*m.transpose();

  // rotate last two parts
  o = AngleAxisd(angle2, Vector3d::UnitZ());
  Vs.at(3) = Vs.at(3)*o.transpose()*n.transpose()*m.transpose();

  // rotate last part
  p = AngleAxisd(angle4, Vector3d::UnitX());
  Vs.at(4) = Vs.at(4)*p.transpose()*o.transpose()*n.transpose()*m.transpose();

  // move parts
  Vs.at(2).rowwise() += 34.6*Vector3d::UnitX().transpose();
  Vs.at(3).rowwise() += 34.6*Vector3d::UnitX().transpose();
  Vs.at(4).rowwise() += 34.6*Vector3d::UnitX().transpose();

  if(Co.size()!=5)
  {
    for(int i=0; i<5; i++)
      Cs.push_back(Co[0]);
  }
  else
  {
    Cs = Co;
  }

  merge_meshes(V,F,C,Vs,Fo,Cs);
}

void draw_skeleton(Eigen::MatrixXd& PO, Eigen::MatrixXi& EO, Eigen::MatrixXd& CO, Eigen::MatrixXd& P, Eigen::MatrixXi& BE, Eigen::MatrixXd& C, float size)
{
  using namespace Eigen;

  //std::cout << "what?" << std::endl;

  std::vector<Eigen::MatrixXd> Ps;
  std::vector<Eigen::MatrixXi> Es;
  std::vector<Eigen::MatrixXd> CEs;

  Eigen::MatrixXd CC;

  // create pyramid
  for(int e=0; e < BE.rows(); e++)
  {
    Eigen::MatrixXd s = P.row(BE(e,0));
    Eigen::MatrixXd d = P.row(BE(e,1));
    Eigen::MatrixXd b = (d-s).transpose().eval();

    Quaterniond q;
    q.setFromTwoVectors(Vector3d(0,0,1),b);

    MatrixXd rot_mat = q.toRotationMatrix();

    Eigen::MatrixXd PI, CI;
    Eigen::MatrixXi EI;

    //std::cout << "bone " << e << std::endl;
    if(C.rows()==1)
    {
      draw_bone(PI, EI, CI, s, rot_mat, C, b.norm(), size);
    }
    else
    {
      CC = C.row(e);
      draw_bone(PI, EI, CI, s, rot_mat, CC, b.norm(), size);
    }

    Ps.push_back(PI);
    Es.push_back(EI);
    CEs.push_back(CI);
  }

  merge_edges(PO, EO, CO, Ps, Es, CEs);
}

void draw_bone(Eigen::MatrixXd& PO, Eigen::MatrixXi& EO, Eigen::MatrixXd& CO, Eigen::MatrixXd& O, Eigen::MatrixXd& RPY, Eigen::MatrixXd& CC, double length, float size)
{
  using namespace Eigen;
  std::vector<Eigen::MatrixXd> Ps;
  std::vector<Eigen::MatrixXi> Es;
  std::vector<Eigen::MatrixXd> CEs;

  // the sphere
  int circle_segments = 20;

  Eigen::MatrixXd P(circle_segments,3);
  Eigen::MatrixXi E(circle_segments,2);
  Eigen::MatrixXd C(circle_segments,3);

  for(int i=0; i < circle_segments; i++)
  {
      P(i,0) = sin(i*2.0*M_PI/circle_segments);
      P(i,1) = cos(i*2.0*M_PI/circle_segments);
      P(i,2) = 0;

      E(i,0) = i;
      E(i,1) = i+1;
      C.row(i) = CC;
  }

  E(circle_segments-1,1) = 0;

  P = size*P;

  Eigen::Matrix3d a;
  a = AngleAxisd(M_PI/2, Vector3d::UnitX());

  Eigen::MatrixXd P4  = P*a.transpose();

  for(int i=0; i<P4.rows(); i++)
      P4.row(i) += O;

  Ps.push_back(P4);
  Es.push_back(E);
  CEs.push_back(CC);

  Eigen::Matrix3d b;
  b = AngleAxisd(M_PI/2, Vector3d::UnitY());

  Eigen::MatrixXd P3 = P*b.transpose();

  for(int i=0; i<P3.rows(); i++)
      P3.row(i) += O;

  Ps.push_back(P3);
  Es.push_back(E);
  CEs.push_back(CC);

  for(int i=0; i<P.rows(); i++)
      P.row(i) += O;

  Ps.push_back(P);
  Es.push_back(E);
  CEs.push_back(CC);

  // the pyramid
  Eigen::MatrixXd P1(5,3);
  Eigen::MatrixXi P2(8,2);

  P1 << 1,1,0,
        -1,1,0,
        -1,-1,0,
        1,-1,0,
        0,0,1;

  P2 << 0,1,
        1,2,
        2,3,
        3,0,
        0,4,
        1,4,
        2,4,
        3,4;

  // bone lenght z
  P1.col(2) = length*P1.col(2);

  // bone width x & y
  P1.col(0) = 0.5*size*P1.col(0);
  P1.col(1) = 0.5*size*P1.col(1);

  if(RPY.rows()==3 && RPY.cols()==3)
  {
    b = RPY;
  }
  else
  {
    b = AngleAxisd(RPY(0,0), Vector3d::UnitX())*
        AngleAxisd(RPY(0,1), Vector3d::UnitY())*
        AngleAxisd(RPY(0,2), Vector3d::UnitZ());
  }

  P1 = P1*b.transpose();

  for(int i=0; i<P1.rows(); i++)
      P1.row(i) += O;

  Ps.push_back(P1);
  Es.push_back(P2);
  CEs.push_back(CC);

  // the merge
  merge_edges(PO, EO, CO, Ps, Es, CEs);
}
