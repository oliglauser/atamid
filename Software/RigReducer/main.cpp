#define ENABLE_SERIALIZATION
#define IGL_VIEWER_WITH_NANOGUI

#include <igl/PI.h>
#include <igl/viewer/Viewer.h>
#include <igl/writeOBJ.h>
#include <igl/writeDMAT.h>
#include <igl/readDMAT.h>
#include <igl/jet.h>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>
#include <iostream>

#include "reduceModel.h"
#include "reducer3.h"

#include "MeshMerger.h"
#include "draw_things.h"

#include <igl/serialize.h>

ReduceModel m;
Reducer3 *pr;

// containers for line drawing for skeletons
Eigen::MatrixXd PO(0,3), CO(0,3);
Eigen::MatrixXi EO(0,2);

Eigen::MatrixXd colors(12,3);

int current_pose = 0;

// library setup
int n_TUT = 8;
int n_S5 = 1;
int n_SY = 1;
int n_SH = 1;

igl::viewer::Viewer viewer;

bool show_original_skeleton = true;
bool show_simplified_skeleton = true;
bool show_reconstructed_skeleton = true;

bool show_device = true;
bool show_mesh = true;
bool show_splitter = true;

// ---------------------------------------------


bool pre_close(igl::viewer::Viewer & viewer)
{
  return true;
}

bool pre_draw(igl::viewer::Viewer & viewer)
{
  if(viewer.core.is_animating)
  {

  }

  return false;
}

void show_pose(int i)
{
  Eigen::MatrixXd WHITE(1,3);
  WHITE << 1,1,1;

  Eigen::MatrixXd GREY(1,3);
  GREY << 0.1,0.1,0.1;

  Eigen::MatrixXd CYAN(1,3);
  CYAN << 0,1,1;

  Eigen::MatrixXd RED(1,3);
  RED << 1,0,0;

  float bone_size = (1/250.0)*(m.V.colwise().maxCoeff() - m.V.colwise().minCoeff()).norm();

  if(show_device)
  {
    if(pr->PVs.size() > 0)
    {
      Eigen::MatrixXd PV, PC;
      Eigen::MatrixXi PF;

      merge_meshes(PV, PF, PC, pr->PVs, pr->PFs, pr->PCs);

      viewer.data.clear();

      viewer.data.set_mesh(0.25*bone_size*PV, PF);
      viewer.data.set_colors(PC);
    }
  }
  else
  {
      viewer.data.clear();
  }

  Eigen::MatrixXd PoseNodes;

  if(i!=-1)
    PoseNodes = m.MC*m.PosesN[i];
  else
    PoseNodes = m.Nodes;

  std::vector<Eigen::MatrixXi> EDGES;
  std::vector<Eigen::MatrixXd> POINTS;
  std::vector<Eigen::MatrixXd> COLORS;

  PO.resize(0,3);
  EO.resize(0,2);
  CO.resize(0,3);

  if(show_original_skeleton)
  {
    Eigen::MatrixXd root = PoseNodes.row(m.root_node_id);

    for(int k=0; k<m.Nodes.rows(); k++)
    {
      PoseNodes.row(k) = PoseNodes.row(k) - root;
    }

    draw_skeleton(PO, EO, CO, PoseNodes, m.Edges, GREY, bone_size);

    EDGES.push_back(EO);
    POINTS.push_back(PO);
    COLORS.push_back(CO);
  }

  if(pr->joints_left)
  {
    Eigen::MatrixXd IPN(m.Nodes.rows(),3);

    Eigen::MatrixXd RR = Eigen::MatrixXd::Constant(3*pr->n_complex, 3*pr->n_complex, 0);

    if(i!=-1)
    {
      for(int k=0; k<pr->n_complex; k++)
      {
        RR.block(3*k, 3*k, 3, 3) = pr->M.block(3*k, 0, 3, 3*pr->n_complex)*pr->R_best[i];
      }

      Eigen::VectorXd ip = pr->K*(RR)*pr->e;

      for(int k=0; k<m.Nodes.rows(); k++)
      {
        IPN.row(k) = ip.block(3*k,0,3,1).transpose();
      }
    }
    else
    {
        IPN = m.Nodes.rowwise() - m.Nodes.row(m.root_node_id);
    }

    if(show_reconstructed_skeleton)
    {
      draw_skeleton(PO, EO, CO, IPN, m.Edges, RED, bone_size);

      EDGES.push_back(EO);
      POINTS.push_back(PO);
      COLORS.push_back(CO);
    }

    //viewer.data.set_points(IPN.row(m.root_node_id),  Eigen::RowVector3d(0.5,0.5,0.5));

    if(show_simplified_skeleton)
    {
      draw_skeleton(PO, EO, CO, IPN, pr->SEdges, CYAN, bone_size);

      EDGES.push_back(EO);
      POINTS.push_back(PO);
      COLORS.push_back(CO);

      for(int k=0; k<pr->rigid_nodes.rows(); k++)
      {
        if(pr->rigid_nodes(k)==0)
        {
          viewer.data.add_points(IPN.row(k), Eigen::RowVector3d(0.3,0.3,0.3));
        }
      }
    }

    for(int i=0; i < pr->CPoints.size(); i++)
    {
      viewer.data.add_points(pr->CPoints[i], pr->CPointsC[i]);
    }

    if(show_splitter)
    {
      // splitters
      merge_edges(PO, EO, CO, pr->SPoints, pr->SLines, pr->SColors);

      EDGES.push_back(EO);
      POINTS.push_back(PO);
      COLORS.push_back(CO);
    }
  }

  if(show_original_skeleton || show_reconstructed_skeleton || show_simplified_skeleton)
    merge_edges(PO, EO, CO, POINTS, EDGES, COLORS);

  viewer.data.set_edges(PO,EO,CO);

  // if(show_mesh)
  // {
  //   // show mesh?
  //   if(i!=-1)
  //   {
  //     Eigen::MatrixXd Kr = pr->K - Eigen::MatrixXd::Identity(pr->K.rows(), pr->K.cols());
  //
  //     Eigen::MatrixXd RePose = Eigen::MatrixXd::Constant(4*(m.W.cols()), 3, 0);
  //
  //     for(int k=0; k < m.Edges.rows(); k++)
  //     {
  //       RePose.block(4*m.Edges(k,0),0,3,3) = RR.block(3*m.Edges(k,1),3*m.Edges(k,1),3,3).transpose();
  //       RePose.block(4*m.Edges(k,0)+3,0,1,3) = (Kr.block(3*m.Edges(k,1), 0, 3, 3*pr->n_complex)*RR*pr->e - RR.block(3*m.Edges(k,1),3*m.Edges(k,1),3,3)*Kr.block(3*m.Edges(k,1), 0, 3, 3*pr->n_complex)*pr->e).transpose();
  //     }
  //
  //     Eigen::MatrixXd Vz = m.V.rowwise() - m.Nodes.row(m.root_node_id);
  //
  //     Eigen::MatrixXd MR;
  //
  //     igl::lbs_matrix(Vz, m.W, MR);
  //   }
  // }
}

bool key_down(igl::viewer::Viewer &viewer, unsigned char key, int mods)
{
  static int i=-1;
  static Eigen::MatrixXd currV;

  Eigen::MatrixXd WHITE(1,3);
  WHITE << 1,1,1;

  Eigen::MatrixXd GREY(1,3);
  GREY << 0.5,0.5,0.5;

  Eigen::MatrixXd CYAN(1,3);
  CYAN << 0,1,1;

  Eigen::MatrixXd RED(1,3);
  RED << 1,0,0;

  float bone_size = (1/100.0)*(m.V.colwise().maxCoeff() - m.V.colwise().minCoeff()).norm();
  Eigen::MatrixXd PoseNodes;

  switch(key)
  {
    case 'O':
      if(i!=-1)
        igl::writeOBJ("pose.obj", currV, m.F);
      else
        igl::writeOBJ("pose.obj", m.V, m.F);
      break;
    case 'S':
      //update_color_subtrees(viewer);
      break;
    case ' ':
      if(m.Poses.size() > 0)
      {
        i = (i+1)%(m.Poses.size());

        current_pose = i;

        show_pose(i);

        std::cout << "Showing pose " << i << std::endl;
      }
      else
      {
        std::cout << "No poses avialable!"<< std::endl;
      }

      return true;
    case 'R':
      //currV = m.V;

      //viewer.data.set_mesh(m.V,m.F);

      //draw_skeleton(PO, EO, CO, m.Nodes, m.Edges, WHITE, bone_size);
      //viewer.data.set_edges(PO,EO,CO);

      show_pose(-1);

      std::cout << "Showing pose r"<< std::endl;
      return true;
    default:
      std::cout << "unknown command: " << key << std::endl;
      break;
  }
  return false;
}

bool mouse_down(igl::viewer::Viewer& viewer, int button, int modifier)
{
  return false;
}

int main(int argc, char *argv[])
{

  using namespace Eigen;
  using namespace std;
  // /Users/oliverglauser/Documents/MATLAB/rigreducer/bar/lbar.ply

  if(argc<2)
  {
    std::cout << "No model supplied!" << std::endl;

    return 0;
  }

  m.load_model(argv[1]);

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

  float bone_size = (1/100.0)*(m.V.colwise().maxCoeff() - m.V.colwise().minCoeff()).norm();

  Eigen::MatrixXd GREY(1,3);
  GREY << 0.75,0.75,0.75;

  draw_skeleton(PO, EO, CO, m.Nodes, m.Edges, GREY, bone_size);
  viewer.data.set_edges(PO,EO,CO);

  std::cout << "F: " << m.F.rows() << " " << m.F.cols() << std::endl;
  std::cout << "V: " << m.V.rows() << " " << m.V.cols() << std::endl;
  std::cout << "E: " << m.Edges.rows() << " " << m.Edges.cols() << std::endl;
  std::cout << "N: " << m.Nodes.rows() << " " << m.Nodes.cols() << std::endl;
  std::cout << "W: " << m.W.rows() << " " << m.W.cols() << std::endl;

  // std::cout << m.Nodes << std::endl;
  // std::cout << m.Edges << std::endl;

  Reducer3 r(&m);
  pr = &r;

  if(argc>2)
  {
    Eigen::MatrixXi NRJ;

    igl::readDMAT(argv[2], NRJ);

    for(int i=0; i<NRJ.rows(); i++)
      r.SetJointImportance(NRJ(i,0), 10000);

      //r.AddFlexibleNode(NRJ(i,0));
  }

  // generate splitter library

  Eigen::MatrixXd splitter;

  bool old = false;

  if(old)
  {
    Eigen::Matrix3d R;
    R <<   0, 0, 1,
           0, 1, 0,
          -1, 0, 0;

    // s splitter
    splitter.resize(5,3);

    splitter << 0, 0, 0,
                0,90,0,
                270,90,270,
                90,270,270,
                0,270,0;

    splitter = (M_PI/180.0)*splitter;

    r.splitter_library_geometry.push_back(splitter);
    r.splitter_library_n.push_back(1);

    // y splitter
    splitter.resize(2,3);

    splitter << 0, 60, 0,
                0, 300, 0;

    splitter = (M_PI/180.0)*splitter;

    r.splitter_library_geometry.push_back(splitter);
    r.splitter_library_n.push_back(1);

    // h splitter
    splitter.resize(3,3);

    splitter << 0, 0, 0,
                0, 90,330,
                15 ,270,30;

    splitter = (M_PI/180.0)*splitter;

    r.splitter_library_geometry.push_back(splitter);
    r.splitter_library_n.push_back(1);

    for(int s=0; s < r.splitter_library_geometry.size(); s++)
    {
      for(int b=0; b < r.splitter_library_geometry[s].rows(); b++)
      {
        Eigen::MatrixXd ROT = R*
                              Eigen::AngleAxisd(r.splitter_library_geometry[s](b,2), Eigen::Vector3d::UnitZ())*
                              Eigen::AngleAxisd(r.splitter_library_geometry[s](b,1), Eigen::Vector3d::UnitX())*
                              Eigen::AngleAxisd(r.splitter_library_geometry[s](b,0), Eigen::Vector3d::UnitZ())*
                              R.transpose();

        //std::cout << ROT << std::endl;

        r.splitter_library_geometry[s](b,0) = atan2(ROT(2,1), ROT(2,2));
        r.splitter_library_geometry[s](b,1) = atan2(-ROT(2,0), sqrt(ROT(2,1)*ROT(2,1)+ROT(2,2)*ROT(2,2)));
        r.splitter_library_geometry[s](b,2) = atan2(ROT(1,0), ROT(0,0));
      }
    }

  }
  else
  {
    // s splitter
    splitter.resize(5,3);

    splitter << 0, 0, 0,
                210,270,60,
                180,0,90,
                0,0,270,
                30,90,300;

    splitter = (M_PI/180.0)*splitter;

    r.splitter_library_geometry.push_back(splitter);
    r.splitter_library_n.push_back(1);

    // y splitter
    splitter.resize(2,3);

    splitter << 0, 0, 60,
                0, 0, 300;

    splitter = (M_PI/180.0)*splitter;

    r.splitter_library_geometry.push_back(splitter);
    r.splitter_library_n.push_back(1);


    // h splitter
    splitter.resize(3,3);

    splitter << 0, 0, 0,
                0,330,90,
                0,330,270;

    splitter = (M_PI/180.0)*splitter;

    r.splitter_library_geometry.push_back(splitter);
    r.splitter_library_n.push_back(1);
  }

  // r.AddFlexibleNode(7);
  // r.AddFlexibleNode(12);
  // r.AddFlexibleNode(79);
  // r.AddFlexibleNode(47);
  // r.AddFlexibleNode(52);
  // r.AddFlexibleNode(52);
  // r.AddFlexibleNode(62);
  // r.AddFlexibleNode(27);

  // Extend viewer menu
  viewer.callback_init = [&](igl::viewer::Viewer& viewer)
  {
    // Add new group
    viewer.ngui->addGroup("Reducer");

    viewer.ngui->addVariable<bool>("Original",[&](bool val) {
      show_original_skeleton = val;
      show_pose(current_pose);
    },[&]() {
      return show_original_skeleton; // get
    });

    viewer.ngui->addVariable<bool>("Reduced",[&](bool val) {
      show_simplified_skeleton = val;
      show_pose(current_pose);
    },[&]() {
      return show_simplified_skeleton; // get
    });

    viewer.ngui->addVariable<bool>("Splitter",[&](bool val) {
      show_splitter = val;
      show_pose(current_pose);
    },[&]() {
      return show_splitter; // get
    });

    viewer.ngui->addVariable<bool>("Reconstruced",[&](bool val) {
      show_reconstructed_skeleton = val;
      show_pose(current_pose);
    },[&]() {
      return show_reconstructed_skeleton; // get
    });

    viewer.ngui->addVariable<bool>("Device",[&](bool val) {
      show_device = val;
      show_pose(current_pose);
    },[&]() {
      return show_device; // get
    });

    viewer.ngui->addVariable<bool>("Mesh",[&](bool val) {
      show_mesh = val;
      show_pose(current_pose);
    },[&]() {
      return show_mesh; // get
    });

    // Add a button
    viewer.ngui->addButton("ADD NEXT",[]()
    {
      pr->ReductionStep();
      show_pose(current_pose);
    });

    // Add a button
    viewer.ngui->addButton("SPLITTER?",[&]()
    {
      pr->FindSplitters();

      //merge_edges(PO, EO, CO, pr->SPoints, pr->SLines, pr->SColors);
      //viewer.data.set_edges(PO,EO,CO);
      show_pose(current_pose);
    });

    // Add a button
    viewer.ngui->addButton("EXPORT TO MAYA",[]()
    {
      pr->ExportSetup();

      show_pose(current_pose);
    });

    viewer.ngui->addWindow(Eigen::Vector2i(240,10),"Part Library");
    viewer.ngui->addVariable("TUT joints", n_TUT);
    viewer.ngui->addVariable("5 splitters", n_S5);
    viewer.ngui->addVariable("Y splitters", n_SY);
    viewer.ngui->addVariable("H splitters", n_SH);

    viewer.ngui->addButton("REDUCE TO ...",[]()
    {
      pr->splitter_library_n[1] = n_S5;
      pr->splitter_library_n[2] = n_SY;
      pr->splitter_library_n[3] = n_SH;

      while(pr->joints_left-1 < n_TUT)
      {
        if(!pr->ReductionStep())
        {
          break;
        }
        show_pose(current_pose);
      }

      pr->FindSplitters();

      show_pose(current_pose);
    });

    viewer.ngui->addWindow(Eigen::Vector2i(240,180),"Poses");
    viewer.ngui->addButton("previous",[]()
    {
      current_pose = (current_pose + m.Poses.size() - 1)%m.Poses.size();
      show_pose(current_pose);
    });

    viewer.ngui->addButton("next",[]()
    {
      current_pose = (current_pose + 1)%m.Poses.size();
      show_pose(current_pose);
    });

    viewer.ngui->addButton("next weights",[&]()
    {
      static int v = 0;

      Eigen::MatrixXd VAC;

      Eigen::MatrixXd WWWW = m.W.col(v);

      igl::jet(WWWW,true,VAC);

      v = (v+1)%m.W.cols();

      viewer.data.set_colors(VAC);
    });

    viewer.ngui->addButton("save view", [&]()
    {
       igl::serialize(viewer.core, "viewercore.ser");

      // igl::writeDMAT("view.dmat", viewer.core.view);
      // igl::writeDMAT("proj.dmat", viewer.core.proj);
      // igl::writeDMAT("model.dmat", viewer.core.model);
      // igl::writeDMAT("trans.dmat", viewer.core.model_translation);
    });

    viewer.ngui->addButton("load view", [&]()
    {
      igl::deserialize(viewer.core, "viewercore.ser");
      // igl::readDMAT("view.dmat", viewer.core.view);
      // igl::readDMAT("proj.dmat", viewer.core.proj);
      // igl::readDMAT("model.dmat", viewer.core.model);
      // igl::readDMAT("trans.dmat", viewer.core.model_translation);
      //
      // viewer.core.draw(viewer.data, viewer.opengl, false);
    });

    // Generate menu
    viewer.screen->performLayout();

    return false;
  };

  //std::cout << m.V.block(0,0,30,3) << std::endl;

  viewer.data.set_mesh(m.V,m.F);

  // Eigen::VectorXd AV;
  //
  // ReduceModel::vertex_area(AV, m.V, m.F);
  //
  // // std::cout << AV.maxCoeff() << std::endl;
  // // std::cout << AV.minCoeff() << std::endl;
  // //
  // // std::cout << AV << std::endl;
  // //
  // // std::cout << AV.rows() << std::endl;
  //
  // for(int i=0; i < AV.rows(); i++)
  //   AV(i) = std::log(AV(i));
  //
  // Eigen::VectorXd VW = m.W.rowwise().sum();
  //
  // //std::cout << VW << std::endl;
  //
  // AV = VW.array() * AV.array();
  //
  // Eigen::MatrixXd VAC(AV.rows(),3);
  //
  // for(int r = 0;r<AV.rows();r++)
  // {
  //   igl::jet((-AV.minCoeff() +AV(r,0))/(AV.maxCoeff() -AV.minCoeff() ),VAC(r,0),VAC(r,1),VAC(r,2));
  // }

  //igl::jet(AV, AV.minCoeff(), AV.maxCoeff(), VAC);

  // std::cout << AV.maxCoeff() << std::endl;
  // std::cout << AV.minCoeff() << std::endl;

  //viewer.data.set_colors(VAC);

  viewer.callback_pre_draw = &pre_draw;
  viewer.callback_key_down = &key_down;
  viewer.callback_pre_close = &pre_close;
  viewer.core.is_animating = true;
  viewer.core.background_color = Eigen::Vector4f(1, 1, 1, 1);
  viewer.core.animation_max_fps = 30.;
  viewer.callback_mouse_down = &mouse_down;
  viewer.launch();
}
