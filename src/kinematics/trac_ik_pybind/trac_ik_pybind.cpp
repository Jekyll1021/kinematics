#include <trac_ik/trac_ik.hpp>
#include <urdf/model.h>
#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>
#include <limits>
#include <tf_conversions/tf_kdl.h>
#include <moveit/py_bindings_tools/roscpp_initializer.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>

namespace py = pybind11;

class TracIKWrapper : public moveit::py_bindings_tools::ROScppInitializer
{
public:
	TRAC_IK::TRAC_IK* newX;

    TracIKWrapper(const std::string& base_link, const std::string& tip_link, const std::string& urdf_string,
      double timeout, double epsilon, const std::string& solve_type="Speed")
    : moveit::py_bindings_tools::ROScppInitializer() 
    {
      urdf::Model robot_model;

      robot_model.initString(urdf_string);

      ROS_DEBUG_STREAM_NAMED("trac_ik","Reading joints and links from URDF");

      KDL::Tree tree;

      if (!kdl_parser::treeFromUrdfModel(robot_model, tree)) {
        ROS_FATAL("Failed to extract kdl tree from xml robot description");
      }


      KDL::Chain chain;

      if(!tree.getChain(base_link, tip_link, chain)) {
        ROS_FATAL("Couldn't find chain %s to %s",base_link.c_str(),tip_link.c_str());
      }

      uint num_joints_;
      num_joints_ = chain.getNrOfJoints();
      
      std::vector<KDL::Segment> chain_segs = chain.segments;

      urdf::JointConstSharedPtr joint;

      std::vector<double> l_bounds, u_bounds;

      KDL::JntArray joint_min, joint_max;

      joint_min.resize(num_joints_);
      joint_max.resize(num_joints_);

      std::vector<std::string> link_names_;
      std::vector<std::string> joint_names_;

      uint joint_num=0;
      for(unsigned int i = 0; i < chain_segs.size(); ++i) {

        link_names_.push_back(chain_segs[i].getName());
        joint = robot_model.getJoint(chain_segs[i].getJoint().getName());
        if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
          joint_num++;
          assert(joint_num<=num_joints_);
          float lower, upper;
          int hasLimits;
          joint_names_.push_back(joint->name);
          if ( joint->type != urdf::Joint::CONTINUOUS ) {
            if(joint->safety) {
              lower = std::max(joint->limits->lower, joint->safety->soft_lower_limit);
              upper = std::min(joint->limits->upper, joint->safety->soft_upper_limit);
            } else {
              lower = joint->limits->lower;
              upper = joint->limits->upper;
            }
            hasLimits = 1;
          }
          else {
            hasLimits = 0;
          }
          if(hasLimits) {
            joint_min(joint_num-1)=lower;
            joint_max(joint_num-1)=upper;
          }
          else {
            joint_min(joint_num-1)=std::numeric_limits<float>::lowest();
            joint_max(joint_num-1)=std::numeric_limits<float>::max();
          }
          ROS_DEBUG_STREAM("IK Using joint "<<chain_segs[i].getName()<<" "<<joint_min(joint_num-1)<<" "<<joint_max(joint_num-1));
        }
      }


      TRAC_IK::SolveType solvetype;

      if (solve_type == "Manipulation1")
        solvetype = TRAC_IK::Manip1;
      else if (solve_type == "Manipulation2")
        solvetype = TRAC_IK::Manip2;
      else if (solve_type == "Distance")
        solvetype = TRAC_IK::Distance;
      else {
          if (solve_type != "Speed") {
              ROS_WARN_STREAM_NAMED("trac_ik", solve_type << " is not a valid solve_type; setting to default: Speed");
          }
          solvetype = TRAC_IK::Speed;
      }
          newX = new TRAC_IK::TRAC_IK(chain, joint_min, joint_max, timeout, epsilon, solvetype);
	}

	std::vector<double> CartToJnt(const std::vector<double> q_init,
     const double x, const double y, const double z, 
     const double rx, const double ry, const double rz, const double rw, 
     // bounds x y z
     const double boundx=0.0, const double boundy=0.0, const double boundz=0.0,
     // bounds on rotation x y z
     const double boundrx=0.0, const double boundry=0.0, const double boundrz=0.0)
    {

      KDL::Frame frame;
      geometry_msgs::Pose pose;
      pose.position.x = x;
      pose.position.y = y;
      pose.position.z = z;
      pose.orientation.x = rx;
      pose.orientation.y = ry;
      pose.orientation.z = rz;
      pose.orientation.w = rw;

      tf::poseMsgToKDL(pose, frame);

      KDL::JntArray in(q_init.size()), out(q_init.size());

      for (uint z=0; z < q_init.size(); z++)
          in(z) = q_init[z];

      KDL::Twist bounds = KDL::Twist::Zero();
      bounds.vel.x(boundx);
      bounds.vel.y(boundy);
      bounds.vel.z(boundz);
      bounds.rot.x(boundrx);
      bounds.rot.y(boundry);
      bounds.rot.z(boundrz);

      int rc = newX->CartToJnt(in, frame, out, bounds);
      std::vector<double> vout;
      // If no solution, return empty vector which acts as None
      if (rc == -3)
          return vout;

      for (uint z=0; z < q_init.size(); z++)
          vout.push_back(out(z));

      return vout;
    }

    int getNrOfJointsInChain(){
      KDL::Chain chain;
      newX->getKDLChain(chain);
      return (int) chain.getNrOfJoints();
    }

    std::vector<std::string> getJointNamesInChain(const std::string& urdf_string){
      KDL::Chain chain;
      newX->getKDLChain(chain);
      std::vector<KDL::Segment> chain_segs = chain.segments;

      std::vector<std::string> joint_names_;
      std::vector<std::string> link_names_;
      urdf::JointConstSharedPtr joint;

      urdf::Model robot_model;
      robot_model.initString(urdf_string);

      for(unsigned int i = 0; i < chain_segs.size(); ++i) {
        link_names_.push_back(chain_segs[i].getName());
        joint = robot_model.getJoint(chain_segs[i].getJoint().getName());
        if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
          joint_names_.push_back(joint->name);
        }
      }
      return joint_names_;
    }

    std::vector<double> getLowerBoundLimits(){
      KDL::JntArray lb_;
      KDL::JntArray ub_;
      std::vector<double> lb;
      newX->getKDLLimits(lb_, ub_);
      for(unsigned int i=0; i < lb_.rows(); i++){
        lb.push_back(lb_(i));
      }
      return lb;
    }

    std::vector<double> getUpperBoundLimits(){
      KDL::JntArray lb_;
      KDL::JntArray ub_;
      std::vector<double> ub;
      newX->getKDLLimits(lb_, ub_);
      for(unsigned int i=0; i < ub_.rows(); i++){
        ub.push_back(ub_(i));
      }
      return ub;
    }

    void setKDLLimits(const std::vector<double> lb, const std::vector<double> ub) {
      KDL::JntArray lb_;
      KDL::JntArray ub_;
      lb_.resize(lb.size());
      for(unsigned int i=0; i < lb.size(); i++){
        lb_(i) = lb[i];
      }
      ub_.resize(ub.size());
      for(unsigned int i=0; i < ub.size(); i++){
        ub_(i) = ub[i];
      }
      newX->setKDLLimits(lb_, ub_);
    }
};

PYBIND11_MODULE(trac_ik_pybind, m) {
    py::class_<TracIKWrapper> trac_ik(m, "TracIKWrapper");
    trac_ik.def(py::init<const std::string&, const std::string&, const std::string&, double, double, const std::string&>(),
        py::arg("base_link"), py::arg("tip_link"), py::arg("urdf_string"), 
        py::arg("timeout"), py::arg("epsilon"), py::arg("solve_type")="Speed");
    trac_ik.def("CartToJnt", (std::vector<double> (TracIKWrapper::*)(const std::vector<double>,
     const double, const double, const double, 
     const double, const double, const double, const double, 
     // bounds x y z
     const double, const double, const double,
     // bounds on rotation x y z
     const double, const double, const double)) &TracIKWrapper::CartToJnt, 
     py::arg("q_init"), py::arg("x"), py::arg("y"), py::arg("z"), 
     py::arg("rx"), py::arg("ry"), py::arg("rz"), py::arg("rw"),
     py::arg("boundx")=0.0, py::arg("boundy")=0.0, py::arg("boundz")=0.0,
     py::arg("boundrx")=0.0, py::arg("boundry")=0.0, py::arg("boundrz")=0.0);
    trac_ik.def("getNrOfJointsInChain", &TracIKWrapper::getNrOfJointsInChain);
    trac_ik.def("getJointNamesInChain", &TracIKWrapper::getJointNamesInChain);
    trac_ik.def("getLowerBoundLimits", &TracIKWrapper::getLowerBoundLimits);
    trac_ik.def("getUpperBoundLimits", &TracIKWrapper::getUpperBoundLimits);
    trac_ik.def("setKDLLimits", &TracIKWrapper::setKDLLimits);
}