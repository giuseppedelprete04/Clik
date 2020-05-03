#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"
#include <std_msgs/Float64.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "geometry_msgs/Point.h"
#include "boost/thread.hpp"
#include "sensor_msgs/JointState.h"
#include <Eigen/Dense>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

using namespace Eigen;
using namespace std;

namespace gazebo
{
  class ClikP : public ModelPlugin
  {

  private:
     
     //model
     physics::ModelPtr model;
     event::ConnectionPtr updateConnection;
     physics::JointPtr _joint[7];
     
     //ros
     ros::NodeHandle _nh;
     ros::Subscriber _des_pos_sub;
     geometry_msgs::Point _des_pos;
     ros::Publisher _cmd_pub[7];
     
     //KDL
     KDL::JntArray *_q_in;
     KDL::ChainFkSolverPos_recursive *_fksolver; 
     KDL::ChainJntToJacSolver *_J_solver; 
     KDL::Frame _p_out;
     KDL::Jacobian  *_J;            
     KDL::Chain _k_chain;
     KDL::Tree iiwa_tree;
     bool trovato;
     int init;


  public: 
     bool init_robot_model();
     void control_loop();
     void des_pos_cB( geometry_msgs::PointConstPtr pos) {
         _des_pos.x = pos->x;
         _des_pos.y = pos->y;
         _des_pos.z = pos->z;
         trovato = true;

     }

     void OnUpdate()  {
	  
          for (int i=0; i<7; i++)
  	      _q_in->data[i]=_joint[i]->Position(0);
	  init++;
     }



     void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
	  init=0;
          trovato = false;	  
	  model = _parent;
          _des_pos_sub = _nh.subscribe("/des_pos", 0, &ClikP::des_pos_cB, this);
          _cmd_pub[0] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint1_position_controller/command", 0);
          _cmd_pub[1] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint2_position_controller/command", 0);
          _cmd_pub[2] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint3_position_controller/command", 0);
          _cmd_pub[3] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint4_position_controller/command", 0);
          _cmd_pub[4] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint5_position_controller/command", 0);
          _cmd_pub[5] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint6_position_controller/command", 0);
          _cmd_pub[6] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint7_position_controller/command", 0);

          _joint[0] = this->model->GetJoint("lbr_iiwa_joint_1");
          _joint[1] = this->model->GetJoint("lbr_iiwa_joint_2");
          _joint[2] = this->model->GetJoint("lbr_iiwa_joint_3");
          _joint[3] = this->model->GetJoint("lbr_iiwa_joint_4");
          _joint[4] = this->model->GetJoint("lbr_iiwa_joint_5");
          _joint[5] = this->model->GetJoint("lbr_iiwa_joint_6");
          _joint[6] = this->model->GetJoint("lbr_iiwa_joint_7");

          init_robot_model();

          this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ClikP::OnUpdate, this));
          boost::thread control_loop_thread ( &ClikP::control_loop, this);
    }

  };

    bool ClikP::init_robot_model() {
  	 std::string robot_desc_string;
  	 _nh.param("robot_description", robot_desc_string, std::string());
  	 if (!kdl_parser::treeFromString(robot_desc_string, iiwa_tree)){
  	    ROS_ERROR("Failed to construct kdl tree");
  	    return false;
         }

  	std::string base_link = "lbr_iiwa_link_0";
  	std::string tip_link  = "lbr_iiwa_link_7";
  	if ( !iiwa_tree.getChain(base_link, tip_link, _k_chain) ) return false;
  	_fksolver = new KDL::ChainFkSolverPos_recursive( _k_chain );
        _J_solver = new KDL::ChainJntToJacSolver(_k_chain);
        _q_in = new KDL::JntArray( _k_chain.getNrOfJoints() );
        _J = new KDL::Jacobian( _k_chain.getNrOfJoints() );
  	return true;
    }

    void ClikP::control_loop() {
         std_msgs::Float64 cmd[7];;
         KDL::JntArray q_out(_k_chain.getNrOfJoints());
         KDL::JntArray q_dot_out(_k_chain.getNrOfJoints());
         ros::Rate r(100);

         Vector3d error( 0 , 0 , 0 );
         Vector3d u( 0 , 0 , 0 );
         VectorXd q_dot(7);
         VectorXd q(7);
         MatrixXd J(6,7);
         MatrixXd _J_t(7,3);
         float Kp  = 20;
	
	 while(init<100) usleep(1);
	 q=_q_in->data;
         while( ros::ok() ) {

              _fksolver->JntToCart(*_q_in, _p_out);

              if (!trovato) {
                 _des_pos.x = _p_out.p.x();
                 _des_pos.y = _p_out.p.y();
                 _des_pos.z = _p_out.p.z();
                 trovato = true;
              }


    	      error(0) = _des_pos.x - _p_out.p.x();
    	      error(1) = _des_pos.y - _p_out.p.y();
    	      error(2) = _des_pos.z - _p_out.p.z();
              u = Kp*error;

              _J_solver->JntToJac(*_q_in, *_J);
              _J_t = _J->data.block(0,0,3,7).transpose();
              q_dot = _J_t*u;
              q = q + 0.01*q_dot;

              for(int i=0; i<7; i++) {
  	          cmd[i].data = q(i);
  	      }
  		
              for(int i=0; i<7; i++) {
  	         _cmd_pub[i].publish(cmd[i]);
  	      }

  	      r.sleep();
  	}
  }

  

  GZ_REGISTER_MODEL_PLUGIN(ClikP)
}
