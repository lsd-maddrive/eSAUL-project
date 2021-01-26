#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include "geometry_msgs/Twist.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/tfMessage.h>

using namespace std;

namespace gazebo
{
class ModelPush : public ModelPlugin
{
	public: 
	ModelPush() : ModelPlugin()
  	{
		cout << "********************Plugin is creating********************" << endl;	
  	}
	void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
	{
		this->model = _parent;
		this->sdf = _sdf;

		wheel_joint_right_1 = model->GetJoint("joint_right_wheel_1");
		wheel_joint_left_1 = model->GetJoint("joint_left_wheel_1");
		wheel_joint_right_2 = model->GetJoint("joint_right_wheel_2");
		wheel_joint_left_2 = model->GetJoint("joint_left_wheel_2");
		wheel_joint_right_3 = model->GetJoint("joint_right_wheel_3");
		wheel_joint_left_3 = model->GetJoint("joint_left_wheel_3");
		wheel_joint_right_4 = model->GetJoint("joint_right_wheel_4");
		wheel_joint_left_4 = model->GetJoint("joint_left_wheel_4");

		if (sdf->HasElement("robotName"))
		{
			roboname = sdf->Get<std::string>("robotName");
			cout << "********************Name received********************" << endl;
			cout << "His name is " << roboname << endl;
		}
		else
		{
			cout << "********************Name don't received********************" << endl;
		}
		updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelPush::OnUpdate, this));
		nh = ros::NodeHandle(roboname);
    		cmd_vel_sub_ = nh.subscribe("cmd_vel", 1, &ModelPush::CmdVel, this);
	}
	void OnUpdate()
    	{
      		wheel_joint_right_1->SetVelocity(0, lin_speed);
		wheel_joint_left_1->SetVelocity(0, lin_speed);
    	}
	void CmdVel(const geometry_msgs::Twist& command)
    	{
		lin_speed = command.linear.x;
		
      	}
	~ModelPush()
	{
		nh.shutdown();
	}
	private: 
	double lin_speed=0;
	physics::ModelPtr model;
	physics::JointPtr wheel_joint_left_1;
	physics::JointPtr wheel_joint_right_1;
	physics::JointPtr wheel_joint_left_2;
	physics::JointPtr wheel_joint_right_2;
	physics::JointPtr wheel_joint_left_3;
	physics::JointPtr wheel_joint_right_3;
	physics::JointPtr wheel_joint_left_4;
	physics::JointPtr wheel_joint_right_4;
	sdf::ElementPtr sdf;
	std::string roboname;	
    	ros::NodeHandle nh;
	event::ConnectionPtr updateConnection;
	ros::Subscriber cmd_vel_sub_;
};
  
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
