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
		_joint_left= model->GetJoint("joint_left");
		_joint_right = model->GetJoint("joint_right");
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

    	}
	void CmdVel(const geometry_msgs::Twist& command)
    	{
		speed = command.linear.x;
		
      	}
	~ModelPush()
	{
		nh.shutdown();
	}
	private: 
	double speed=0;
	//double lin_speed;
	physics::ModelPtr model;
	sdf::ElementPtr sdf;
	std::string roboname;
	//physics::ContactManager *contactManager;	
    	ros::NodeHandle nh;
	event::ConnectionPtr updateConnection;
	ros::Subscriber cmd_vel_sub_;
	physics::JointPtr _joint_left;
	physics::JointPtr _joint_right;
};
  
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
