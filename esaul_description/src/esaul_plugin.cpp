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
		_joint_left1= model->GetJoint("joint_left_wheel_1");
		_joint_right1 = model->GetJoint("joint_right_wheel_1");
		_joint_left2= model->GetJoint("joint_left_wheel_2");
		_joint_right2 = model->GetJoint("joint_right_wheel_2");
		_joint_left3= model->GetJoint("joint_left_wheel_3");
		_joint_right3 = model->GetJoint("joint_right_wheel_3");
		_joint_left4= model->GetJoint("joint_left_wheel_4");
		_joint_right4 = model->GetJoint("joint_right_wheel_4");
		
		sdf->GetElement("width_chassis")->GetValue()->Get(width);

		cout << "His width is " << width << endl;

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
			_joint_left1->SetVelocity(0, lin_speed_l);
			_joint_left2->SetVelocity(0, lin_speed_l);
			_joint_left3->SetVelocity(0, lin_speed_l);
			_joint_left4->SetVelocity(0, lin_speed_l);

			_joint_right1->SetVelocity(0, lin_speed_r);
			_joint_right2->SetVelocity(0, lin_speed_r);
			_joint_right3->SetVelocity(0, lin_speed_r);
			_joint_right4->SetVelocity(0, lin_speed_r);

    	}
	void CmdVel(const geometry_msgs::Twist& command)
    	{
			speed = command.linear.x;
			// ang_speed = command.angular.z;
			lin_speed_l = speed; 
			lin_speed_r = -lin_speed_l;
      	}
	~ModelPush()
		{
			nh.shutdown();
		}
	private: 
	double ang_speed=0;
	double lin_speed_r=0;
	double lin_speed_l=0;
	double speed=0;
	double width=0;
	physics::ModelPtr model;
	sdf::ElementPtr sdf;
	std::string roboname;
	//physics::ContactManager *contactManager;	
    ros::NodeHandle nh;
	event::ConnectionPtr updateConnection;
	ros::Subscriber cmd_vel_sub_;
	physics::JointPtr _joint_left1;
	physics::JointPtr _joint_right1;
	physics::JointPtr _joint_left2;
	physics::JointPtr _joint_right2;
	physics::JointPtr _joint_left3;
	physics::JointPtr _joint_right3;
	physics::JointPtr _joint_left4;
	physics::JointPtr _joint_right4;
};
  
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
