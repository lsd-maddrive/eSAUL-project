#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include "geometry_msgs/Twist.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/tfMessage.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

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
		
		sdf->GetElement("width_chassis")->GetValue()->Get(width);
		sdf->GetElement("wh_rad")->GetValue()->Get(rad);

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
		odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    	cmd_vel_sub_ = nh.subscribe("cmd_vel", 50, &ModelPush::CmdVel, this);
		// br = make_shared<tf::TransformBroadcaster>();
	}
	void OnUpdate()
    	{	

			_joint_left1->SetVelocity(0, lin_speed_l);
			_joint_left2->SetVelocity(0, lin_speed_l);
			_joint_left3->SetVelocity(0, lin_speed_l);

			_joint_right1->SetVelocity(0, lin_speed_r);
			_joint_right2->SetVelocity(0, lin_speed_r);
			_joint_right3->SetVelocity(0, lin_speed_r);
    	}
	void CmdVel(const geometry_msgs::Twist &msg)
    	{
			// lin_speed_r = command.linear.x/rad + (command.angular.z*width/2)/rad;
			// lin_speed_l = command.linear.x/rad - (command.angular.z*width/2)/rad; 
			lin_speed_r = msg.linear.x/rad + (msg.angular.z*width/2)/rad;
			lin_speed_l = msg.linear.x/rad - (msg.angular.z*width/2)/rad; 
      	}
	~ModelPush()
		{
			nh.shutdown();
		}
	private: 
	double x, y;
	double ang_speed=0;
	double lin_speed_r=0;
	double lin_speed_l=0;
	double speed=0;
	double width=0;
	double rad=0;
	physics::ModelPtr model;
	sdf::ElementPtr sdf;
	std::string roboname;	
    ros::NodeHandle nh;
	event::ConnectionPtr updateConnection;
	ros::Subscriber cmd_vel_sub_;
	physics::JointPtr _joint_left1;
	physics::JointPtr _joint_right1;
	physics::JointPtr _joint_left2;
	physics::JointPtr _joint_right2;
	physics::JointPtr _joint_left3;
	physics::JointPtr _joint_right3;
	std_msgs::Float32 msg;
	ros::Publisher odom_pub;
	//hared_ptr<tf::TransformBroadcaster> br;
	//ros::Time current_time = ros::Time::now();
};
  
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
