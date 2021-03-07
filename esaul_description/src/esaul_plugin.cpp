#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
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
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

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
			br = make_shared<tf::TransformBroadcaster>();
			nh = ros::NodeHandle(roboname);
			odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
			cmd_vel_sub_ = nh.subscribe("cmd_vel", 50, &ModelPush::CmdVel, this);
		}
	void publishOdometryData()
		{
			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(alpha);
			geometry_msgs::TransformStamped odom_trans;
			odom_trans.header.stamp = ros::Time::now();
			odom_trans.header.frame_id = "map";
			odom_trans.child_frame_id = "base_link";

			odom_trans.transform.translation.x = x;
			odom_trans.transform.translation.y = y;
			odom_trans.transform.translation.z = 0.0;
			odom_trans.transform.rotation = odom_quat;

    		br->sendTransform(odom_trans);
		}
	void OnUpdate()
    	{	
			publishOdometryData();
			_joint_left1->SetVelocity(0, speed_l);
			_joint_left2->SetVelocity(0, speed_l);
			_joint_left3->SetVelocity(0, speed_l);

			_joint_right1->SetVelocity(0, speed_r);
			_joint_right2->SetVelocity(0, speed_r);
			_joint_right3->SetVelocity(0, speed_r);				

			delta_time = ros::Time::now().toSec() - last_time;

			alpha = alpha + ang_speed*delta_time;
			x = x+(speed_r*rad + speed_l*rad)*0.5*cos(alpha)*delta_time;
			y = y+(speed_r*rad + speed_l*rad)*0.5*sin(alpha)*delta_time;

			last_time = ros::Time::now().toSec();
			alpha_deg=alpha*180/3,1415926535;
			if((last_x!=x) || (last_y!=y) || (last_alpha!=alpha))
			{
				cout << "Позиция по Х: " << x << endl;
				cout << "Позиция по Y: " << y << endl;
				cout << "Угол градусы: " << alpha_deg << endl;
				last_x = x;
				last_y = y;
				last_alpha = alpha;
			}

    	}
	void CmdVel(const geometry_msgs::Twist &msg)
    	{
			ang_speed = msg.angular.z;
			speed_r = msg.linear.x/rad + (msg.angular.z*width/(2*rad));
			speed_l = msg.linear.x/rad - (msg.angular.z*width/(2*rad));
 			 
      	}
	~ModelPush()
		{
			nh.shutdown();
		}
	private: 
	double x=0, y=0, delta_time=0, alpha=0, pos_x=0, pos_y=0, last_x=0, last_y=0, last_alpha=0, alpha_deg=0;
	double last_time;
	double speed_r=0;
	double speed_l=0;
	double ang_speed=0;
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
	shared_ptr<tf::TransformBroadcaster> br;
};
  
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
