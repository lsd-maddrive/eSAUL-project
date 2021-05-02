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

#define Pi 3.1415926535
#define noise 0
#define size_noise 0.01

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

				_joint_left= model->GetJoint("joint_left_wheel");
				_joint_right = model->GetJoint("joint_right_wheel");

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
				odom_pub = nh.advertise<nav_msgs::Odometry>("/eSAUl/encoder", 50);
				cmd_vel_sub_ = nh.subscribe("cmd_vel", 50, &ModelPush::CmdVel, this);
			}
		void updateOdom()
			{
				delta_time = ros::Time::now().toSec()-delta_time;
				if(delta_time>0)
				{
					if(noise==1)
					{
						temp2 = (rand()/( (double)RAND_MAX )); 
						
						if (temp2 == 0)
							p = 1;
						else
							p = -1;

						temp1 = cos( ( 2.0 * (double)Pi ) * rand() / ( (double)RAND_MAX ) );
						result = sqrt( -2.0 * log( temp2 ) ) * temp1 * size_noise;
					}
					else 
						result = 0;

					left_turn = (_joint_left->Position(0))-left_turn;
					right_turn = (_joint_right->Position(0))-right_turn;		

					left_speed = left_turn*rad/delta_time;
					right_speed = right_turn*rad/delta_time;	

					alpha = alpha + (right_speed-left_speed)/(width+rad)*delta_time;
					x = x + (left_speed+right_speed)*0.5*cos(alpha)*delta_time;
					y = y + (left_speed+right_speed)*0.5*sin(alpha)*delta_time;	

					if ((x-x_last)>0.01 || (y-y_last)>0.01 || (alpha-alpha_last)>0.01)
					{
						x = x + result;
						y = y + result;						
						alpha = alpha + result;						
					}
					x_last = x;
					y_last = y;
					alpha_last = alpha;
				}
				delta_time = ros::Time::now().toSec();
				left_turn = (_joint_left->Position(0));
				right_turn = (_joint_right->Position(0));		
			}		
			
		void publishOdometryData()
			{
				nav_msgs::Odometry msg;
				geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(alpha);
				msg.header.stamp = ros::Time::now();
				msg.header.frame_id = "odom";

				msg.pose.pose.position.x = x;
				msg.pose.pose.position.y = y;
				msg.pose.pose.position.z = 0.0;
				msg.pose.pose.orientation = odom_quat;

				msg.child_frame_id = "base_link";
				msg.twist.twist.linear.x = line_speed;
				msg.twist.twist.angular.z = ang_speed;

				odom_pub.publish(msg);

				

				// geometry_msgs::TransformStamped odom_trans;
				// odom_trans.header.stamp = ros::Time::now();
				// odom_trans.header.frame_id = "odom";
				// odom_trans.child_frame_id = "base_link";

				// odom_trans.transform.translation.x = x;
				// odom_trans.transform.translation.y = y;
				// odom_trans.transform.translation.z = 0.0;
				// odom_trans.transform.rotation = odom_quat;

				// br->sendTransform(odom_trans);
			}
		void OnUpdate()
			{
				updateOdom();
				publishOdometryData();	
				_joint_left->SetVelocity(0, speed_l);
				_joint_right->SetVelocity(0, speed_r);
					// if(abs(left_mid_speed-right_mid_speed)<0.1)
					// {
					// 	delta_time = ros::Time::now().toSec() - last_time;
					// 	alpha = alpha;
					// 	x = x + (left_mid_speed+right_mid_speed)*0.5*rad*cos(alpha)*delta_time;
					// 	y = y + (left_mid_speed+right_mid_speed)*0.5*rad*sin(alpha)*delta_time;
					// 	publishOdometryData();
					// 	last_time = ros::Time::now().toSec();
					// }
					// if(abs(left_speed_1-left_speed_2)>1 || abs(left_speed_1-left_speed_3)>1)
					// {
					// 	delta_time = ros::Time::now().toSec() - last_time;
					// 	alpha = alpha + rad*(left_mid_speed-right_mid_speed)/width*delta_time;
					// 	x = x + (left_mid_speed+right_mid_speed)*0.5*rad*cos(alpha)*delta_time;
					// 	y = y + (left_mid_speed+right_mid_speed)*0.5*rad*sin(alpha)*delta_time;
					// 	// if(left_mid - right_mid<0.01)
					// 	// {
					// 	// 	x = rad*(left_mid + right_mid)/2;
					// 	// 	y = rad*(left_mid + right_mid)/2;
					// 	// }
					// 	// if (abs(x-t_last)>0.01)
					// 	// {
					// 	// 	cout << rad*(left_mid_speed-right_mid_speed)/width << endl;
					// 	// 	cout << alpha << endl;
					// 	// 	cout << abs(left_speed_1-left_speed_2) << endl;
					// 	// 	cout << abs(left_speed_1-left_speed_3) << endl;
					// 	// 	// cout << right_speed_3 << endl;
					// 	// 	cout << "..........................." << endl;
					// 	// 	t_last = x;
					// 	// }	
					// 	//cout << "Зашел сюды" << endl;
					// 	publishOdometryData();
					// 	last_time = ros::Time::now().toSec();
					// }
					// delta_time = ros::Time::now().toSec() - last_time;

					// alpha = alpha + ang_speed*delta_time;
					// //x = x+(speed_r*rad + speed_l*rad)*0.5*cos(alpha)*delta_time;
					// y = y+(speed_r*rad + speed_l*rad)*0.5*sin(alpha)*delta_time;

					// last_time = ros::Time::now().toSec();
					// alpha_deg=alpha*180/3,1415926535;
					// if((last_x!=x) || (last_y!=y) || (last_alpha!=alpha))
					// {
					// 	cout << "Позиция по Х: " << x << endl;
					// 	cout << "Позиция по Y: " << y << endl;
					// 	cout << "Угол градусы: " << alpha_deg << endl;
					// 	last_x = x;
					// 	last_y = y;
					// 	last_alpha = alpha;
					// }
				
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
		double t_last, left_turn=0, right_turn=0;
		double left_speed=0, right_speed=0;
		double speed_r=0;
		double speed_l=0;
		double ang_speed=0, line_speed=0;
		double width=0;
		double rad=0;
		double delta_time=0;
		double x=0, y=0, alpha=0, pos_x=0, pos_y=0, last_x=0, last_y=0, last_alpha=0, alpha_deg=0;
		double size = 100;
		double p=1, temp2=0, temp1=0, result=0;
		double x_last=0, y_last=0, alpha_last=0;
		physics::ModelPtr model;
		sdf::ElementPtr sdf;
		std::string roboname;	
		ros::NodeHandle nh;
		event::ConnectionPtr updateConnection;
		ros::Subscriber cmd_vel_sub_;
		physics::JointPtr _joint_left;
		physics::JointPtr _joint_right;
		std_msgs::Float32 msg;
		ros::Publisher odom_pub;
		shared_ptr<tf::TransformBroadcaster> br;
	};

  	GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
