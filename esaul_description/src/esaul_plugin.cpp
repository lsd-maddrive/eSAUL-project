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
#include <geometry_msgs/Pose2D.h>


#define Pi 3.1415926535
#define noise 1
#define size_noise 0.01
#define mode 0

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
				odom_pub_enc = nh.advertise<nav_msgs::Odometry>("/eSAUl/encoder", 100);
				odom_pub_lazer = nh.advertise<nav_msgs::Odometry>("/eSAUl/lazer_scan_matcher/odom", 100);

				cmd_vel_sub_ = nh.subscribe("cmd_vel", 50, &ModelPush::CmdVel, this);
				Calman_odom_sub_ = nh.subscribe("/Calman/odom", 50, &ModelPush::Calman, this);
				Lazer_scan_matcher_sub_ = nh.subscribe("/eSAUl/lazer_scan_matcher", 50, &ModelPush::ConvLazer, this);
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


					alpha = alpha + (right_speed-left_speed)/(width+rad)*delta_time + (right_speed-left_speed)/(width+rad)*delta_time*result;
					x = x + (left_speed+right_speed)*0.5*cos(alpha)*delta_time + (left_speed+right_speed)*0.5*cos(alpha)*delta_time*result;
					y = y + (left_speed+right_speed)*0.5*sin(alpha)*delta_time + (left_speed+right_speed)*0.5*sin(alpha)*delta_time *result;	

					x_nonoise = x_nonoise + (left_speed+right_speed)*0.5*cos(alpha_nonise)*delta_time;
					y_nonoise = y_nonoise + (left_speed+right_speed)*0.5*sin(alpha_nonise)*delta_time;
					alpha_nonise = alpha_nonise + (right_speed-left_speed)/(width+rad)*delta_time; 

					lin_speed_enc_x = (right_speed + left_speed) * cos(alpha);	
					lin_speed_enc_y = (right_speed + left_speed) * sin(alpha);

					ang_speed_enc_z = (right_speed - left_speed)/width;	

				}
				delta_time = ros::Time::now().toSec();
				left_turn = (_joint_left->Position(0));
				right_turn = (_joint_right->Position(0));		
			}		
			
		void publishOdometryData()
			{
				geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(alpha);
				geometry_msgs::Quaternion odom_quat_lazer = tf::createQuaternionMsgFromYaw(alpha_lazer);
				if(mode)
				{
					nav_msgs::Odometry msg;
					msg.header.stamp = ros::Time::now();
					msg.header.frame_id = "odom";

					msg.pose.pose.position.x = x;
					msg.pose.pose.position.y = y;
					msg.pose.pose.position.z = 0.0;
					msg.pose.pose.orientation = odom_quat;

					msg.child_frame_id = "base_link";
					msg.twist.twist.linear.x = lin_speed_enc_x;
					msg.twist.twist.linear.y = lin_speed_enc_y;
					msg.twist.twist.angular.z = ang_speed_enc_z;

					odom_pub_enc.publish(msg);

					nav_msgs::Odometry msg_1;

					msg_1.header.stamp = ros::Time::now();
					msg_1.header.frame_id = "odom";

					msg_1.pose.pose.position.x = x_lazer;
					msg_1.pose.pose.position.y = y_lazer;
					msg_1.pose.pose.position.z = 0.0;
					msg_1.pose.pose.orientation = odom_quat_lazer;

					msg_1.child_frame_id = "base_link";
					
					odom_pub_lazer.publish(msg_1);
				}
				
				else
				{
					geometry_msgs::TransformStamped odom_trans;
					odom_trans.header.stamp = ros::Time::now();
					odom_trans.header.frame_id = "odom";
					odom_trans.child_frame_id = "base_link";

					odom_trans.transform.translation.x = x;
					odom_trans.transform.translation.y = y;
					odom_trans.transform.translation.z = 0.0;
					odom_trans.transform.rotation = odom_quat;

					br->sendTransform(odom_trans);
				}
			}
		void OnUpdate()
			{
				updateOdom();
				publishOdometryData();
				_joint_left->SetVelocity(0, speed_l);
				_joint_right->SetVelocity(0, speed_r);
				real_x = model->RelativePose().Pos().X();
				real_y = model->RelativePose().Pos().Y();
				// FileWrite();	
				last_y = real_y;
				last_x = real_x;					
				
			}
			// void FileWrite()
			// {

			// 	ofstream out ("", ios::app);
			// 	if (out.is_open())
			// 	{
			// 		out << real_x << " " << real_y << " " << x << " " << y <<  " " << Calman_x << " " << Calman_y << " " << x_nonoise << " " << y_nonoise << std::endl;
			// 	}
			// 	out.close();

			// }
			
			void CmdVel(const geometry_msgs::Twist &msg)
			{
				ang_speed = msg.angular.z;
				speed_r = msg.linear.x/rad + (msg.angular.z*width/(2*rad));
				speed_l = msg.linear.x/rad - (msg.angular.z*width/(2*rad));
				
			}
			void Calman(const nav_msgs::Odometry &msg)
			{  
				Calman_x = msg.pose.pose.position.x;
				Calman_y = msg.pose.pose.position.y;
				
			}
			void ConvLazer(const geometry_msgs::Pose2D::ConstPtr &msg)
			{  
				x_lazer = msg->x;
				y_lazer = msg->y;
				alpha_lazer = msg->theta;
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
		double x_lazer = 0, y_lazer=0, alpha_lazer=0;
		double delta_time=0;
		double x=0, y=0, alpha=0, pos_x=0, pos_y=0, last_x=0, last_y=0, last_alpha=0, alpha_deg=0;
		double size = 100;
		double x_nonoise=0, y_nonoise=0, alpha_nonise=0;
		double p=1, temp2=0, temp1=0, result=0;
		double x_last=0, y_last=0, alpha_last=0;
		double lin_speed_enc_x = 0, lin_speed_enc_y = 0, ang_speed_enc_z = 0;
		double real_x=0, real_y=0, Calman_x=0, Calman_y=0; 
		physics::ModelPtr model;
		sdf::ElementPtr sdf;
		std::string roboname;	
		ros::NodeHandle nh;
		event::ConnectionPtr updateConnection;
		ros::Subscriber cmd_vel_sub_;
		ros::Subscriber Calman_odom_sub_;
		ros::Subscriber Lazer_scan_matcher_sub_;
		physics::JointPtr _joint_left;
		physics::JointPtr _joint_right;
		std_msgs::Float32 msg;
		ros::Publisher odom_pub_enc;
		ros::Publisher odom_pub_lazer;
		shared_ptr<tf::TransformBroadcaster> br;
	};

  	GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
