#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <termios.h>

#define PI 3.1415926535897

ros::Publisher thetas_pub;
ros::Publisher positions_pub;
ros::Publisher positions2_pub;

float time_ = 0;
char check;

float x[201];
float y[201];
int j = 0;
int flag = 0;

pcl::PointCloud<pcl::PointXYZRGB> position_cluster1;
pcl::PointCloud<pcl::PointXYZRGB> position_cluster2;

char getch();

int main(int argc, char** argv) {
	ros::init(argc, argv, "robot_control_node");
	ros::NodeHandle nh;

	thetas_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
	positions_pub = nh.advertise<sensor_msgs::PointCloud2>("/positions", 1);
	positions2_pub = nh.advertise<sensor_msgs::PointCloud2>("/positions2", 1);
    	ros::Rate loop_rate(10);

	std::string joint_name[2] = {"joint0", "joint1"};
	float joint_position[2] = {0, 0};

	float f1 = 1, f2 = 0.5;
	float l = 1;

	while(ros::ok()) {
        	ros::spinOnce();

		sensor_msgs::JointState thetas;
		thetas.header.stamp = ros::Time::now();
		thetas.header.frame_id = "base";
		position_cluster1.header.frame_id = "base";
		position_cluster2.header.frame_id = "base";

		char c;
        	c = getch();
		pcl::PointXYZRGB pt;

		if(c!=0) { time_ = 0; j = 0; check = c; }

		if(check != 0) {
			if(flag == 0) {
				// Draw an elliptical trajectory
				// (x * x) / (a * a) + (y * y) / (b * b)		// a = 2, b = 1
				float theta1 = 2 * PI * f1 * time_;	// Robot arm formula
				float theta2 = 2 * PI * f2 * time_;

				x[j] = ((l*cos(theta1)) + (l*cos(theta1 + theta2)));	// Track formula 
				y[j] = ((l*sin(theta1)) + (l*sin(theta1 + theta2)));

				if(j % 2 == 0){
					pt.x = x[j], pt.y = y[j], pt.z = 0;
					pt.r = 255, pt.g = 150, pt.b = 0;
					position_cluster1.push_back(pt);
				}

				if(time_ < 2.00 - 0.01) {
					j++;
					time_ += 0.01;
				}
				else { flag++; time_ = 0; j = 0; }
			}

			else if(flag == 1) {
				joint_position[0] = 2 * PI * f1 * time_;	// Robot arm formula 
				float theta2 = 2 * PI * f2 * time_;

				x[j] = ((l*cos(joint_position[0])) + (l*cos(joint_position[0] + theta2)));	// Track formula 
				y[j] = ((l*sin(joint_position[0])) + (l*sin(joint_position[0] + theta2)));

				pcl::PointCloud<pcl::PointXYZRGB> position_cluster;
		  		position_cluster.header.frame_id = "base";

				if(time_ < 2.00 - 0.01) {
					j++;
					time_ += 0.01;
				}
				else { flag++; time_ = 0; j = 0; }
			}

			else if(flag == 2) {
				joint_position[0] = 2 * PI * f1 * time_;	// Robot arm formula 
				joint_position[1] = 2 * PI * f2 * time_;

				x[j] = ((l*cos(joint_position[0])) + (l*cos(joint_position[0] + joint_position[1])));	// Track formula 
				y[j] = ((l*sin(joint_position[0])) + (l*sin(joint_position[0] + joint_position[1])));

				if(j % 2 == 0) {		// Track 2
					pt.x = x[j], pt.y = y[j], pt.z = 0;
					pt.r = 0, pt.g = 0, pt.b = 255;
					position_cluster2.push_back(pt);
				}

				if(time_ < 2.00 - 0.01) {
					j++;
					time_ += 0.01;
				}
				else { flag++; time_ = 0; j = 0; }
			}

			sensor_msgs::PointCloud2 output_position2;
	  		pcl::toROSMsg(position_cluster2, output_position2);
	  		output_position2.header.stamp = thetas.header.stamp;
			//std::cout << output_position << std::endl;

			positions2_pub.publish(output_position2);

			sensor_msgs::PointCloud2 output_position1;
	  		pcl::toROSMsg(position_cluster1, output_position1);
	  		output_position1.header.stamp = thetas.header.stamp;
			//std::cout << output_position << std::endl;

			positions_pub.publish(output_position1);
		}
		
		for(int i = 0; i < 2; i++) {
			thetas.name.push_back(joint_name[i]);
			thetas.position.push_back(joint_position[i]);
		}

		thetas_pub.publish(thetas);
		loop_rate.sleep();
    	}
}

char getch() {
    fd_set set;
    struct timeval timeout;
    int rv;
    char buff = 0;
    int len = 1;
    int filedesc = 0;
    FD_ZERO(&set);
    FD_SET(filedesc, &set);

    timeout.tv_sec = 0;
    timeout.tv_usec = 1000;

    rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

    struct termios old = {0};
    if (tcgetattr(filedesc, &old) < 0)
        ROS_ERROR("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(filedesc, TCSANOW, &old) < 0)
        ROS_ERROR("tcsetattr ICANON");

    if(rv == -1)
        ROS_ERROR("select");
    else if(rv == 0)
        {/*ROS_INFO("no_key_pressed");*/}
    else
        read(filedesc, &buff, len );

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
        ROS_ERROR ("tcsetattr ~ICANON");
    return (buff);
}
