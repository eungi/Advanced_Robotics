#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <termios.h>

#define PI 3.1415926535897

ros::Publisher thetas_pub;
ros::Publisher positions_pub;

float time_ = 0;
float time2 = 0;
char check;

float x[201];
float y[201];
int j = 0;

char getch();

int main(int argc, char** argv) {
	ros::init(argc, argv, "robot_control_node");
	ros::NodeHandle nh;

	thetas_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
	positions_pub = nh.advertise<sensor_msgs::PointCloud2>("/positions", 1);
    	ros::Rate loop_rate(10);

	std::string joint_name[2] = {"joint0", "joint1"};
	float joint_position[2] = {0, 0};

	float f1 = 1, f2 = 1;

	while(ros::ok()) {
        	ros::spinOnce();

		char c;
        	c = getch();

		if(c!=0) { time_ = 0; time2 = 0; j = 0; check = c; }

		sensor_msgs::JointState thetas;
		thetas.header.stamp = ros::Time::now();
		thetas.header.frame_id = "base";

		joint_position[0] = 2 * PI * f1 * time_;	// Robot arm formula
		joint_position[1] = -1 * f2 * time2;

		x[j] = (2.0 + joint_position[1]) * cos(joint_position[0]);   // Track formula
		y[j] = (2.0 + joint_position[1]) * sin(joint_position[0]);
		
		for(int i = 0; i < 2; i++) {
			thetas.name.push_back(joint_name[i]);
			thetas.position.push_back(joint_position[i]);
		}

		pcl::PointCloud<pcl::PointXYZRGB> position_cluster;
  		position_cluster.header.frame_id = "base";
		pcl::PointXYZRGB pt;

		for (int i = 0; i < j; i += 2) {  // Track
			pt.x = x[i], pt.y = y[i], pt.z = 0;
			pt.r = 255, pt.g = 150, pt.b = 0;
			position_cluster.push_back(pt);
		}

		if(time_ < 2.01 - 0.01 && check != 0) {
			j++;
			time_ += 0.01;

			if(time_ < 1.00 - 0.01)
				time2 += 0.01;
			else
				time2 -= 0.01;
		}

		sensor_msgs::PointCloud2 output_position;
  		pcl::toROSMsg(position_cluster, output_position);
  		output_position.header.stamp = thetas.header.stamp;
		std::cout << output_position << std::endl;

		thetas_pub.publish(thetas);
		positions_pub.publish(output_position);

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
