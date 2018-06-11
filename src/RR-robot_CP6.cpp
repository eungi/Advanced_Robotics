#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <termios.h>

#define PI 3.1415926535897

ros::Publisher thetas1_pub;
ros::Publisher positions_pub;
ros::Publisher positions2_pub;
ros::Publisher positions3_pub;

ros::Publisher thetas_plot;
ros::Publisher omegas_plot;
ros::Publisher runge_thetas_plot;
ros::Publisher runge_omegas_plot;
ros::Publisher prob4_thetas_plot;
ros::Publisher prob4_omegas_plot;
ros::Publisher hand_error_plot;

float time_ = 0;
char check;

float x[201];
float y[201];
int j = 0;
int flag = 0;

float f = 5;
int L1 = 1, L2 = 1;

float theta1_runge = 0, theta2_runge = 0;
float omega1_runge = 0, omega2_runge = 0;

float theta1_prob4 = 0, theta2_prob4 = 0;
float omega1_prob4 = 0, omega2_prob4 = 0;

float x_result[201];
float y_result[201];

pcl::PointCloud<pcl::PointXYZRGB> position_cluster1;
pcl::PointCloud<pcl::PointXYZRGB> position_cluster2;
pcl::PointCloud<pcl::PointXYZRGB> position_cluster3;

double (*inverse(double a[2][2]))[2];
char getch();

int main(int argc, char** argv) {
	ros::init(argc, argv, "computer_problem_6_node");
	ros::NodeHandle nh;

	thetas1_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
	positions_pub = nh.advertise<sensor_msgs::PointCloud2>("/positions", 1);
	positions2_pub = nh.advertise<sensor_msgs::PointCloud2>("/positions2", 1);
	positions3_pub = nh.advertise<sensor_msgs::PointCloud2>("/positions3", 1);

	thetas_plot = nh.advertise<sensor_msgs::JointState>("/thetas_plot", 1);
	omegas_plot = nh.advertise<sensor_msgs::JointState>("/omegas_plot", 1);
	runge_thetas_plot = nh.advertise<sensor_msgs::JointState>("/runge_thetas_plot", 1);
	runge_omegas_plot = nh.advertise<sensor_msgs::JointState>("/runge_omegas_plot", 1);
	prob4_thetas_plot = nh.advertise<sensor_msgs::JointState>("/prob4_thetas_plot", 1);
	prob4_omegas_plot = nh.advertise<sensor_msgs::JointState>("/prob4_omegas_plot", 1);
	hand_error_plot = nh.advertise<sensor_msgs::JointState>("/hand_error_plot", 1);

    	ros::Rate loop_rate(10);

	std::string joint_name[2] = {"joint0", "joint1"};
	float joint_position[2] ;//= {0, 0};
	double d_theta[2] = {0, 0};
	float runge_joint_position[2] = {0, 0};
	double runge_d_theta[2] = {0, 0};
	float prob4_joint_position[2] = {0, 0};
	double prob4_d_theta[2] = {0, 0};
	double hand_error[2] = {0, 0};

	float l = 1;

	while(ros::ok()) {
        	ros::spinOnce();

		sensor_msgs::JointState thetas1;
		thetas1.header.stamp = ros::Time::now();
		thetas1.header.frame_id = "base";
		position_cluster1.header.frame_id = "base";
		position_cluster2.header.frame_id = "base";
		position_cluster3.header.frame_id = "base";

		sensor_msgs::JointState plot_thetas;
		plot_thetas.header.stamp = ros::Time::now();
		plot_thetas.header.frame_id = "base";

		sensor_msgs::JointState plot_omegas;
		plot_omegas.header.stamp = ros::Time::now();
		plot_omegas.header.frame_id = "base";

		sensor_msgs::JointState runge_plot_thetas;
		runge_plot_thetas.header.stamp = ros::Time::now();
		runge_plot_thetas.header.frame_id = "base";

		sensor_msgs::JointState runge_plot_omegas;
		runge_plot_omegas.header.stamp = ros::Time::now();
		runge_plot_omegas.header.frame_id = "base";

		sensor_msgs::JointState prob4_plot_thetas;
		prob4_plot_thetas.header.stamp = ros::Time::now();
		prob4_plot_thetas.header.frame_id = "base";

		sensor_msgs::JointState prob4_plot_omegas;
		prob4_plot_omegas.header.stamp = ros::Time::now();
		prob4_plot_omegas.header.frame_id = "base";

		sensor_msgs::JointState hand_errors;
		hand_errors.header.stamp = ros::Time::now();
		hand_errors.header.frame_id = "base";

		char c;
        	c = getch();
		pcl::PointXYZRGB pt;

		if(c!=0) { time_ = 0; j = 0; check = c; flag = 0; }

		if(check != 0) {
			if(flag == 0) {
				// Problem 1		// Draw Track + Inverse Kinematics + Jacobian(omega1,2)
				// Draw Track
				float x_ = (0.7 * cos(2 * PI * f * time_)) + 0.1;
				float y_ = (-0.7 * cos(2 * PI * f * time_)) + 0.1;

				x[j] = x_; 
				y[j] = y_;

				pt.x = x[j], pt.y = y[j], pt.z = 0;
				pt.r = 255, pt.g = 150, pt.b = 0;
				position_cluster1.push_back(pt);
				
				// Inverse Kinematics
				float c2 = (pow(x[j], 2) + pow(y[j], 2) - 2) / 2;	
				float s2 = sqrt(1 - pow(c2, 2));

				if(atan2(s2, c2) > 0 && atan2(s2, c2) < PI)
					joint_position[1] = atan2(s2, c2);
				else
					joint_position[1] = -atan2(s2, c2);

				float c1 = ((1 + c2) * x[j] + s2 * y[j]) / (pow((1 + c2), 2) + pow(s2, 2));
				float s1 = (-1 * s2 * x[j] + (1 + c2) * y[j]) / (pow((1 + c2), 2) + pow(s2, 2));
				joint_position[0] = atan2(s1, c1);

				// Jacobian
				double J[2][2] = { { (-L1 * sin(joint_position[0]) - L2 * sin(joint_position[0] + joint_position[1])), (-L2 * sin(joint_position[0] + joint_position[1])) },
					   	   { (L1 * cos(joint_position[0]) + L2 * cos(joint_position[0] + joint_position[1])), (L2 * cos(joint_position[0] + joint_position[1])) } };

				double(*J_inverse)[2] = inverse(J);

				float dx = -0.7 * sin(2 * PI * f * time_) * 2 * PI * f;
				float dy = 0.7 * sin(2 * PI * f * time_) * 2 * PI * f;

				d_theta[0] = J_inverse[0][0] * dx + J_inverse[0][1] * dy;
				d_theta[1] = J_inverse[1][0] * dx + J_inverse[1][1] * dy;

				for(int i = 0; i < 2; i++) {
					plot_thetas.name.push_back(joint_name[i]);
					plot_thetas.position.push_back(joint_position[i]);
					plot_omegas.name.push_back(joint_name[i]);
					plot_omegas.position.push_back(d_theta[i]);
				}

				thetas_plot.publish(plot_thetas);
				omegas_plot.publish(plot_omegas);

				// Problem 2
				float Kp1 = pow((16 * PI), 2) * 4 / 3;
				float Kd1 = 2 * 16 * PI * 4 / 3;
				float Kp2 = pow((16 * PI), 2) * 4 / 15;
				float Kd2 = 2 * 16 * PI * 4 / 15;

				// Problem 3		// Runge-Kutta + Forward Kinematics
				float tau1 = Kp1 * (joint_position[0] - theta1_runge) + Kd1 * (d_theta[0] - omega1_runge);
				float tau2 = (Kp2 * (joint_position[1] - theta2_runge)) + (Kd2 * (d_theta[1] - omega2_runge));
				
				float X1 = joint_position[0];
				float X2 = d_theta[0];
				float X3 = joint_position[1];
				float X4 = d_theta[1];

				float X1_dot = X2;
				float X2_dot = 0.75 * tau1;
				float X3_dot = X4;
				float X4_dot = 3.75 * tau2;

				float K1 = 0.01 * omega1_runge;
				float L1 = 0.01 * X2_dot;
				float B1 = 0.01 * omega2_runge;
				float C1 = 0.01 * X4_dot;

				float K2 = 0.01 * (omega1_runge + L1);
				float B2 = 0.01 * (omega2_runge + C1);
				float L2 = 0.01 * (X2_dot);
				float C2 = 0.01 * (X4_dot);

				theta1_runge += (K1 + K2) * 0.5;
				theta2_runge += (B1 + B2) * 0.5;
				omega1_runge += (L1 + L2) * 0.5;
				omega2_runge += (C1 + C2) * 0.5;

				tau1 = Kp1 * (joint_position[0] - theta1_runge) + Kd1 * (d_theta[0] - omega1_runge);
				tau2 = (Kp2 * (joint_position[1] - theta2_runge)) + (Kd2 * (d_theta[1] - omega2_runge));
				float X2_dot_2 = 0.75 * tau1;
				float X4_dot_2 = 3.75 * tau2;

				L2 = 0.01 * (X2_dot_2);
				C2 = 0.01 * (X4_dot_2);

				omega1_runge += (L1 + L2) / 2;
				omega2_runge += (C1 + C2) / 2;

				runge_joint_position[0] = theta1_runge;
				runge_joint_position[1] = theta2_runge;
				runge_d_theta[0] = omega1_runge;
				runge_d_theta[1] = omega2_runge;

				for(int i = 0; i < 2; i++) {
					runge_plot_thetas.name.push_back(joint_name[i]);
					runge_plot_thetas.position.push_back(runge_joint_position[i]);
					runge_plot_omegas.name.push_back(joint_name[i]);
					runge_plot_omegas.position.push_back(runge_d_theta[i]);

					//joint_position[i] = runge_joint_position[i];
				}

				runge_thetas_plot.publish(runge_plot_thetas);
				runge_omegas_plot.publish(runge_plot_omegas);

				x_result[j] = ((l*cos(runge_joint_position[0])) + (l*cos(runge_joint_position[0] + runge_joint_position[1])));
				y_result[j] = ((l*sin(runge_joint_position[0])) + (l*sin(runge_joint_position[0] + runge_joint_position[1])));

				pt.x = x_result[j], pt.y = y_result[j], pt.z = 0;
				pt.r = 0, pt.g = 0, pt.b = 255;
				position_cluster2.push_back(pt);

				// Problem 4		// Nonlinear Robot + Runge-Kutta + Forward Kinematics
				float H11 = cos(joint_position[1]) + 1.6666666666;
				float H12 = cos(joint_position[1]) * 0.5 + 0.3333333333;
				float H21 = cos(joint_position[1]) * 0.5 + 0.3333333333;
				float H22 = 0.3333333333;
				double H[2][2] = { { H11, H12 }, { H21, H22 } };

				double(*H_inverse)[2] = inverse(H);

				tau1 = Kp1 * (joint_position[0] - theta1_prob4) + Kd1 * (d_theta[0] - omega1_prob4);
				tau2 = Kp2 * (joint_position[1] - theta2_prob4) + Kd2 * (d_theta[1] - omega2_prob4);
				
				X1 = joint_position[0];
				X2 = d_theta[0];
				X3 = joint_position[1];
				X4 = d_theta[1];

				X1_dot = X2;
				X2_dot = H_inverse[0][0] * tau1 + H_inverse[0][1] * tau2;
				X3_dot = X4;
				X4_dot = H_inverse[1][0] * tau1 + H_inverse[1][1] * tau2;

				K1 = 0.01 * omega1_prob4;
				L1 = 0.01 * X2_dot;
				B1 = 0.01 * omega2_prob4;
				C1 = 0.01 * X4_dot;

				K2 = 0.01 * (omega1_prob4 + L1);
				B2 = 0.01 * (omega2_prob4 + C1);
				L2 = 0.01 * X2_dot;
				C2 = 0.01 * X4_dot;

				theta1_prob4 += (K1 + K2) * 0.5;
				theta2_prob4 += (B1 + B2) * 0.5;
				omega1_prob4 += (L1 + L2) * 0.5;
				omega2_prob4 += (C1 + C2) * 0.5;

				tau1 = Kp1 * (joint_position[0] - theta1_prob4) + Kd1 * (d_theta[0] - omega1_prob4);
				tau2 = Kp2 * (joint_position[1] - theta2_prob4) + Kd2 * (d_theta[1] - omega2_prob4);

				X2_dot_2 = H_inverse[0][0] * tau1 + H_inverse[0][1] * tau2;
				X4_dot_2 = H_inverse[1][0] * tau1 + H_inverse[1][1] * tau2;

				L2 = 0.01 * (X2_dot_2);
				C2 = 0.01 * (X4_dot_2);

				omega1_prob4 += (L1 + L2) * 0.5;
				omega2_prob4 += (C1 + C2) * 0.5;

				prob4_joint_position[0] = theta1_prob4;
				prob4_joint_position[1] = theta2_prob4;
				prob4_d_theta[0] = omega1_prob4;
				prob4_d_theta[1] = omega2_prob4;
				hand_error[0] = sqrt(pow(joint_position[0] - prob4_joint_position[0], 2) + pow(joint_position[1] - prob4_joint_position[1], 2));
				hand_error[1] = 0;

				for(int i = 0; i < 2; i++) {
					prob4_plot_thetas.name.push_back(joint_name[i]);
					prob4_plot_thetas.position.push_back(prob4_joint_position[i]);
					prob4_plot_omegas.name.push_back(joint_name[i]);
					prob4_plot_omegas.position.push_back(prob4_d_theta[i]);
					hand_errors.name.push_back(joint_name[i]);
					hand_errors.position.push_back(hand_error[i]);

					joint_position[i] = prob4_joint_position[i];
				}

				prob4_thetas_plot.publish(prob4_plot_thetas);
				prob4_omegas_plot.publish(prob4_plot_omegas);
				hand_error_plot.publish(hand_errors);

				x_result[j] = ((l*cos(joint_position[0])) + (l*cos(joint_position[0] + joint_position[1])));
				y_result[j] = ((l*sin(joint_position[0])) + (l*sin(joint_position[0] + joint_position[1])));

				pt.x = x_result[j], pt.y = y_result[j], pt.z = 0;
				pt.r = 255, pt.g = 0, pt.b = 0;
				position_cluster3.push_back(pt);

				if(time_ < 2.00 - 0.01) {
					j++;
					time_ += 0.01;
				}
				else { flag++; time_ = 0; j = 0; }
			}

			sensor_msgs::PointCloud2 output_position3;
	  		pcl::toROSMsg(position_cluster3, output_position3);
	  		output_position3.header.stamp = thetas1.header.stamp;
			//std::cout << output_position << std::endl;

			positions3_pub.publish(output_position3);

			sensor_msgs::PointCloud2 output_position2;
	  		pcl::toROSMsg(position_cluster2, output_position2);
	  		output_position2.header.stamp = thetas1.header.stamp;
			//std::cout << output_position << std::endl;

			positions2_pub.publish(output_position2);

			sensor_msgs::PointCloud2 output_position1;
	  		pcl::toROSMsg(position_cluster1, output_position1);
	  		output_position1.header.stamp = thetas1.header.stamp;
			//std::cout << output_position << std::endl;

			positions_pub.publish(output_position1);
		}
		
		for(int i = 0; i < 2; i++) {
			thetas1.name.push_back(joint_name[i]);
			thetas1.position.push_back(joint_position[i]);
		}

		thetas1_pub.publish(thetas1);
		loop_rate.sleep();
    	}
}

double(*inverse(double a[2][2]))[2]{
	static double result[2][2];
	double f = a[0][0] * a[1][1] - a[0][1] * a[1][0];

	result[0][0] = a[1][1] / f;
	result[0][1] = -1 * a[0][1] / f;
	result[1][0] = -1 * a[1][0] / f;
	result[1][1] = a[0][0] / f;

	return result;
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
