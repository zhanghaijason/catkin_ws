#include <ctime>
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <iomanip>
#include <raspicam/raspicam_cv.h>
#include "compute_ttc.h"


#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include "camera_processing.h"
#include "pid.hpp"
#include "control_mapping.h"
#include "kalman.hpp"

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
using namespace std; 
using namespace cv;
using namespace Eigen;

enum controlMode{takeoff = 0,Move2Start = 1,yawEstimate = 2, perchPrepare = 3, perchDisCTDTS = 4, perchDisIPTS =5, perchImg = 6, failSafe =7};

mavros_msgs::State current_state;
mavros_msgs::AttitudeTarget att_msg;

double a_zf = 0.0;
double v_zf = 0.0, v_xf =0.0, v_yf = 0.0;
//Eigen::Vector3d mavPos_;
//geometry_msgs::Quaternion ori;
double roll_fb, pitch_fb, yaw_fb;
float thrust_fb = 0;
float yaw_dot = 0;
int nCount = 100;
double x_fb = 0, y_fb =0, z_fb = 0;
Eigen::Vector3d toEigen(const geometry_msgs::Point&p){
    Eigen::Vector3d ev3(p.x, p.y, p.z);
    return ev3;
}

void Desired2Quater(mavros_msgs::AttitudeTarget &att_msg, const vector<float> &DesiredRPY){
	att_msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE | mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE | mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
	tf2::Quaternion goal;
	goal.setRPY(DesiredRPY[1], DesiredRPY[2], DesiredRPY[3]);
	att_msg.orientation.x = goal[0];
	att_msg.orientation.y = goal[1];
	att_msg.orientation.z = goal[2];
	att_msg.orientation.w = goal[3];
	att_msg.thrust = DesiredRPY[0];
	return;
}
/*
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}*/



void mavposeCallback(const geometry_msgs::PoseStamped::ConstPtr& c_pose){   //retrive position feedback
    x_fb = c_pose->pose.position.x;
    y_fb = c_pose->pose.position.y;
    z_fb = c_pose->pose.position.z;
    tf::Quaternion temp(c_pose->pose.orientation.x, c_pose->pose.orientation.y, c_pose->pose.orientation.z, c_pose->pose.orientation.w);
    tf::Matrix3x3 m(temp);
    m.getRPY(roll_fb, pitch_fb, yaw_fb);
}

void mavvelocityCallback(const geometry_msgs::TwistStamped:: ConstPtr & c_velocity){
    v_zf = c_velocity->twist.linear.z;
    v_xf = c_velocity->twist.linear.x;
    v_yf = c_velocity->twist.linear.y;
}

void IMUCallback(const sensor_msgs::Imu::ConstPtr & IMU_data){
    a_zf = IMU_data->linear_acceleration.z;
    yaw_dot = IMU_data->angular_velocity.z;
}

/*
void mavaccelCallback(const geometry_msgs::Vector3:: ConstPtr & c_accel){
    a_zf = c_accel->vector.z;
}*/


/*
void distance_sensorCallback(const sensor_msgs::Range::ConstPtr & distance_sensor){
    z_fb = distance_sensor->range;
}*/

/*********************************************************************************************/
/*kalman filter*/
int n = 1;
int m = 1;
float sigma_z = 2;
float sigma_a = 0.2;
Eigen::MatrixXd F(n,n);
Eigen::MatrixXd I(n,n);
Eigen::MatrixXd B(n,m);
Eigen::MatrixXd Q(n,n);
Eigen::MatrixXd H(m,n);
Eigen::VectorXd R(m);
Eigen::MatrixXd P(n, n);
Eigen::MatrixXd x_hat(n,m);
Eigen::MatrixXd U(m, m);
Eigen::VectorXd Z(m);
void  Kalmanf(const Eigen::VectorXd& Z, float yaw_dot, float dt){
  //define all the internal variables
  Eigen::MatrixXd x_hat_new(n,m);
  Eigen::MatrixXd Pup(n, n);
  Eigen::VectorXd Y_tilt;
  Eigen::VectorXd S;
  Eigen::MatrixXd K(n,m);
  R<<sigma_z*sigma_z;
  Q<<sigma_a*sigma_a;
  B<<dt;
  H<<1;
  F<<1;
  U<<yaw_dot;
  I << 1;
// update
//  cout <<"x_hat: "<< x_hat <<" "
  x_hat_new = F * x_hat+B*U;
  Pup = F*P*F.transpose() + Q;
  K = Pup*H.transpose()*(H*Pup*H.transpose() + R).inverse();
  x_hat_new += K * (Z - H*x_hat_new);
  P = (I - K*H)*Pup;
  x_hat = x_hat_new;
}

/******************************************************************************************/


int main ( int argc,char **argv ) {
	ros::init(argc, argv, "image_capture");
	ros::NodeHandle n_image;
	ros::NodeHandle n_imagePrivate("~");
	// define all the needed subscribers and publishers
//	ros::Subscriber state_sub = n_image.subscribe<mavros_msgs::State>("mavros/state", 1, &state_cb, ros::TransportHints().tcpNoDelay());
	ros::Subscriber pose_sub = n_image.subscribe("/mavros/local_position/pose", 1, &mavposeCallback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber velocity_sub = n_image.subscribe("/mavros/local_position/velocity_local", 1, &mavvelocityCallback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber acce_sub = n_image.subscribe("mavros/imu/data", 1, &IMUCallback, ros::TransportHints().tcpNoDelay());
//	ros::Subscriber acce_sub = n_image.subscribe("mavros/local_position/accel", 1, &mavaccelCallback, ros::TransportHints().tcpNoDelay());

//	ros::Subscriber distance_sub = n_image.subscribe("mavros/distance_sensor/hrlv_ez4_pub", 1, &distance_sensorCallback, ros::TransportHints().tcpNoDelay());
	ros::Publisher att_pub = n_image.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 5);
	ros::Publisher vel_pub = n_image.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 2);
	ros::Publisher pos_pub = n_image.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 2);
	ros::Rate loop_rate(100);
	int width = 192;
	int height = 144;
	double real_width = 3.68;
	double real_height = 2.76;

	int threshold;
	raspicam::RaspiCam_Cv Camera;
	cv::Mat image;
	vector<Mat> Concat;
	int controlMode = 0;
	double set_speed = 0.1;
	double z_limit = 0;
	double tau0 = 0, tau_ref = 0, C= 0.5, tauS = -0.5, tau =0, tS=0;
	bool switched = false; //the sign to see whether the condition to switch to second stage is met or not
	double xInitial = 2.8, yInitial = 0.00, zInitial = -0.9;
        geometry_msgs::Twist velmsg;    
   
	vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
	compression_params.push_back(100);
	//set camera params
	if ( findParam ( "-help",argc,argv ) !=-1 ) {
        	showUsage();
        	return -1;
   	 }
    	processCommandLine ( argc,argv,Camera );
    	Camera.set (CV_CAP_PROP_FPS, 180);
	Camera.set ( cv::CAP_PROP_FRAME_WIDTH,  getParamVal ( "-w",argc,argv,width ) );
    	Camera.set ( cv::CAP_PROP_FRAME_HEIGHT, getParamVal ( "-h",argc,argv,height ) );
	cout<<"Connecting to camera"<<endl;
    	if ( !Camera.open() ) {
        	cerr<<"Error opening camera"<<endl;
        	return -1;
    	}
	cout<<"Connected to camera ="<<Camera.getId() <<endl;
	// Prepare the image destination folder
	stringstream ss;
	string name = "image";
	string type = ".jpg";
	string folderName = "imageFolder";
	string folderCreateCommand = "mkdir "+folderName;
	system(folderCreateCommand.c_str());
	//
	//Open camera
	cout<<"Opening Camera..."<<endl;
	//Start capture
	cout<<"Capturing "<<nCount<<" frames ...."<<endl;
	ros::Time time_begin = ros::Time::now();
	ros::Time time_takeoff, time_perch, time_move, time_yawEstimate, time_ctdts, time_ipts, cur_time, time_switchtau, time_final;
	time_takeoff = time_begin;
	double Kp_x, Kp_y, Kp_z, Kd_x, Kd_y, Kd_z, Ki_x, Ki_y, Ki_z, Kp_t, Kd_t, Ki_t;
	int tau_control = 1;
	double k0 = -0.4848, k1 = -0.24, k2 = -0.8824, k3 = 0.3106; 
	bool verbose;
	double yaw_image = 0.0;
	double xGoal = 3.3, yGoal = 0, zGoal = -0.9;
	n_imagePrivate.getParam("controller_px", Kp_x);
	n_imagePrivate.getParam("controller_py", Kp_y);
	n_imagePrivate.getParam("controller_pz", Kp_z);
	n_imagePrivate.getParam("controller_pt", Kp_t);
	n_imagePrivate.getParam("controller_dx", Kd_x);
	n_imagePrivate.getParam("controller_dy", Kd_y);
	n_imagePrivate.getParam("controller_dz", Kd_z);
	n_imagePrivate.getParam("controller_dt", Kd_t);	
	n_imagePrivate.getParam("controller_ix", Ki_x);
	n_imagePrivate.getParam("controller_iy", Ki_y);
	n_imagePrivate.getParam("controller_iz", Ki_z);
	n_imagePrivate.getParam("controller_it", Ki_t);
	n_imagePrivate.getParam("verbose_setting", verbose);
	n_imagePrivate.getParam("line_threshold", threshold);
        n_imagePrivate.getParam("control_mode", controlMode);
	n_imagePrivate.getParam("sigma_a", sigma_a);
	n_imagePrivate.getParam("sigma_z", sigma_z);
	n_imagePrivate.getParam("nCount", nCount);
	n_imagePrivate.getParam("Vz", set_speed);
	n_imagePrivate.getParam("zlimit", z_limit);
	n_imagePrivate.getParam("tau_control", tau_control);
	//n_imagePrivate.getParam("ygoal", yInitial);
	// construct all the needed PID controllers
	PID controller_x(Kp_x, Kd_x,Ki_x, -100, 100, -100, 100, "x_controller");
	PID controller_y(Kp_y, Kd_y,Ki_y, -100, 100, -100, 100, "y_controller");
	PID controller_z(Kp_z, Kd_z,Ki_z, -100, 100, -100, 100, "z_controller");
	PID controller_ttc(Kp_t, Kd_t,Ki_t, -100, 100, -100, 100, "ttc_controller");
	ofstream fbLog;
	ofstream ControlLog;
	ofstream imageLog;
	ofstream ttcLog;
	ofstream safeLog;
	ofstream switchLog;
	ControlLog.open("/home/ubuntu/Desktop/control_log.txt");
	fbLog.open("/home/ubuntu/Desktop/fb_log.txt");
	imageLog.open("/home/ubuntu/Desktop/angle_log.txt");
	ttcLog.open("/home/ubuntu/Desktop/ttc.txt");
	safeLog.open("/home/ubuntu/Desktop/safe.txt");
	ros::Time time_last = ros::Time::now();
        float kal_yaw = 0.00;
	x_hat <<0;
	double u_x = 0.0, u_y = 0.0, u_z = 0.0, d_yaw = 0;
	for(int wait = 0; wait < 100; wait++){
		cout << "Waiting for Initial yaw feedback" << endl;
		ros::spinOnce();
		loop_rate.sleep();
	}
	d_yaw = yaw_fb; //keep the yaw angle before move to the start position
	int switch1i = 0, switch2i = 0, switch3i = 0, switch4i = 0, switch5i = 0, switchtaui = 0;
	vector<float> position(3,0);
	for(int i=0; i <nCount;++i) {
		Camera.grab();
		Camera.retrieve(image);
		if(controlMode != takeoff && controlMode != Move2Start)
			Concat.push_back(image.clone());
		
		
		// DO the line detection and error calculation
		Vec4i l_line, r_line;
		if(controlMode == yawEstimate){// the yaw estimation is only conducted in the fist three stages
			LineDetection2(image, threshold, l_line, r_line, 5, verbose);
			position = PositionFb2(l_line, r_line, width, height, 10, verbose);
		}
		
		if(verbose){
		cout <<"Feedback RPY are: "<<roll_fb <<", "<<pitch_fb <<", "<<yaw_fb<<endl;
		cout << "Current x y z: " <<x_fb << ", " << y_fb <<", " <<z_fb <<endl;
		}


		ros::Time time_current = ros::Time::now();
                double dt = (time_current-time_last).toSec();
                time_last = time_current;

		switch(controlMode){
			case takeoff:
				if ((ros::Time::now()-time_takeoff).toSec() >= 4 && abs(x_fb-xGoal)<=0.1 && abs(y_fb-yGoal) <=0.1 && abs(z_fb-zGoal) <= 0.1){
					controlMode = Move2Start;
					time_move = ros::Time::now();
					switch1i = i;
				}  
				cout << "This is control mode " << controlMode << endl;
				break;
			
			case Move2Start:
				xGoal = xInitial;
				yGoal = yInitial;
				zGoal = zInitial;
                                if ((ros::Time::now()-time_move).toSec() >= 4 && abs(x_fb-xGoal)<=0.1 && abs(y_fb-yGoal) <=0.1 && abs(z_fb-zGoal) <= 0.1){
                                        controlMode = yawEstimate;
                                        time_yawEstimate = ros::Time::now();
					switch2i = i;
                                }
                                cout << "This is control mode " << controlMode << endl;
                                cout <<"Desired yaw: " << d_yaw << endl;
                                break;
			
			case yawEstimate:
                                xGoal = xInitial;
                                yGoal = yInitial;
                                zGoal = zInitial;
				if(i == switch2i+1){
                                   x_hat << position[1];
                                   kal_yaw = position[1];
                                   continue;
                                }

                                if(position[2] == 1.0)
                                        Z << position[1];
                                else
                                        Z << kal_yaw;
                                Kalmanf(Z, yaw_dot, dt);
                                //cout << "This is x_hat: " << x_hat;
                                kal_yaw = x_hat(0,0);
                                d_yaw = yaw_fb+3.1415926/2-kal_yaw*3.1415926/180;  // this is the final dyaw we will use for the perch, so we choose 6s to wait for the kalman filter to converge
                                if ((ros::Time::now()-time_yawEstimate).toSec() >= 6 && abs(x_fb-xGoal)<=0.1 && abs(y_fb-yGoal) <=0.1 && abs(z_fb-zGoal) <= 0.1){
                                        controlMode = perchPrepare;
                                        time_perch = ros::Time::now();
					switch3i = i;
                                }
                                cout << "This is control mode " << controlMode << endl;
                                cout <<"Desired yaw: " << d_yaw << endl;
                                break;

			case perchPrepare:
				if(z_fb>=-0.6 && z_fb<=-0.55 && v_zf <= 0.9 && v_zf >= 0.65){
					if(tau_control == 1) {
						controlMode = perchDisCTDTS;
						time_ctdts = ros::Time::now();
					}
					else {
						controlMode = perchDisIPTS;
						time_ipts = ros::Time::now();
					}
					switch4i = i;
				}
				if(v_zf!=0)
					tau0 = (1-z_fb)/(-v_zf);
				z_limit = -0.4;
				if(z_fb >= z_limit)
					controlMode = failSafe;
				break;
			
			case perchDisCTDTS:
				C = 0.4053;
				cur_time = ros::Time::now();
				if(tau>=tauS)
					switched = true;
				if(!switched){
					tau_ref = C*(cur_time-time_ctdts).toSec()+tau0;
					tS = (cur_time-time_ctdts).toSec();
					time_switchtau = ros::Time::now();
					switchtaui = i;
				}
				else{
					tau_ref = tauS+(cur_time-time_ctdts).toSec()-tS;
					if(tau_ref > 0)
						tau_ref =-0.02;
				}
				z_limit = 0.98;
				if(z_fb >= z_limit || v_zf >= 1.6){
					controlMode = failSafe;
					switch5i = i;
					time_final = ros::Time::now();
				}
				break;		

			case perchDisIPTS:
				cur_time = ros::Time::now();
 				
				if(tau >= tauS)
					switched = true;
				if(!switched) {
					double tt = (cur_time-time_ipts).toSec();
					tau_ref = 1/(k0+k1*tt+k2*tt*tt+k3*tt*tt*tt);
					tS = (cur_time-time_ipts).toSec();
					time_switchtau = ros::Time::now();
                                        switchtaui = i;
				}
				/*
				doubele tt = (cur_time-time_ipts).toSec();
				if (tt <= 1.4310) {
					tau_ref = 1/(k0+k1*tt+k2*tt*tt+k3*tt*tt*tt);
                                        time_switchtau = ros::Time::now();
                                        switchtaui = i;	
				}*/
				else{
                                        tau_ref = tauS+(cur_time-time_ipts).toSec()-tS;
                                        if(tau_ref > 0)
                                                tau_ref =-0.02;
                                }
				z_limit = 0.98;
				if(z_fb >= z_limit || v_zf >= 1.5){
                                        controlMode = failSafe;
                                        switch5i = i;
                                        time_final = ros::Time::now();
                                }
                                break; 

			case failSafe:
				cout<<"FailSafe with z_limit = "<<z_limit<<endl;
				double cur_tick = (ros::Time::now()-time_begin).toSec();
                        	safeLog <<cur_tick << ","<< x_fb << "," << y_fb << "," << z_fb<<"," << v_zf <<"," << a_zf <<","<< i << endl;
                        	velmsg.linear.x = 0;
                        	velmsg.linear.y = 0;
                        	velmsg.linear.z = -0.05;
                        	velmsg.angular.x = 0;
                        	velmsg.angular.y = 0;
                        	velmsg.angular.z = 0;
                        	vel_pub.publish(velmsg);
				break;
		
		}
		if(controlMode != failSafe){
			if(v_zf!=0){
				tau = (1-z_fb)/(-v_zf);
			}
			u_x = controller_x.update(x_fb, xGoal);
                	u_y = controller_y.update(y_fb, yGoal);  
			if(controlMode == perchDisCTDTS || controlMode == perchDisIPTS)
				u_z = controller_ttc.update(tau_ref/tau, 1);
			else
				u_z = controller_z.update(z_fb, zGoal);

			vector<float> RPY_desired = control_mapping(u_x, u_y, u_z, d_yaw);
			double cur_tick = (ros::Time::now()-time_begin).toSec();
			fbLog <<cur_tick << ","<< x_fb << "," << y_fb << "," << z_fb<<"," << v_zf <<"," << a_zf <<","<< i << endl;
			ControlLog << cur_tick << ","<< RPY_desired[1] << "," << RPY_desired[2] << "," << RPY_desired[3]<<"," << RPY_desired[0] << endl;
			imageLog << cur_tick << "," << position[0] << ", " << position[1]<< ", " << kal_yaw <<"," << dt << endl;
			if((controlMode == perchDisCTDTS || controlMode == perchDisIPTS)&&i>=switch4i+1)
                        	ttcLog<<cur_tick<<","<<tau_ref<<","<<tau<<","<<i<<endl;
			if(controlMode == perchPrepare){
        	                //RPY_desired[0] = 0.8; //in the prepare stage, use full thrust to accelerate
				velmsg.linear.x = 0;
                                velmsg.linear.y = 0;
                                velmsg.linear.z = 0.8;
                                velmsg.angular.x = 0;
                                velmsg.angular.y = 0;
                                velmsg.angular.z = 0;
                                vel_pub.publish(velmsg);
	                }
			else {
				Desired2Quater(att_msg, RPY_desired);
				att_pub.publish(att_msg);
			}
		}

		ros::spinOnce();
		if(i >= 1000 && v_xf==0 && v_yf==0 )
			break;
	}
	fbLog.close();
	ControlLog.close();
	imageLog.close();
	ttcLog.close();
	safeLog.close();
	cout<<"Stop camera..."<<endl;
	//show time statistics:
	ros::Time time_end = ros::Time::now(); 
	double secondsElapsed = (time_end-time_move).toSec();
	cout<< secondsElapsed<<" seconds for "<< Concat.size()<<"  frames : FPS = "<< float(Concat.size())/float(secondsElapsed)  <<endl;
	//save image
	 
	for (int j = 0; j < Concat.size(); ++j){
		ss<<folderName<<"/"<<name<<j<<type;
		string fullPath = ss.str();
		ss.str("");
		imwrite(fullPath, Concat[j]);
	}
	Camera.release();
	cout<<"Image saved in "<< folderName<<endl;
	cout<<"Switch move i: " <<switch1i << ", " << "Switch time: "<<     (time_move-time_begin).toSec()<<endl;
	cout<<"Switch yawestimate i: " <<switch2i << ", " << "Switch time: "<<     (time_yawEstimate-time_begin).toSec()<<endl;
	cout<<"Switch perchPrepare i: " <<switch3i << ", " << "Switch time: "<<     (time_perch-time_begin).toSec()<<endl;
	if (tau_control == 1)
		cout<<"Switch perchfirst i: " <<switch4i << ", " << "Switch time: "<<     (time_ctdts-time_begin).toSec()<<endl;
	else
		cout<<"Switch perchfirst i: " <<switch4i << ", " << "Switch time: "<<     (time_ipts-time_begin).toSec()<<endl;
	cout<<"Switch perchSecond i: " <<switchtaui << ", " << "Switch time: "<< (time_switchtau-time_begin).toSec()<<endl;
	cout<<"Switch final protection i: " <<switch5i << ", " << "Switch time: "<< (time_final-time_begin).toSec()<<endl;
	switchLog.open("/home/ubuntu/Desktop/switch.txt");
	switchLog << switch1i << "," << (time_move-time_begin).toSec() << endl;
	switchLog << switch2i << "," << (time_yawEstimate-time_begin).toSec() << endl;
	switchLog << switch3i << "," << (time_perch-time_begin).toSec() << endl;
	if (tau_control == 1)
		switchLog << switch4i << "," << (time_ctdts-time_begin).toSec() << endl;
	else
		switchLog << switch4i << "," << (time_ipts-time_begin).toSec() << endl;
	switchLog << switchtaui << "," << (time_switchtau-time_begin).toSec() << endl;
	switchLog << switch5i << "," << (time_final-time_begin).toSec() << endl;
	switchLog.close();
	return 0;
}
