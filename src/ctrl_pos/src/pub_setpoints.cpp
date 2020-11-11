#include <ros/ros.h>
#include <std_msgs/String.h> 
#include <stdio.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"

using namespace std;
double x_fb, y_fb, z_fb;
void mavposeCallback(const geometry_msgs::PoseStamped::ConstPtr& c_pose){   //r$
    x_fb = c_pose->pose.position.x; 
    y_fb = c_pose->pose.position.y;
    z_fb = c_pose->pose.position.z;
//    tf::Quaternion temp(c_pose->pose.orientation.x, c_pose->pose.orientation.y,$
//    tf::Matrix3x3 m(temp);
//    m.getRPY(roll_fb, pitch_fb, yaw_fb);
}

 
int main(int argc, char **argv)
{
   ros::init(argc, argv, "pub_setpoints");
   ros::NodeHandle n;

   ros::Subscriber pose_sub = n.subscribe("/mavros/local_position/pose", 1, &mavposeCallback, ros::TransportHints().tcpNoDelay());
   ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",100);
   ros::Rate loop_rate(100);
   ros::spinOnce();
   
   geometry_msgs::PoseStamped msg;
   int count = 1;
     
        //PositionReciever qp;:
        //Body some_object;
        //qp.connect_to_server();
 
     
   while(ros::ok()){
       //some_object = qp.getStatus();
        // some_object.print();
        //printf("%f\n",some_object.position_x);
       msg.header.stamp = ros::Time::now();
       msg.header.seq=count;
       msg.header.frame_id = 1;
       msg.pose.position.x = 3.7;//0.001*some_object.position_x;
       msg.pose.position.y = 0.0;//0.001*some_object.position_y;
       msg.pose.position.z = -0.9;//0.001*some_object.position_z;
       msg.pose.orientation.x = 0;
       msg.pose.orientation.y = 0;
       msg.pose.orientation.z = 0;
       msg.pose.orientation.w = 1;
       cout << "The current feedback position is: " << x_fb << ", " << y_fb << ", " << z_fb << endl;
       chatter_pub.publish(msg);
       ros::spinOnce();
       count++;
       loop_rate.sleep();
   }
    
       
   return 0;
}
