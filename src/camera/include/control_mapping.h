#include<iostream>
#include<vector>
#include<algorithm>
using namespace std;
/// the equation in to map desired control input to three angles and thrust

vector<float> control_mapping(float u_x, float u_y, float u_z, float target_psi = 0.0){             
	float g=9.81;
	float target_phi=asin((u_x*sin(target_psi)-u_y*cos(target_psi))/sqrt((u_x*u_x+u_y*u_y+(u_z+g)*(u_z+g))));         //target roll
	float target_theta=atan((u_x*cos(target_psi)+u_y*sin(target_psi))/(g+u_z));   //target pitch
	float a1=sin(target_theta)*cos(target_psi)*cos(target_phi)+sin(target_psi)*sin(target_phi);             // 
	float a2=sin(target_theta)*sin(target_psi)*cos(target_phi)-cos(target_psi)*sin(target_phi);             // 
	float a3=cos(target_theta)*cos(target_phi);             //
	float u=(u_x*a1+u_y*a2+(u_z+g)*a3);
    	//float phi=target_phi*180/3.14159;
    	//float theta=target_theta*180/3.14159;
	vector<float> res(4,0);
	res[0] = std::max(0.0, std::min(1.0, u/g*0.5+0.05)); //since the quad is too heavy, make the default hovering thrust as 0.6
	res[1] = std::max(float(-30.0/180*3.14159), std::min(float(30.0/180*3.14159), target_phi));
	res[2] = std::max(float(-30.0/180*3.14159), std::min(float(30.0/180*3.14159), target_theta));
	res[3] = target_psi;
	return res;
}


