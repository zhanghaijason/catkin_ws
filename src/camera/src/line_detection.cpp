#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <ros/ros.h>
#include "camera_processing.h"
using namespace cv;
using namespace std;
int main(int argc, char **argv)
{
//    ros::init(argc, argv, "line_detection");
 //   ros::NodeHandle nh_line;
 //   ros::NodeHandle nh_line_Private("~");

    int threshold;

//    nh_line_Private.getParam("line_threshold", threshold);
    // Declare the output variables
    Mat dst, cdstP;
    string filename = "image10.jpg";
    Mat src = imread( filename, IMREAD_GRAYSCALE );
    if(src.empty()){
        printf(" Error opening image\n");
        //printf(" Program Arguments: [image_name -- default %s] \n", default_file);
        return -1;
    }
    // Edge detection
    Canny(src, dst, 10, 100, 3);
    // Copy edges to the images that will display the results in BGR
    cvtColor(dst, cdstP, COLOR_GRAY2BGR);
    // Standard Hough Line Transform
    // Probabilistic Line Transform
    vector<Vec4i> linesP; // will hold the results of the detection
    int height = src.rows;
    int width = src.cols;
    HoughLinesP(dst, linesP, 1, CV_PI/180, 45, 50, 10 ); // runs the actual detection
    // Draw the lines
    cout << "Size of linesP is "<<linesP.size() <<endl;
    int n_lines_used = 5;

    
    for( size_t i = 0; i < linesP.size() && i <n_lines_used; i++ )
    {   
        Vec4i l = linesP[i];
        std::cout<< i << "th Line: ";
        std::cout << "First point (" << l[0]<<","<<l[1]<< "), second point ("<< l[2]<<"," <<l[3] << ")"<<std::endl;
     //   line( cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,255,0), 1, LINE_AA);
    }
	     
    //Extend the lines
    int x_intercept, y_intercept;
    for(size_t i= 0; i < linesP.size() && i < n_lines_used; ++i){
	Vec4i & line = linesP[i];
	int x11 = line[0];
	int y11 = line[1];
	int x12 = line[2];
	int y12 = line[3];
	if(x11 == x12){
	    x_intercept = x11;
	    line[0] = x_intercept;
	    line[1] = 0;
	    line[2] = x_intercept;
            line[3] = height;
	}
	else{
  	    float slope = float((y12-y11))/float((x12-x11));
	    x_intercept = x11-float(y11)/slope;
	    y_intercept = y11-slope*float(x11);
	    if(x_intercept <= float(height-y11)/slope+x11){
	      line[0] = x_intercept;
	      line[1] = 0;
              line[2] = float(height-y11)/slope+x11;
	      line[3] = height;
	    }
	    else{
	      line[0] = float(height-y11)/slope+x11;
	      line[1] = height;
              line[2] = x_intercept;
              line[3] = 0;
            }
	}
    }
    
    int min_x = linesP[0][0], min_i = 0, max_x =linesP[0][0], max_i = 0;  // find the leftmost and right most points (starting with sammeller x), use them as the base of two groups of lines
    for( size_t i = 0; i < linesP.size() && i < n_lines_used; i++ )
    {   
        const Vec4i& l = linesP[i];
	if(l[0] > max_x){   //only use the endpoint at left to compare
	  max_i = i;
	  max_x = l[0];
	}
	if(l[0] < min_x){
	  min_i = i;
	  min_x = l[0];
	}
        std::cout<< i << "th Line: ";
        std::cout << "First point (" << l[0]<<","<<l[1]<< "), second point ("<< l[2]<<"," <<l[3] << ")"<<std::endl;
    //    line( cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, LINE_AA);
    }    
    cout << "The left most point is: "<<min_i <<"th line at: ("<< linesP[min_i][0] <<", " <<linesP[min_i][1] <<") and (" << linesP[min_i][2]<< ", "<<linesP[min_i][3] << endl;
    cout << "The right most point is: "<<max_i <<"th line at: ("<< linesP[max_i][0] <<", " << linesP[max_i][1] <<") and (" << linesP[max_i][2]<< ", "<<linesP[max_i][3] << endl;  
    
    vector<int> left_lines, right_lines;  // two groups to store the index of the lins in linesP
    left_lines.push_back(min_i);
    right_lines.push_back(max_i);
    int left_x = linesP[min_i][0], left_y = linesP[min_i][1], right_x = linesP[max_i][0], right_y = linesP[max_i][1];
    cout << "The two bases are at ("<< left_x << ", " << left_y <<") and (" <<right_x << ", "<<right_y << ")" << endl; 
    for(int i = 0; i < linesP.size() && i < n_lines_used; i++){
	if(i == min_i || i == max_i)
	  continue;
	const Vec4i& l = linesP[i];
	float dist1 = pow(l[0]-left_x, 2)+pow(l[1]-left_y, 2);
        float dist2 = pow(l[0]-right_x, 2)+pow(l[1]-right_y, 2);
	if(dist1 <= dist2){
	  left_lines.push_back(i);
          cout << i << " th line is grouped into left group" <<endl;
	}
	else{
	  right_lines.push_back(i);
          cout << i << " th line is grouped into right group" << endl;
	}
    }


    // show the gruoped lines and find the mean value for each
    int sum_lx1 = 0, sum_lx2 = 0, sum_ly1 = 0, sum_ly2 = 0, sum_rx1 = 0, sum_rx2 = 0, sum_ry1 = 0, sum_ry2 = 0;
    cout << "The lines at the left in light blue: " << endl;
    for(int i = 0 ; i < left_lines.size(); i++){
    	cout << "( " <<linesP[left_lines[i]][0] << ", " << linesP[left_lines[i]][1] <<" )" << endl;
        sum_lx1 += linesP[left_lines[i]][0];
        sum_ly1 += linesP[left_lines[i]][1];
        sum_lx2 += linesP[left_lines[i]][2]; 
	sum_ly2 += linesP[left_lines[i]][3];
        line( cdstP, Point(linesP[left_lines[i]][0], linesP[left_lines[i]][1]), Point(linesP[left_lines[i]][2], linesP[left_lines[i]][3]), Scalar(255,255,0), 1, LINE_AA);
    }
    cout << "The lines at the right in blue: " << endl;
    for(int i = 0 ; i < right_lines.size(); i++){
        cout << "( " <<linesP[right_lines[i]][0] << ", " << linesP[right_lines[i]][1] <<" )" << endl;
        sum_rx1 += linesP[right_lines[i]][0];
        sum_ry1 += linesP[right_lines[i]][1];
        sum_rx2 += linesP[right_lines[i]][2];
        sum_ry2 += linesP[right_lines[i]][3];
        line( cdstP, Point(linesP[right_lines[i]][0], linesP[right_lines[i]][1]), Point(linesP[right_lines[i]][2], linesP[right_lines[i]][3]), Scalar(255,0,0), 1, LINE_AA);
    }

    Vec4i l_line = linesP[0], r_line = linesP[0];
    l_line[0] = sum_lx1/int(left_lines.size());
    l_line[1] = sum_ly1/int(left_lines.size());
    l_line[2] = sum_lx2/int(left_lines.size());
    l_line[3] = sum_ly2/int(left_lines.size());

    r_line[0] = sum_rx1/int(right_lines.size());
    r_line[1] = sum_ry1/int(right_lines.size());
    r_line[2] = sum_rx2/int(right_lines.size());
    r_line[3] = sum_ry2/int(right_lines.size());
    
    cout << "sum lx1 and l left_lines size: " << sum_lx1 << ", " << left_lines.size() << endl;
    // draw the two mean lines from the two groups
    line( cdstP, Point(l_line[0], l_line[1]), Point(l_line[2], l_line[3]), Scalar(0,0,255), 1, LINE_AA);
    line( cdstP, Point(r_line[0], r_line[1]), Point(r_line[2], r_line[3]), Scalar(0,0,255), 1, LINE_AA);
    cout << "The two final lines are in red"<<endl;
    cout << "x, y = " << l_line[0] << ", " << l_line[1] << " and x,y = "<< l_line[2]<<", "<< l_line[3]<<endl;
    cout << "x, y = " << r_line[0] << ", " << r_line[1] << " and x,y = "<< r_line[2] << ", "<< r_line[3] << endl;
    vector<float> error = PositionFb2(l_line, r_line, width, height);
    cout<<"The detected yaw error is: "<<error[1]<<"."<<endl;
    // Show results
    imshow("Source", src);
    imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP);
    // Wait and Exit
    waitKey();
    return 0;
}
