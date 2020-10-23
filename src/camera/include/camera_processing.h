#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <math.h>
using namespace cv;
using namespace std;

void LineDetection2(const Mat&Im, int threshold, Vec4i &l_line, Vec4i &r_line, int n_lines_used = 5, bool verbose = false){
    	if(Im.empty()){
                cout << "Empty image detected" << endl;
        }
        Mat dst, cdstP;
        Canny(Im, dst, 10, 100, 3);
    // Copy edges to the images that will display the results in BGR
    cvtColor(dst, cdstP, COLOR_GRAY2BGR);
    // Standard Hough Line Transform
    // Probabilistic Line Transform
    vector<Vec4i> linesP; // will hold the results of the detection
    int height = Im.rows;
    int width = Im.cols;
    HoughLinesP(dst, linesP, 1, CV_PI/180, threshold, 50, 10 ); // runs the actual detection
    // Draw the lines
    if(verbose){
        cout << "Size of linesP is "<<linesP.size() <<endl;
    }
     if(linesP.size() == 0){
	l_line[0] = width/2;
    	l_line[1] = 0;
    	l_line[2] = width/2;
    	l_line[3] = height;

    	r_line[0] = width/2;
    	r_line[1] = 0;
    	r_line[2] = width/2;
    	r_line[3] = height;
	cout << "NO line detected!" << endl;
	return;
    } 
    
    for( size_t i = 0; i < linesP.size() && i <n_lines_used; i++ )
    {   
        Vec4i l = linesP[i];
//        std::cout<< i << "th Line: ";
//        std::cout << "First point (" << l[0]<<","<<l[1]<< "), second point ("<< l[2]<<"," <<l[3] << ")"<<std::endl;
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
//        std::cout<< i << "th Line: ";
//        std::cout << "First point (" << l[0]<<","<<l[1]<< "), second point ("<< l[2]<<"," <<l[3] << ")"<<std::endl;
    //    line( cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, LINE_AA);
    }    
//    cout << "The left most point is: "<<min_i <<"th line at: ("<< linesP[min_i][0] <<", " <<linesP[min_i][1] <<") and (" << linesP[min_i][2]<< ", "<<linesP[min_i][3] << endl;
//    cout << "The right most point is: "<<max_i <<"th line at: ("<< linesP[max_i][0] <<", " << linesP[max_i][1] <<") and (" << linesP[max_i][2]<< ", "<<linesP[max_i][3] << endl;  
    
    vector<int> left_lines, right_lines;  // two groups to store the index of the lins in linesP
    left_lines.push_back(min_i);
    right_lines.push_back(max_i);
    int left_x = linesP[min_i][0], left_y = linesP[min_i][1], right_x = linesP[max_i][0], right_y = linesP[max_i][1];
  //  cout << "The two bases are at ("<< left_x << ", " << left_y <<") and (" <<right_x << ", "<<right_y << ")" << endl; 
    for(int i = 0; i < linesP.size() && i < n_lines_used; i++){
	if(i == min_i || i == max_i)
	  continue;
	const Vec4i& l = linesP[i];
	float dist1 = pow(l[0]-left_x, 2)+pow(l[1]-left_y, 2);
        float dist2 = pow(l[0]-right_x, 2)+pow(l[1]-right_y, 2);
	if(dist1 <= dist2){
	  left_lines.push_back(i);
    //      cout << i << " th line is grouped into left group" <<endl;
	}
	else{
	  right_lines.push_back(i);
      //    cout << i << " th line is grouped into right group" << endl;
	}
    }


    // show the gruoped lines and find the mean value for each
    int sum_lx1 = 0, sum_lx2 = 0, sum_ly1 = 0, sum_ly2 = 0, sum_rx1 = 0, sum_rx2 = 0, sum_ry1 = 0, sum_ry2 = 0;
    //cout << "The lines at the left in light blue: " << endl;
    for(int i = 0 ; i < left_lines.size(); i++){
    	//cout << "( " <<linesP[left_lines[i]][0] << ", " << linesP[left_lines[i]][1] <<" )" << endl;
        sum_lx1 += linesP[left_lines[i]][0];
        sum_ly1 += linesP[left_lines[i]][1];
        sum_lx2 += linesP[left_lines[i]][2]; 
	sum_ly2 += linesP[left_lines[i]][3];
  //      line( cdstP, Point(linesP[left_lines[i]][0], linesP[left_lines[i]][1]), Point(linesP[left_lines[i]][2], linesP[left_lines[i]][3]), Scalar(255,255,0), 1, LINE_AA);
    }
   // cout << "The lines at the right in blue: " << endl;
    for(int i = 0 ; i < right_lines.size(); i++){
    //    cout << "( " <<linesP[right_lines[i]][0] << ", " << linesP[right_lines[i]][1] <<" )" << endl;
        sum_rx1 += linesP[right_lines[i]][0];
        sum_ry1 += linesP[right_lines[i]][1];
        sum_rx2 += linesP[right_lines[i]][2];
        sum_ry2 += linesP[right_lines[i]][3];
      //  line( cdstP, Point(linesP[right_lines[i]][0], linesP[right_lines[i]][1]), Point(linesP[right_lines[i]][2], linesP[right_lines[i]][3]), Scalar(255,0,0), 1, LINE_AA);
    }

    
    l_line[0] = sum_lx1/int(left_lines.size());
    l_line[1] = sum_ly1/int(left_lines.size());
    l_line[2] = sum_lx2/int(left_lines.size());
    l_line[3] = sum_ly2/int(left_lines.size());

    r_line[0] = sum_rx1/int(right_lines.size());
    r_line[1] = sum_ry1/int(right_lines.size());
    r_line[2] = sum_rx2/int(right_lines.size());
    r_line[3] = sum_ry2/int(right_lines.size());
    if(verbose){
//    	cout << "The left line is at: (x,y) =  "<<l_line[0] << ", "<< l_line[1]<< " and (x,y)="<< l_line[2] << ", "<<l_line[3]<<endl;
//    	cout << "The right line is at: (x,y) = "<< r_line[0] << ", "<<r_line[1] << " and (x,y) = "<< r_line[2] << ", "<<r_line[3] <<endl;
    }
    return;
}







vector<float> PositionFb2(const Vec4i &l_line, const Vec4i &r_line, const int &width, const int &height, int thres = 10, bool verbose = false){
        vector<float> result (3,0.0);
        int rows = height;
        int cols = width;
        float orientation_d = 90;
        result[0] = cols/2;
        result[1] = orientation_d;
	
	
        // Four points of the detected 2 lines
        int x11 = l_line[0];
        int y11 = rows-l_line[1];
        int x12 = l_line[2];
        int y12 = rows-l_line[3];
        if(verbose)
		cout << "The left group centered at: (" << x11 << ", "<<y11 << ") and (" << x12 << ", " << y12 << ")" << endl;
        int x21 = r_line[0];
  	int y21 = rows-r_line[1];
	int x22 = r_line[2];
	int y22 = rows-r_line[3];
	
	if(verbose)
		cout << "The right group centered at: (" << x21 << ", "<<y21 <<") and (" << x22 << ", "<<y22 << ")"<<endl;
	float k1, k2, k;
	if(x12 != x11 && x21 != x22){
	  
           k1 = (float(y12)-y11)/(float(x12)-x11);
           k2 = (float(y22)-y21)/(float(x22)-x21);
           if(abs(atan(k1)-atan(k2)) < 3.1415926*thres/180){   // 10 degree
	   	k = (k1+k2)/2;
		float new_x = (x11+x21)/2;
		float new_y = (y11+y21)/2;
		float x_fb = (rows/2-new_y)/k+new_x;
		result[0] = x_fb;
		result[1] = atan(k)*180/3.1415926;
                if(result[1] < 0)
			result[1] += 180;
		if(verbose)
			cout << "The current position is: " << x_fb << ", the rod orientation is: "<< result[1]<<endl;
		result[2] = 1.0;
		return result;
	   }
	   else{
		if(verbose)
		cout << "The two lines have large slope difference!"<<endl;
	   }

	}
	else{
	   cout << "Detected perfect vertical lines, no need to adjust orientation"<<endl;
	   result[0] = (x11+x21)/2;  // only try to move it to the mid
	}
        // Transform to coordiante system with (0,0) at the bottom left cornor

        return result;
}
