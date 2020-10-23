#include <iostream>
#include <ctime>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <string.h>
#include "raspicam_cv.h"
//#include "mcp3008Spi.h"
using namespace cv;
using namespace std;
bool doTestSpeedOnly=false;
int findParam ( string param,int argc,char **argv ) {
    int idx=-1;
    for ( int i=0; i<argc && idx==-1; i++ )
        if ( string ( argv[i] ) ==param ) idx=i;
    return idx;

}
//parse command line
//returns the value of a command line param. If not found, defvalue is returned
float getParamVal ( string param,int argc,char **argv,float defvalue=-1 ) {
    int idx=-1;
    for ( int i=0; i<argc && idx==-1; i++ )
        if ( string ( argv[i] ) ==param ) idx=i;
    if ( idx==-1 ) return defvalue;
    else return atof ( argv[  idx+1] );
}

void processCommandLine ( int argc,char **argv,raspicam::RaspiCam_Cv &Camera ) {
    Camera.set ( CV_CAP_PROP_FRAME_WIDTH,  getParamVal ( "-w",argc,argv,192 ) );
    Camera.set ( CV_CAP_PROP_FRAME_HEIGHT, getParamVal ( "-h",argc,argv,144 ) );
    Camera.set ( CV_CAP_PROP_BRIGHTNESS,getParamVal ( "-br",argc,argv,50 ) );			  
    Camera.set ( CV_CAP_PROP_CONTRAST ,getParamVal ( "-co",argc,argv,50 ) );
    Camera.set ( CV_CAP_PROP_SATURATION, getParamVal ( "-sa",argc,argv,50 ) );
    Camera.set ( CV_CAP_PROP_GAIN, getParamVal ( "-g",argc,argv ,50 ) );
    Camera.set ( CV_CAP_PROP_EXPOSURE, getParamVal ( "-ss",argc,argv,50)  );
    if ( findParam ( "-gr",argc,argv ) !=-1 )
        Camera.set ( CV_CAP_PROP_FORMAT, CV_8UC1 );
    if ( findParam ( "-test_speed",argc,argv ) !=-1 )
        doTestSpeedOnly=true;
//    if ( findParam ( "-ss",argc,argv ) !=-1 )
//        Camera.set ( CV_CAP_PROP_EXPOSURE, getParamVal ( "-ss",argc,argv )  );
    if ( findParam ( "-wb_r",argc,argv ) !=-1 )
        Camera.set ( CV_CAP_PROP_WHITE_BALANCE_RED_V,getParamVal ( "-wb_r",argc,argv )     );
    if ( findParam ( "-wb_b",argc,argv ) !=-1 )
        Camera.set ( CV_CAP_PROP_WHITE_BALANCE_BLUE_U,getParamVal ( "-wb_b",argc,argv )     );
}

void showUsage() {
    cout<<"Usage: "<<endl;
    cout<<"[-gr set gray color capture]\n";
    cout<<"[-test_speed use for test speed and no images will be saved]\n";
    cout<<"[-w width] [-h height] \n[-br brightness_val(0,100)]\n";
    cout<<"[-co contrast_val (0 to 100)]\n[-sa saturation_val (0 to 100)]";
    cout<<"[-g gain_val  (0 to 100)]\n";
    cout<<"[-ss shutter_speed (0 to 100) 0 auto]\n";
    cout<<"[-wb_r val  (0 to 100),0 auto: white balance red component]\n";
    cout<<"[-wb_b val  (0 to 100),0 auto: white balance blue component]\n";

    cout<<endl;
}
double compute_ttc(Mat Im1, Mat Im2){
	Mat dst1;
	Mat dst2;
	Mat sIm1;
	Mat sIm2;
	//Gaussian Filter
	for (int i = 1; i < 3; i = i + 2)
	{
		GaussianBlur(Im1, dst1, Size(i, i), 0, 0);
	}
	for (int j = 1; j < 3; j = j + 2)
	{
		GaussianBlur(Im2, dst2, Size(j, j), 0, 0);
	}
	//crop the image
	//if (Num < 44){
	sIm1 = cv::Mat(dst1, cv::Rect(29, 0, 133, 70));//(29,0,133,70)(19,0,60,35)
	sIm2 = cv::Mat(dst2, cv::Rect(29, 0, 133, 70));
	//}
	int Height = sIm1.rows;
	int Width = sIm2.cols;
	Mat smallIm1;
	Mat smallIm2;
	sIm1.convertTo(smallIm1, CV_64F);
	sIm2.convertTo(smallIm2, CV_64F);
        //defined all needed matrix
	Mat Ix(Height - 2, Width - 2, DataType<double>::type);
	Mat Iy(Height - 2, Width - 2, DataType<double>::type);
	Mat It(Height - 2, Width - 2, DataType<double>::type);
	Mat G(Height - 2, Width - 2, DataType<double>::type);
	Mat A(3, 3, DataType<double>::type);
	Mat B(3, 1, DataType<double>::type);
	Mat C(3, 1, DataType<double>::type);
	Mat sumA00; Mat sumA01; Mat sumA02; Mat sumA10; Mat sumA11; Mat sumA12; Mat sumA20; Mat sumA21; Mat sumA22;
	Mat sumB00; Mat sumB10; Mat sumB20;
	int m = Height;
	int n = Width;
	Ix = (smallIm1(Range(2, m), Range(1, n - 1)) - smallIm1(Range(1, m - 1), Range(1, n - 1)) + smallIm1(Range(2, m), Range(2, n)) - smallIm1(Range(1, m - 1), Range(2, n)) + smallIm2(Range(2, m), Range(1, n - 1)) - smallIm2(Range(1, m - 1), Range(1, n - 1)) + smallIm2(Range(2, m), Range(2, n)) - smallIm2(Range(1, m - 1), Range(2, n))) / 4;
	Iy = (smallIm1(Range(1, m - 1), Range(2, n)) - smallIm1(Range(1, m - 1), Range(1, n - 1)) + smallIm1(Range(2, m), Range(2, n)) - smallIm1(Range(2, m), Range(1, n - 1)) + smallIm2(Range(1, m - 1), Range(2, n)) - smallIm2(Range(1, m - 1), Range(1, n - 1)) + smallIm2(Range(2, m), Range(2, n)) - smallIm2(Range(2, m), Range(1, n - 1))) / 4;
	It = (smallIm2(Range(1, m - 1), Range(1, n - 1)) - smallIm1(Range(1, m - 1), Range(1, n - 1)) + smallIm2(Range(2, m), Range(1, n - 1)) - smallIm1(Range(2, m), Range(1, n - 1)) + smallIm2(Range(1, m - 1), Range(2, n)) - smallIm1(Range(1, m - 1), Range(2, n)) + smallIm2(Range(2, m), Range(2, n)) - smallIm1(Range(2, m), Range(2, n))) / 4;
	for (int a = 0; a < Height - 2; a++){
		for (int b = 0; b < Width - 2; b++) {
			G.at<double>(a, b) = (b + 1)*Iy.at<double>(a, b) + (a + 1)*Ix.at<double>(a, b);
		}
	}
	sumA00 = Ix.mul(Ix); sumA01 = Iy.mul(Ix); sumA02 = Ix.mul(G); sumA10 = Ix.mul(Iy); sumA11 = Iy.mul(Iy); sumA12 = G.mul(Iy); sumA20 = Ix.mul(G); sumA21 = Iy.mul(G); sumA22 = G.mul(G);
	sumB00 = Ix.mul(It); sumB10 = Iy.mul(It); sumB20 = G.mul(It);
	A.at<double>(0, 0) = sum(sumA00)[0]; A.at<double>(0, 1) = sum(sumA01)[0]; A.at<double>(0, 2) = sum(sumA02)[0];
	A.at<double>(1, 0) = sum(sumA10)[0]; A.at<double>(1, 1) = sum(sumA11)[0]; A.at<double>(1, 2) = sum(sumA12)[0];
	A.at<double>(2, 0) = sum(sumA20)[0]; A.at<double>(2, 1) = sum(sumA21)[0]; A.at<double>(2, 2) = sum(sumA22)[0];
	B.at<double>(0, 0) = sum(sumB00)[0];
	B.at<double>(1, 0) = sum(sumB10)[0];
	B.at<double>(2, 0) = sum(sumB20)[0];
	C = -A.inv()*B;
	return (1 / C.at<double>(2, 0)/33/1.5);
}
int compute_velocity(double t, int v, double tp, float t0, float distance){
        //t is feedback ttc, v is velocity, tp is time t from t0, t0 is tau0
        float  k1=-0.4924;
        float  k2=0.0861;
	float  c = 0.4714;
        int reference=1; //1= optimization  0= constant tau dot
        double timetocontact; 
        if (reference==1) 
	 {
          timetocontact=1/(k2*tp*tp+k1*tp+1/t0);
	 }
        else
         {
         timetocontact=c*tp+t0;
         }
        printf("tau_ref: %f\n", timetocontact);
	timetocontact = abs(timetocontact);
        int new_speed;
        float kp;
        double error=t-timetocontact;
        kp=13/distance;// or adaptive controller    13
        new_speed=v+kp*error;
        if (new_speed>180){new_speed=180;}
        if (new_speed<=0){new_speed=0;}
	return new_speed;
}
/*float anaread(void)
{
	float distance;
	mcp3008Spi a2d("/dev/spidev0.0", SPI_MODE_0, 1000000, 8);
	//int i = 20;
	int a2dVal = 0;
	int a2dChannel = 0;
	unsigned char data[3];

	//while (i > 0)
	//{
		data[0] = 1;  //  first byte transmitted -> start bit
		data[1] = 0b10000000 | (((a2dChannel & 7) << 4)); // second byte transmitted -> (SGL/DIF = 1, D2=D1=D0=0)
		data[2] = 0; // third byte transmitted....don't care

		a2d.spiWriteRead(data, sizeof(data));

		a2dVal = 0;
		a2dVal = (data[1] << 8) & 0b1100000000; //merge data[1] & data[2] to get result
		a2dVal |= (data[2] & 0xff);
		if ((a2dVal >= 0) && (a2dVal < 70))    { distance = a2dVal*5.2 / 1000; }
		else if ((a2dVal >= 70) && (a2dVal <150 )){ distance = a2dVal*4.4 / 1000; }
		else if ((a2dVal >= 150) && (a2dVal <320)){ distance = a2dVal*4.0 / 1000; }
		else if ((a2dVal >= 320) && (a2dVal <620)){ distance = a2dVal*3.9 / 1000; }
		else  { distance = a2dVal*3.8 / 1000; }
		cout << "distance: " << distance << endl;
                return distance;
}*/

