/**********************************************************************************
2018/12/18/ OpenCV for human body solhouette  Tianwei@YNL
**********************************************************************************/

#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/core/core.hpp>
#include<stdio.h>
#include<iostream>

using namespace cv;
using namespace std;

static string filename = "0_pose.xml";
//...

int main (int argv, char **argc)
{
	/////////////read your jpg image/////////////
	Mat Img(480, 640, CV_8UC3);
	Img = imread("0.jpg", IMREAD_COLOR);

	////////read your labelled joints from xml ///////////////////
	FileStorage fs(filename, FileStorage::READ);
	fs.open(filename, FileStorage::READ);

	//Mat Pose(1,18,3) I dont no why he use dim 3, size 1x18x3 channel 1;
	Mat Pose;
	fs["pose_0"]>>Pose;
	//cout<<"pose dims  is " << Pose.dims <<" size "<< Pose.size <<" channels " <<Pose.channels() <<endl;
	fs.release();

	Vec<Point, 18> Joints;
	Matx <int, 18, 2> J;
	for(int i = 0; i < 18; i++)
	{
		J(i,0) =	Joints[i].x = int(Pose.at<float>(0,i,0));
		J(i,1) =	Joints[i].y = int(Pose.at<float>(0,i,1));
		////////drow circles on the joint points////
		circle(Img, Joints[i], 3, 255, -1);
	}
	vector<int> array1,array2;
	for(int i = 0; i<Joints.rows; i++){
		if(Joints[i].x != 0) 
		{
			array1.push_back(Joints[i].x);
			array2.push_back(Joints[i].y);
		}
	}
	///////drow lines 
	line(Img, Joints[0], Joints[1], Scalar(0,255,0), 2);//link from joint 0 to 1
	line(Img, Joints[1], Joints[2], Scalar(0,255,0), 2);//link from joint 1 to 2
	line(Img, Joints[2], Joints[3], Scalar(0,255,0), 2);
	line(Img, Joints[3], Joints[4], Scalar(0,255,0), 2);

	////add more links here .........


	////drow regt_//////
	cout << " \n " << J <<endl;
	Point max_px, min_px, max_py, min_py;
	double min_x, max_x, min_y, max_y;
	minMaxLoc(array1, &min_x, &max_x);
	minMaxLoc(array2, &min_y, &max_y);
	cout<< min_x <<" "<< max_x <<" "<< min_y <<" "<<max_y <<" "<<endl;

	Rect rec(Point(min_x-20, min_y-50), Point(max_x+20, max_y+50));
	rectangle(Img, rec, Scalar(0, 0, 255), 2);

	//////////////save img to a ppm/jpg/png file for further processing/////////
	vector<int> para;
	para.push_back(CV_IMWRITE_PXM_BINARY);
	para.push_back(1);

	try{
		imwrite("0.ppm", Img, para);
	}
	catch (runtime_error& ex){
		cout<<" runtime error "<<  ex.what()<<endl;
		return 1;
	}

	return 0;
}
