#include "OpticalFlow.h"

using namespace cv;
//using namespace cv::gpu;
//using namespace cv::cuda;
using namespace pcl;
using namespace std;

void ComputeOpticalFlow(const Mat &past, const Mat &current, const PointCloud<PointXYZRGBA>::ConstPtr &pastCloud, const PointCloud<PointXYZRGBA>::ConstPtr &currCloud, PointCloud<Normal>::Ptr &flow) {
	Mat in1, in2, flow2d;
	cvtColor(past,in1,CV_BGR2GRAY);
	cvtColor(current,in2,CV_BGR2GRAY);
	//opencv dense optical flow method; basing on Gunnar Farneback 's Polynomial Expansion method; VGA may take 200ms/per frame
	calcOpticalFlowFarneback(in1,in2,flow2d,0.5f,2,5,2,7,1.5,0);
	flow->height = flow2d.rows;
	flow->width = flow2d.cols;
	flow->is_dense = false;
	flow->resize(flow->height * flow->width);
	flow->sensor_origin_.setZero();
	PointCloud<Normal>::iterator pOut = flow->begin();
	PointCloud<PointXYZRGBA>::const_iterator pCloud = currCloud->begin();
	Mat_<Vec2f>::iterator pIn = flow2d.begin<Vec2f>();
	int safeWidth = pastCloud->width - 1, safeHeight = pastCloud->height - 1;
	float bad_point = std::numeric_limits<float>::quiet_NaN ();
	for(int j = 0; j < pastCloud->height; j++) {
		for(int i = 0; i < pastCloud->width ; i++) {
			if(pCloud->z == 0 || pCloud->z == bad_point) {
				pOut->normal_x = pOut->normal_y = pOut->normal_z = bad_point;
			} else {
				/*pOut->x = pCloud->x;
				pOut->y = pCloud->y;
				pOut->z = pCloud->z;*/
				pOut->normal_x = (*pIn)[0] * pCloud->z * KINECT_FX_D;
				pOut->normal_y = (*pIn)[1] * pCloud->z * KINECT_FY_D;
				pOut->normal_z = (pCloud->z - (*pastCloud)(Clamp(int(i - (*pIn)[0]),0,safeWidth), Clamp(int(j - (*pIn)[1]),0,safeHeight)).z);
			}
			++pIn; ++pOut; ++pCloud;
		}
	}
}

void ComputeOpticalFlow(const PointCloud<PointXYZRGBA> &pastCloud, const PointCloud<PointXYZRGBA> &currCloud, PointCloud<Normal> *flow) {
	Mat flow2d;
	Mat in1 = Mat(pastCloud.height,pastCloud.width,CV_8UC1);
	Mat in2 = Mat(pastCloud.height,pastCloud.width,CV_8UC1);
	PointCloud<PointXYZRGBA>::const_iterator pPast = pastCloud.begin(), pCloud = currCloud.begin();
	Mat_<char>::iterator pi1 = in1.begin<char>(), pi2 = in2.begin<char>();
	int tmp;
	while(pi1 != in1.end<char>()) {
		tmp = (pPast->b + 6*pPast->g + 3*pPast->r) / 10;
		*pi1 = min(max(tmp, 0), 255);
		tmp = (pCloud->b + 6*pCloud->g + 3*pCloud->r) / 10;
		*pi2 = min(max(tmp, 0), 255);
		++pi1; ++pi2; ++pPast; ++pCloud;
	}
	calcOpticalFlowFarneback(in1,in2,flow2d,0.5f,2,5,2,7,1.5,0);
	flow->height = flow2d.rows;
	flow->width = flow2d.cols;
	flow->is_dense = false;
	flow->resize(flow->height * flow->width);
	flow->sensor_origin_.setZero();
	PointCloud<Normal>::iterator pOut = flow->begin();
	pCloud = currCloud.begin();
	Mat_<Vec2f>::iterator pIn = flow2d.begin<Vec2f>();
	int safeWidth = pastCloud.width - 1, safeHeight = pastCloud.height - 1;
	float bad_point = std::numeric_limits<float>::quiet_NaN ();
	for(int j = 0; j < pastCloud.height; j++) {
		for(int i = 0; i < pastCloud.width ; i++) {
			if(pCloud->z == 0 || pCloud->z == bad_point) {
				pOut->normal_x = pOut->normal_y = pOut->normal_z = bad_point;
			} else {
				/*pOut->x = pCloud->x;
				pOut->y = pCloud->y;
				pOut->z = pCloud->z;*/
				pOut->normal_x = (*pIn)[0] * pCloud->z * KINECT_FX_D;
				pOut->normal_y = (*pIn)[1] * pCloud->z * KINECT_FY_D;
				pOut->normal_z = (pCloud->z - pastCloud(Clamp(int(i - (*pIn)[0]),0,safeWidth), Clamp(int(j - (*pIn)[1]),0,safeHeight)).z);
				/*if(_isnan(pOut->normal_x))
					pOut->normal_x = 0;
				if(_isnan(pOut->normal_y))
					pOut->normal_y = 0;
				if(_isnan(pOut->normal_z))
					pOut->normal_z = 0;*/
			}
			++pIn; ++pOut; ++pCloud;
		}
	}
}

void ComputeOpticalFlowGPU( const PointCloud<PointXYZRGBA>::ConstPtr &pastCloud, const PointCloud<PointXYZRGBA>::ConstPtr &currCloud, PointCloud<Normal>::Ptr &flow) {
	try {
	Mat in1 = Mat(pastCloud->height,pastCloud->width,CV_8UC1);
	Mat in2 = Mat(pastCloud->height,pastCloud->width,CV_8UC1);
	PointCloud<PointXYZRGBA>::const_iterator pPast = pastCloud->begin(), pCloud = currCloud->begin();
	Mat_<char>::iterator pi1 = in1.begin<char>(), pi2 = in2.begin<char>();
	int tmp;
	while(pi1 != in1.end<char>()) {
		tmp = (pPast->b + 6*pPast->g + 3*pPast->r) / 10;
		*pi1 = min(max(tmp, 0), 255);
		tmp = (pCloud->b + 6*pCloud->g + 3*pCloud->r) / 10;
		*pi2 = min(max(tmp, 0), 255);
		++pi1; ++pi2; ++pPast; ++pCloud;
	}
//	Mat In1, In2;
	//cvtColor(in1,In1,CV_BGR2GRAY);
	//cvtColor(in2,In2,CV_BGR2GRAY);
	if(in1.empty()||in2.empty())
	return;
	if(in1.size()!= in2.size())
	{
		printf("n1 != n2\n");
		return;
	}
	cv::cuda::GpuMat d_frameL(in1), d_frameR(in2);
	cv::cuda::GpuMat d_flow(in1.size(), CV_32FC2);
	//Mat in1, in2;
//	Mat d_flowx, d_flowy;
//	cv::gpu::FarnebackOpticalFlow calc_flow;
        cv::Ptr<cv::cuda::DensePyrLKOpticalFlow>calc_flow  = cv::cuda::DensePyrLKOpticalFlow::create(Size(7,7));
       // cv::Ptr<cv::cuda::OpticalFlowDual_TVL1>calc_flow  = cv::cuda::OpticalFlowDual_TVL1::create();
	Mat flowx, flowy;
	cuda::GpuMat planes[2];
	//Mat in1, in2;
	const int64 start = getTickCount();
	calc_flow->calc(d_frameL, d_frameR, d_flow);
const double timeSec = (getTickCount() - start) / getTickFrequency();
cout << "GPU OF : " << timeSec << " sec" << endl;
	cv::cuda::split(d_flow, planes);

	//Mat in1, in2;
	planes[0].download(flowx);
	planes[1].download(flowy);
	//Mat in1, in2;
	//calcOpticalFlowFarneback(in1,in2,flow2d,0.5f,2,5,2,7,1.5,0);
	flow->height = flowx.rows;
	flow->width = flowx.cols;
	flow->is_dense = false;
	flow->resize(flow->height * flow->width);
	flow->sensor_origin_.setZero();
	PointCloud<Normal>::iterator pOut = flow->begin();
	//PointCloud<PointXYZRGBA>::const_iterator pCloud = currCloud->begin();
	Mat_<float>::iterator pInX = flowx.begin<float>();
	Mat_<float>::iterator pInY = flowy.begin<float>();
	int safeWidth = pastCloud->width - 1, safeHeight = pastCloud->height - 1;
	float bad_point = std::numeric_limits<float>::quiet_NaN ();
	for(int j = 0; j < pastCloud->height; j++) {
		for(int i = 0; i < pastCloud->width ; i++) {
			if(pCloud->z == 0 || pCloud->z == bad_point) {
				pOut->normal_x = pOut->normal_y = pOut->normal_z = bad_point;
			} else {
				/*pOut->x = pCloud->x;
				pOut->y = pCloud->y;
				pOut->z = pCloud->z;*/
				pOut->normal_x = *pInX * pCloud->z * KINECT_FX_D;
				pOut->normal_y = *pInY * pCloud->z * KINECT_FY_D;
				pOut->normal_z = (pCloud->z - (*pastCloud)(Clamp(int(i - *pInX),0,safeWidth), Clamp(int(j - *pInY),0,safeHeight)).z);
			}
			++pInX; ++pInY; ++pOut; ++pCloud;
		}
	}
	} catch(cv::Exception &e) {
		cout << e.what() << endl;
	} catch(std::exception &e) {
		cout << e.what() << endl;
	} catch(...) {

	}
}


void Downsample2x2(const Mat &in, Mat &out) { resize(in,out,Size(),0.5f,0.5f); }
