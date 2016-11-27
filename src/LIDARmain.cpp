#include "Laser.hpp"
#include "lidar.h"

#include "opencv2/opencv.hpp"

#define _USE_MATH_DEFINES
#include <string.h>
#include <math.h>

#define CM_TO_PIXELS 5
#define LIDAR_SIZE_PIXELS 600

using namespace cv;


float flann_knn(Mat& m_destinations, Mat& m_object, vector<int>& ptpairs, vector<float>& dists) {
    // find nearest neighbors using FLANN
    cv::Mat m_indices(m_object.rows, 1, CV_32S);
    cv::Mat m_dists(m_object.rows, 1, CV_32F);

    Mat dest_32f; m_destinations.convertTo(dest_32f,CV_32FC2);
    Mat obj_32f; m_object.convertTo(obj_32f,CV_32FC2);

    assert(dest_32f.type() == CV_32F);

    cv::flann::Index flann_index(dest_32f, cv::flann::KDTreeIndexParams(2));  // using 2 randomized kdtrees
    flann_index.knnSearch(obj_32f, m_indices, m_dists, 1, cv::flann::SearchParams(64) );

    int* indices_ptr = m_indices.ptr<int>(0);
    //float* dists_ptr = m_dists.ptr<float>(0);
    for (int i=0;i<m_indices.rows;++i) {
        ptpairs.push_back(indices_ptr[i]);
    }

    dists.resize(m_dists.rows);
    m_dists.copyTo(Mat(dists));

    return cv::sum(m_dists)[0];
}

void findBestReansformSVD(Mat& _m, Mat& _d) {
    Mat m; _m.convertTo(m,CV_32F);
    Mat d; _d.convertTo(d,CV_32F);

    Scalar d_bar = mean(d);
    Scalar m_bar = mean(m);
    Mat mc = m - m_bar;
    Mat dc = d - d_bar;

    mc = mc.reshape(1); dc = dc.reshape(1);

    Mat H(2,2,CV_32FC1);
    for(int i=0;i<mc.rows;i++) {
        Mat mci = mc(Range(i,i+1),Range(0,2));
        Mat dci = dc(Range(i,i+1),Range(0,2));
        H = H + mci.t() * dci;
    }

    cv::SVD svd(H);

    Mat R = svd.vt.t() * svd.u.t();
    double det_R = cv::determinant(R);
    if(abs(det_R + 1.0) < 0.0001) {
        double _tmp[4] = {1,0,0,cv::determinant(svd.vt*svd.u)};
        R = svd.u * Mat(2,2,CV_32FC1,_tmp) * svd.vt;
    }
    float* _R = R.ptr<float>(0);
    Scalar T(d_bar[0] - (m_bar[0]*_R[0] + m_bar[1]*_R[1]),d_bar[1] - (m_bar[0]*_R[2] + m_bar[1]*_R[3]));

    m = m.reshape(1);
    m = m * R;
    m = m.reshape(2);
    m = m + T;// + m_bar;
    m.convertTo(_m,CV_32S);
}

int main(int argc, char* argv[]){
	
	std::string serialPort = "dev/tty0";
	LidarSim laser;
	
	int * scanVals = new int[laser.scan_size];
	
	Mat scan(LIDAR_SIZE_PIXELS, LIDAR_SIZE_PIXELS, CV_8UC1, Scalar(0,0,0));
	Mat map(LIDAR_SIZE_PIXELS, LIDAR_SIZE_PIXELS, CV_8UC1, Scalar(0,0,0));
	for(int i = 0; i < laser.scan_size; ++i){
		float angle = i * laser.detection_angle_degrees / laser.scan_size;
		int x = sin(angle*M_PI / 180) * 100;
		int y = cos(angle*M_PI / 180) * 100;
		printf("%f %d %d\n", angle, x, y);
		map.at<uchar>(Point(x + LIDAR_SIZE_PIXELS/2, y + LIDAR_SIZE_PIXELS/2)) = 255;

	}

	cv::imshow("Map", map);
	cv::waitKey(0);

	for(int i = 0; i < 1; i++){
		laser.transferScan(scanVals);

		float stepSize = laser.detection_angle_degrees / laser.scan_size;
		for(int step = 0; step < laser.scan_size; step++){
			float angle = step * stepSize;
			int x = sin(angle*M_PI / 180) * (float)scanVals[step] / 500;
			int y = cos(angle*M_PI / 180) * (float)scanVals[step] / 500;
			printf("%d %d %d %d\n", step, scanVals[step], x, y);

			scan.at<uchar>(Point(x + LIDAR_SIZE_PIXELS/2, y + LIDAR_SIZE_PIXELS/2)) = 255;
		}

		//Find k nearest neighbors
		/*Mat indices(scan.rows, 1, CV_32S);
		Mat distances(scan.rows, 1, CV_32F);
		vector<int> ptpairs;

		Mat map32f, scan32f;
		map.convertTo(map32f, CV_32F);
		scan.convertTo(scan32f, CV_32F);

		flann::Index flann_index(map32f, flann::KDTreeIndexParams(2));
		flann_index.knnSearch(scan32f, indices, distances, 1, flann::SearchParams(64));

		int* indices_ptr = indices.ptr<int>(0);
		for (int i=0;i<indices.rows;++i) {
			ptpairs.push_back(indices_ptr[i]);
		}

		//Compute transform
		Mat scan_mean = scan - mean(scan);
		Mat map_mean = map - mean(map);

		Mat H(2,2,CV_32FC1);
	    for(int i=0;i<scan_mean.rows;i++) {
	        Mat scan_i = scan_mean(Range(i,i+1),Range(0,2));
	        Mat map_i = map_mean(Range(i,i+1),Range(0,2));
	        H = H + scan_i.t() * map_i;
		}*/

		vector<int> ptpairs;
		vector<float> dists;
		double lastDist = 99999999;
		Mat lastGood;
		Mat scanMod = scan;
	    while(true) {
	        ptpairs.clear(); dists.clear();
	        double dist = flann_knn(map, scanMod, ptpairs, dists);

	        cout << "distance: " << dist << endl;

	        if(lastDist <= dist) {
	            scanMod = lastGood;
	            break;  //converged?
	        }
	        lastDist = dist;
	        scanMod.copyTo(lastGood);

	        Mat X_bar(scanMod.size(), scanMod.type());
	        for(int i=0;i<scanMod.rows;i++) {
	            Point p = map.at<Point>(ptpairs[i],0);
	            X_bar.at<Point>(i,0) = p;
	        }

	        //ShowQuery(destination,X,X_bar);

	        scanMod = scanMod.reshape(2);
	        X_bar = X_bar.reshape(2);
	        findBestReansformSVD(scanMod,X_bar);
	        scanMod = scanMod.reshape(1); // back to 1-channel

	        imshow("ICP", scanMod);
	        waitKey(0);
	    }

		imshow("LIDAR", scan);
		waitKey(0);
	}

	delete scanVals;

	return 0;
}
