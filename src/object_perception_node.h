#ifndef _OBJECT_PERCEPTION_NODE_H_
#define _OBJECT_PERCEPTION_NODE_H_


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/nonfree/nonfree.hpp>
#include "opencv2/legacy/legacy.hpp"
#include "opencv2/video/tracking.hpp"


#include <cstring>
#include <vector>
#include <fstream>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>


using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;

// = "/home/ir/perception_ws/src/object_perception/data/";

struct eC{
    double err01, err02;
    int pos;
    Mat H;
    bool found;
};


class object_perception
{
	private:
		ros::NodeHandle nh_, nh;

        message_filters::Subscriber<Image> *image_sub;
        message_filters::Subscriber<Image> *depth_sub;
        typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
        Synchronizer<MySyncPolicy> *sync;

        Mat rvec, tvec, cameraM, distC, viconImg, dataH_im, dataH_v;
        int nM, imageNo, failureNo, numPix;
        string markerName;

        string imgFileName, viconFileName, Path, targetFrame;
//        void callback(const ImageConstPtr& image, const ImageConstPtr& depth);
        void callback(const ImageConstPtr& image);//, const ImageConstPtr& depth);
        void extractPoseImage(Mat &src);      
        void getImagePose(Mat &temp);

        std_msgs::Float32MultiArray ptsData;
        ros::Publisher pubPts;

        vector<eC> errorVec;

        tf::TransformListener listener;
        tf::StampedTransform transform, dataBaseTr, newTr, oldTr, tempTransform;

        tf::TransformListener ll;
        tf::StampedTransform tr_C2B;

        Mat handEye;
        typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
        ros::Publisher pubCloud;
        
        vector<tf::Vector3> originalAxis;
		image_transport::ImageTransport it_;
		image_transport::Subscriber image_sub_;
		image_transport::Publisher image_pub_;

        ofstream poseDistFile, matchDistFile;

		vector<int> wsParam;//stc, str, widc, widr;
		vector<Point> contourObj, contourPose, contourHolder;
        void prepareHolderContour(string fileName);
        vector< vector<Point> > contourScenes;
        vector<Point> axis, dataBaseContour, contourCurrent;
        vector<Point2f> dataBaseAxis;
		Mat element, original, H_im, H_v, original_H_im, original_H_v;
		int threshCanny, contourNo;
		double areaTh, areaOri, viconErr;
        bool smallObj, objFound, createDatabase;
        string buf;
        void writeImage(char* buf01, Mat image);
        void checkAndDeleteNegative(vector<Point2f> &obj, vector<Point2f> &scene);
        bool finalCheckTransform(Mat work, vector<Point> contouri);
        void checkContours(vector< vector<Point> > contourFinal, double &v1, double &v2);
        void transformContour(Mat H, vector<Point> contouri, vector<Point> &contourj);

        void estimateNewAxis_small(vector<Point> contouri, vector<Point> &newaxis);

        void evaluateAxis();
		void prepare_contours(Mat src);
		void imageCb(const sensor_msgs::ImageConstPtr& msg);
		void findContourScene(Mat &src);
        void findPoseContour(Mat &src, vector<Point> contouri);
        void findExactContour(Mat &src);
        void prepareImage(Mat src, Mat &work, vector<Point> contouri);

/* new testing methods */
        void findFeature(vector<Point> contour);
        void findPose(vector<Point> contour);
	public:
		object_perception();
		~object_perception();
};


#endif
