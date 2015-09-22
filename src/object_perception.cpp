#include "object_perception_node.h"

template<typename T>
T getParam(ros::NodeHandle& n, const std::string& name, const T& defaultValue)
{
        T v;
        if (n.getParam(name, v))
        {
                ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
                return v;
        }
        else
                ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
        return defaultValue;
}


object_perception::object_perception()
	: it_(nh_),
	nh("~")
{
    createDatabase = getParam<bool> (nh,"createData",false);
    imageNo = 0;
    failureNo = 0;
	string input_topic = getParam<string> (nh,"input_topic","/kinect2/rgb_rect/image");
	string depth_topic = getParam<string> (nh,"depth_topic","/kinect2/depth_rect/image");
	threshCanny = 50;

	image_sub_ = it_.subscribe (input_topic, 1, &object_perception::callback, this);
#if 0
    image_sub = new message_filters::Subscriber<Image> (nh,input_topic,2);
    depth_sub = new message_filters::Subscriber<Image> (nh,depth_topic,2);
    sync = new Synchronizer<MySyncPolicy> (MySyncPolicy(2), *image_sub, *depth_sub);
    sync->registerCallback(boost::bind(&object_perception::callback, this, _1, _2));
#endif
    string cameraFile = getParam<string> (nh, "camera_calib_file","/home/ir/kinect2WS/src/iai_kinect2/kinect2_bridge/data/507554242542/calib_color.yaml");
    FileStorage fs2(cameraFile.c_str(), FileStorage::READ);
    fs2["cameraMatrix"]>>cameraM;
    fs2["distortionCoefficients"]>>distC;
    fs2.release();
    imageNo = 0;

	string output_topic = getParam<string> (nh,"output_topic","/camera/image/objectPose");
	image_pub_ = it_.advertise (output_topic, 1);
	string output_topic_transform = getParam<string> (nh,"output_topic_transform","/camera/image/test");
	//image_pub_02 = it_.advertise (output_topic_transform, 1);
	areaTh = getParam<double> (nh,"threshold_for_homography",0.3);
	char paramStr[100] = "param%d";
    char parambuffer[100]="";
	for(int i=0;i<4;i++){
		sprintf(parambuffer,paramStr,i+1);
		int temp = getParam<int> (nh,parambuffer,0);
		wsParam.push_back(temp);
	}

	element = getStructuringElement (MORPH_RECT, Size(2*3+1,2*3+1), Point(3,3));
	string orig_image = getParam<string> (nh,"original_image","/home/ir/perception_ws/src/object_perception/data/obj02.png");
    string contour_orig = getParam<string> (nh,"contour_image","/home/ir/perception_ws/src/object_perception/data/contour02.png");
	original = imread(orig_image,CV_LOAD_IMAGE_COLOR);
    Mat contour_img = imread(contour_orig,CV_LOAD_IMAGE_COLOR);
    cout<<contour_img.channels()<<" "<<contour_img.rows<<endl;
    smallObj = getParam<bool> (nh,"smallObj",false);
	if (!original.data){
		ROS_ERROR("hello!!");
	}
	prepare_contours(contour_img);
    evaluateAxis();
    if(!smallObj){
        string objHolder = getParam<string> (nh, "axisPoints", "/home/ir/perception_ws/src/object_perception/data/kinect2_objHolder.png");
        prepareHolderContour(objHolder);
    }
    buf = getParam<string> (nh,"path","/home/ir/perception_ws/src/object_perception/data/");

    pubPts = nh.advertise<std_msgs::Float32MultiArray> ("points",10);
    std_msgs::MultiArrayDimension dim0;//("row",3,3*3);
    dim0.label = "row";
    dim0.size = 3;
    dim0.stride = 3*3;
    std_msgs::MultiArrayDimension dim1;//("col",3,3);
    dim1.label = "col";
    dim1.size = 3;
    dim1.stride = 3;
    ptsData.layout.dim.push_back(dim0);
    ptsData.layout.dim.push_back(dim1);
    for(int i=0;i<dim0.stride;i++){
        ptsData.data.push_back(0.0);
    }
    string hand_eye_calib_path = "/home/ir/perception_ws/src/hand_eye_calib/src/hand_eye_calib.txt";
    ifstream fpHand;
    fpHand.open(hand_eye_calib_path.c_str());
//    handEye = Mat::zeros(4,4,cameraM.type());
//    for(int i=0;i<4;i++)
    pubCloud = nh.advertise<PointCloud> ("objectPoints",10); 

    ll.waitForTransform("robot_base",  "end_effector", ros::Time::now(), ros::Duration(0));
    try{
        ll.lookupTransform("robot_base",  "end_effector", ros::Time(0), tr_C2B);
    }
    catch (tf::TransformException ex){
//      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }  

	ROS_INFO_STREAM("Initialisation is complete");
}

void object_perception::prepareHolderContour(string fileName){
    Mat image = imread(fileName.c_str(), CV_LOAD_IMAGE_COLOR);
    vector<Mat> chs;
    split(image,chs);
    for(int i=0;i<3;i++){
        Point temp;
        contourHolder.push_back(temp);
    }
    for(int i=0;i<3;i++){
        vector<Point2i> locations;
        findNonZero(chs[i],locations);
        contourHolder[i].x = locations[0].x;
        contourHolder[i].y = locations[0].y;
    }
#if 0
    Mat test = original.clone();
    line(test,contourHolder[2],contourHolder[0],Scalar(0,0,255),4,8,0);
    line(test,contourHolder[2],contourHolder[1],Scalar(0,255,0),4,8,0);
    imshow("test",test);
    waitKey(-1);
#endif
}

object_perception::~object_perception()
{
//	fp_.close();
//	fp_n.close();
}

void object_perception::prepare_contours(Mat src){
	Mat gray, cannyO;
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	cvtColor(src,gray, CV_BGR2GRAY);
	int rows = gray.rows;
	int cols = gray.cols;
#if 0
	Mat temp = Mat::zeros(original.size(),gray.type());
	gray(Rect(wsParam[0],wsParam[1],wsParam[2],wsParam[3])).copyTo(temp(Rect(wsParam[0],wsParam[1],wsParam[2],wsParam[3])));
	temp.copyTo(gray);
#endif
	Mat morphTemp;
	dilate(gray,morphTemp,element);
	erode(morphTemp,gray,element);
	Canny(gray,cannyO,threshCanny, 2*threshCanny, 3);
	findContours(cannyO,contours,hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

    double maxA = -1;
    size_t pos = 0;
    for(size_t i=0;i<contours.size();i++){
        if(maxA<contourArea(contours[i])){
            maxA = contourArea(contours[i]);
            contourObj = contours[i];
            pos = i;
        }
    }
    Mat tempPix = Mat::zeros(gray.size(),CV_8UC3);

    drawContours(tempPix,contours,pos,Scalar(0,0,255),CV_FILLED);
    vector<Mat> chs;
    split(tempPix,chs);
    numPix = countNonZero(chs[2]);
    ROS_INFO_STREAM("original number of pixels: "<<numPix);

//	contourObj = contours[0];
#if 0
    drawContours(original,contours,pos,Scalar(0,0,255),CV_FILLED);
    imshow("test",original);
    waitKey(0);
#endif
}

void printTfVector(tf::Vector3 x){
    cout<<x.getX()<<" "<<x.getY()<<" "<<x.getZ()<<endl;
}

void object_perception::evaluateAxis(){
    axis.clear();
    if(!smallObj){
        Mat contourImage = Mat::zeros(original.rows,original.cols,CV_8UC3);
        vector< vector<Point> > testContour;
        testContour.push_back(contourObj);
        Scalar color = Scalar(0,0,255);
        drawContours(contourImage,testContour,0,color,CV_FILLED);
        vector<Mat> channels;
        split(contourImage,channels);
        Mat nonzerocoord;
        findNonZero(channels[2],nonzerocoord);
        Mat data_pts = Mat(nonzerocoord.total(), 2, CV_64FC1);
        for (int i = 0; i < data_pts.rows; ++i)
        {
            data_pts.at<double>(i, 0) = nonzerocoord.at<Point>(i).x;
            data_pts.at<double>(i, 1) = nonzerocoord.at<Point>(i).y;
        }
        PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);

        Point pos = Point(pca_analysis.mean.at<double>(0, 0),
                      pca_analysis.mean.at<double>(0, 1));
        vector<Point> eigen_vecs(2);
        vector<double> eigen_val(2);
        for (int i = 0; i < 2; ++i)
        {
            eigen_vecs[i] = Point(30*pca_analysis.eigenvectors.at<double>(i, 0),
                                30*pca_analysis.eigenvectors.at<double>(i, 1));

            eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
        }
        Point a = pos + eigen_vecs[0];
        Point b = pos + eigen_vecs[1];
        ROS_INFO_STREAM(pos<<" "<<a<<" "<<b);
        axis.push_back(pos);
        axis.push_back(a);
        axis.push_back(b);
    }
    else{
        for(size_t i=0;i<3;i++){
            Point a;
            axis.push_back(a);
        }
        estimateNewAxis_small(contourObj,axis);
    }
}

void object_perception::getImagePose(Mat &tempH_im){
    vector<Point> cameraAxis;
    Point a(1,0), b(0,1), c(0,0);
    cameraAxis.push_back(a);
    cameraAxis.push_back(b);
    cameraAxis.push_back(c);
    try{
        tempH_im = estimateRigidTransform(cameraAxis,axis,1);
    }
    catch(cv::Exception &e){
        ROS_ERROR_STREAM("From image transform estimation. TRY: vim object_perception.cpp +359");
    }
}

void object_perception::writeImage(char* buf01, Mat image){
//    char buf01[100] = "image%d.png";
    char buf02[100] = "";
    strcat(buf02,buf.c_str());
    strcat(buf02,buf01);
    char imageN[100]=" ";
    sprintf(imageN,buf02,imageNo);
//    ROS_INFO_STREAM("writing image: "<<imageN);
    imwrite(imageN,image);

}
