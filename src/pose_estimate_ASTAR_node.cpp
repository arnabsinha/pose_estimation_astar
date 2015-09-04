#include "object_perception_node.h"


# if 0
void object_perception::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		int rows = cv_ptr->image.rows;
		int cols = cv_ptr->image.cols;
        if(!createDatabase){
        findContourScene(cv_ptr->image);
        for(size_t i=0;i<contourScenes.size();i++){
            contourNo = i;
#if 0        
            objFound = false;
            if(find_feature)
                findFeature(contourScenes[i]);
            else
                findPose(contourScenes[i]);
#endif
#if 1
            findPoseContour(cv_ptr->image,contourScenes[i]);
            if(objFound){
                vector< vector<Point> > contourFinal;
                contourFinal.push_back(contourObj);
//                contourFinal.push_back(contourScenes[i]);
                contourFinal.push_back(axis);
                transformContour(H_im, contourObj, contourFinal[0]);
                transformContour(H_im, axis, contourFinal[1]);
//                drawContours(cv_ptr->image,contourFinal,1,Scalar(255,0,0),4,8,0,0,Point());
                if(!smallObj){
                    drawContours(cv_ptr->image,contourFinal,0,Scalar(0,0,255),4,8,0,0,Point());
                    line(cv_ptr->image,contourFinal[1][0],contourFinal[1][1],Scalar(0,0,255),4,8,0);
                    line(cv_ptr->image,contourFinal[1][0],contourFinal[1][2],Scalar(255,0,0),4,8,0);
                }
                else{
                    drawContours(cv_ptr->image,contourFinal,0,Scalar(0,0,255),1,4,0,0,Point());
                    line(cv_ptr->image,contourFinal[1][0],contourFinal[1][1],Scalar(0,255,0),4,8,0);
                    line(cv_ptr->image,contourFinal[1][0],contourFinal[1][2],Scalar(255,0,0),4,8,0);
                }
//            transform(original,cv_ptr->image,H_im);
            
            }
#endif
        }
        }
        else{
            char buf01[100]="abcd%d.png";
            writeImage(buf01,cv_ptr->image);
        }
        imageNo++;
#if 0
        contourScenes.push_back(contourObj);
        for(size_t i=0;i<contourScenes.size()-1;i++)
            drawContours(cv_ptr->image,contourScenes,i,Scalar(255,0,0),4,8,0,0,Point());
        drawContours(cv_ptr->image,contourScenes,contourScenes.size()-1,Scalar(0,0,255),4,8,0,0,Point());
#endif        
//        findExactContour(cv_ptr->image);
		//findTransformOriginal(cv_ptr->image, contour02);
    

/* TO DO */
/* extract contours according to former approach, and estimate contour w.r.t. new surf-feature based homography transformation -- next find a statistical or boolean operator to take a final decision based upon these two approaches */
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	image_pub_.publish(cv_ptr->toImageMsg());
}

#endif
void object_perception::findContourScene (Mat &src){
	Mat gray, cannyO;
	vector< vector<Point> > contours;
	vector<double> szC;
	vector<int> final_contours;
	vector<Vec4i> hierarchy;
	cvtColor(src,gray, CV_BGR2GRAY);
	int rows = gray.rows;
	int cols = gray.cols;
	Mat temp = Mat::zeros(src.size(),gray.type());
	gray(Rect(wsParam[0],wsParam[1],wsParam[2],wsParam[3])).copyTo(temp(Rect(wsParam[0],wsParam[1],wsParam[2],wsParam[3])));
	temp.copyTo(gray);
	
	Mat morphTemp;
	dilate(gray,morphTemp,element);
	erode(morphTemp,gray,element);

	Canny(gray,cannyO,threshCanny, 2*threshCanny, 3);
	findContours(cannyO,contours,hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

	double tMatch = 100;
	vector< vector<Point> > transCon;
	double areaMatch, act_area_th = 0.8;
    contourScenes.clear();
    contours.push_back(contourObj);
    Mat imgTemp = Mat::zeros(src.size(),src.type());
    drawContours(imgTemp,contours,contours.size()-1,Scalar(255,255,255),CV_FILLED);
    vector<Mat> s;
    split(imgTemp,s);
    areaOri = countNonZero(s[2]);
    Mat srcTest = src.clone();
	for(int i=0;i<contours.size()-1;i++){
        imgTemp =  Mat::zeros(src.size(),src.type());
        drawContours(imgTemp,contours,i,Scalar(255,255,255),CV_FILLED);
        split(imgTemp,s);
        areaMatch = countNonZero(s[2])/areaOri;
//		areaMatch = fabs(contourArea(contourObj)-contourArea(contours[i]));
		try{
			tMatch = matchShapes(contours[i],contourObj,CV_CONTOURS_MATCH_I1,0);
		}
		catch (cv::Exception& e){
			ROS_ERROR("cv exception: %s", e.what());
                	return;
		}
		if(tMatch<0.4 && areaMatch<1.1 && areaMatch>act_area_th){
//            ROS_INFO_STREAM(areaOri<<" "<<i<<" "<<countNonZero(s[2])<<" "<<areaMatch);
		    drawContours(srcTest,contours,i,Scalar(0,255,0),4,8,0,0,Point());
			contourScenes.push_back(contours[i]);
		}
	}
#if 0
	imshow("test",srcTest);
    waitKey(1);
#endif
}

void object_perception::findExactContour(Mat &src){
    /* TODO correspond with pose estimation and contourestimation */
    for (size_t i=0;i<contourScenes.size();i++){
	    drawContours(src,contourScenes,i,Scalar(255,0,0));        
    }

}
#if 0

void object_perception::getOrientation(vector<Point> &pts, Mat &img, Mat &dir, vector<double> &meanVec){
	Mat contourImage = Mat::zeros(img.rows,img.cols,CV_8UC3);
	vector< vector<Point> > testContour;
	testContour.push_back(pts);
	Scalar color = Scalar(0,0,255);
	drawContours(contourImage,testContour,0,color,CV_FILLED);
	vector<Mat> channels;
	split(contourImage,channels);
//	int sz = countNonZero(channels[2]);
	Mat nonzerocoord;
	findNonZero(channels[2],nonzerocoord);
    Mat data_pts = Mat(nonzerocoord.total(), 2, CV_64FC1);
    for (int i = 0; i < data_pts.rows; ++i)
    {
	
        data_pts.at<double>(i, 0) = nonzerocoord.at<Point>(i).x;
        data_pts.at<double>(i, 1) = nonzerocoord.at<Point>(i).y;
    }

    //Perform PCA analysis
    PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);

    //Store the position of the object
    Point pos = Point(pca_analysis.mean.at<double>(0, 0),
                      pca_analysis.mean.at<double>(0, 1));
	meanVec.clear();
	meanVec.push_back(pos.x);
	meanVec.push_back(pos.y);
    //Store the eigenvalues and eigenvectors
    vector<Point2d> eigen_vecs(2);
    vector<double> eigen_val(2);
    for (int i = 0; i < 2; ++i)
    {
        eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                                pca_analysis.eigenvectors.at<double>(i, 1));

        eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
    }

    // Draw the principal components
    circle(img, pos, 3, CV_RGB(255, 0, 255), 2);
    line(img, pos, pos + 0.02 * Point(eigen_vecs[0].x * eigen_val[0], eigen_vecs[0].y * eigen_val[0]) , CV_RGB(255, 255, 0));
    line(img, pos, pos + 0.02 * Point(eigen_vecs[1].x * eigen_val[1], eigen_vecs[1].y * eigen_val[1]) , CV_RGB(0, 255, 255));
	dir.at<double>(0,0) = eigen_vecs[0].x;
	dir.at<double>(1,0) = eigen_vecs[0].y;
	dir.at<double>(0,1) = eigen_vecs[1].x;
	dir.at<double>(1,1) = eigen_vecs[1].y;
//    return atan2(eigen_vecs[0].y, eigen_vecs[0].x);
}

#endif
