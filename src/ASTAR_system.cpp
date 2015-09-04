#include "object_perception_node.h"


bool cmp(const eC& e1, const eC& e2);
//void object_perception::callback(const ImageConstPtr& image, const  ImageConstPtr& depth)
void object_perception::callback(const ImageConstPtr& image)//, const  ImageConstPtr& depth)
{
    ROS_INFO_STREAM("We got both images");
#if 1
    cv_bridge::CvImagePtr cv_ptr;
//    cv_bridge::CvImagePtr depth_ptr;
    Point a(wsParam[0],wsParam[1]), b(wsParam[0],wsParam[3]), c(wsParam[2],wsParam[1]), d(wsParam[2],wsParam[3]);
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
//        depth_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_16UC1);
        viconImg = cv_ptr->image.clone();
        extractPoseImage(cv_ptr->image);//Estimate H_im within camera coordinate system (transformation matrix from contourObj (old database) to recent contour)
        if(createDatabase){
            imageNo++;
            if(imageNo>0 && objFound){
                createDatabase = false;
                vector<Point> newAxis = axis, contourA = contourObj;
                transformContour(H_im,newAxis,axis);
                transformContour(H_im,contourA,contourObj);
                if(!smallObj){
                    vector<Point> tempC = contourHolder;
                    transformContour(H_im,tempC,contourHolder);
                }
                dataBaseContour = contourObj;
                getImagePose(original_H_im);//estimate the transformation from camera coordinate (0,1; 1,0; 0,0) to axis
                prepareImage(viconImg,original,dataBaseContour);//chang the original image according to this new contourObj
                dataH_im = Mat::eye(3,3,CV_32F);//original_H_im.clone();
                original_H_im.copyTo(dataH_im(Rect(0,0,3,2)));//dataH_im now correspond to initial camera axis to axis of contourObj

                ROS_INFO_STREAM("Moving to actual");
            }
            if(imageNo>10 && !objFound){
                imwrite("/home/ir/perception_ws/src/pose_estimate_ASTAR/data/data.png",cv_ptr->image);
            }
        }
        else{
            if(objFound){
                double valZ = 1.3;
//                ROS_INFO_STREAM("From image based pose estimation: \n"<<H_im);
                vector< vector<Point> > contourFinal0;
                contourFinal0.push_back(dataBaseContour);
                contourFinal0.push_back(contourObj);
                transformContour(H_im, dataBaseContour, contourFinal0[1]);
                drawContours(cv_ptr->image,contourFinal0,1,Scalar(255,0,0),2,8,0,0,Point());
                vector<Point> newAxis = axis;
                transformContour(H_im,axis,newAxis);
                line(cv_ptr->image,newAxis[0],newAxis[1],Scalar(0,0,255),4,8,0);
                line(cv_ptr->image,newAxis[0],newAxis[2],Scalar(0,255,0),4,8,0);

                if(!smallObj){
                    vector<Point> tempC = contourHolder;
                    transformContour(H_im,contourHolder,tempC);
                    line(cv_ptr->image,tempC[2],tempC[0],Scalar(0,0,255),4,8,0);
                    line(cv_ptr->image,tempC[2],tempC[1],Scalar(0,255,0),4,8,0);                
                }
#if 1
                double fx = 1.0/cameraM.at<double>(0,0);
                double fy = 1.0/cameraM.at<double>(1,1);
                double cx = cameraM.at<double>(0,2);
                double cy = cameraM.at<double>(1,2);
                pcl::PointCloud<pcl::PointXYZ> cloud;
                cloud.header.frame_id = "/kinect2_rgb_optical_frame"; 
                if(smallObj){
                for(int i=0;i<3;i++){
                    pcl::PointXYZ pt;
                    ptsData.data[i*3+2] = valZ;
                    cout<<newAxis[i]<<endl;
//                    ptsData.data[i*3+2] = (static_cast<float>(depth_ptr->image.at<uint16_t>(newAxis[i])))/1000.0;
                    ptsData.data[i*3] = (newAxis[i].x-cx)*ptsData.data[i*3+2]*fx;
                    ptsData.data[i*3+1] = (newAxis[i].y-cy)*ptsData.data[i*3+2]*fy;
                    pt.x = ptsData.data[i*3];
                    pt.y = ptsData.data[i*3+1];
                    pt.z = ptsData.data[i*3+2];
                    cloud.points.push_back(pt);
                }
                    cout<<endl;
                }
                else{
                    vector<Point> tempC = contourHolder;
                    transformContour(H_im,contourHolder,tempC);
                for(int i=0;i<3;i++){
                    pcl::PointXYZ pt;
                    ptsData.data[i*3+2] = valZ;
//                    ptsData.data[i*3+2] = (static_cast<float>(depth_ptr->image.at<uint16_t>(tempC[i])))/1000.0;
                    ptsData.data[i*3] = (tempC[i].x-cx)*ptsData.data[i*3+2]*fx;
                    ptsData.data[i*3+1] = (tempC[i].y-cy)*ptsData.data[i*3+2]*fy;
                    pt.x = ptsData.data[i*3];
                    pt.y = ptsData.data[i*3+1];
                    pt.z = ptsData.data[i*3+2];
                    cloud.points.push_back(pt);
                }
                }
		
                pubCloud.publish(cloud);
                pubPts.publish(ptsData);

					 // convert to transformation format
                
				    

#endif
                imageNo++;
            }
        }
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    line(cv_ptr->image,a,b,Scalar(0,0,255),4,8,0);
    line(cv_ptr->image,a,c,Scalar(255,0,0),4,8,0);
    line(cv_ptr->image,b,d,Scalar(255,0,255),4,8,0);
    line(cv_ptr->image,c,d,Scalar(0,255,255),4,8,0);
//    image_pub_.publish(cv_ptr->toImageMsg());
    image_pub_.publish(cv_ptr->toImageMsg());
#endif
}

bool cmp(const eC& e1, const eC& e2){
    if(e1.found && e2.found){
        if(e1.err01<e2.err01 && e1.err02<=e2.err02)
            return true;
        else
            if(e1.err01==e2.err01 && e1.err02<e2.err02)
                return true;
            else
                if((e1.err01+e1.err02)<=(e2.err01+e2.err02))
                    return true;
                else
                    return false;
    }
    else{
        if(e1.found)
            return true;
        else
            if(e2.found)
                return false;
            else
                if(e1.err01<e2.err01 && e1.err02<=e2.err02)
                    return true;
                else
                    if(e1.err01==e2.err01 && e1.err02<e2.err02)
                        return true;
                    else
                        if((e1.err01+e1.err02)<=(e2.err01+e2.err02))
                            return true;
                        else
                            return false;
    }
                
}

void object_perception::extractPoseImage(Mat &src){
    errorVec.clear();
    findContourScene(src);
    double eps0=10, eps1=10;
    objFound = false;
//    Mat view = src.clone();
    
    for(size_t i=0;i<contourScenes.size();i++){
//        objFound = false;
        contourNo = i;
        findPoseContour(src,contourScenes[i]);
        eC eCurrent;
        eCurrent.pos = i;
        vector< vector<Point> > contourCheck;
        contourCheck.push_back(contourObj);
        contourCheck.push_back(contourScenes[i]);
        transformContour(H_im, contourObj, contourCheck[0]);
        checkContours(contourCheck,eps0,eps1);
//        errorVec[errorVec.size()-1].pos = i;
        eCurrent.err01 = eps0;
        eCurrent.err02 = eps1;
        eCurrent.H = H_im.clone();
        eCurrent.found = false;
        if(eps0<areaTh && eps1<areaTh){
            eCurrent.found = true;
#if 0
            vector< vector<Point> > contourFinal;
            contourFinal.push_back(contourObj);
            contourFinal.push_back(axis);
#endif
        }
//        drawContours(view,contourScenes,i,Scalar(255,0,0),2,8,0,0,Point());
        errorVec.push_back(eCurrent);
    }
#if 0
    imshow("contours",view);
    waitKey(1);
#endif
    sort(errorVec.begin(), errorVec.end(), cmp);
    if(errorVec.size()>0){
        objFound = errorVec[0].found;
//        ROS_INFO_STREAM("Object is not found "<<objFound<<" "<<errorVec[0].err01);
    }
    if(objFound){
        contourCurrent = contourScenes[errorVec[0].pos];
        H_im = errorVec[0].H.clone();
    }
    else{
        H_im = Mat::zeros(3,3,CV_32F);
    }
    if(H_im.type()==6)
        H_im.convertTo(H_im,CV_32F);
}
