#include "object_perception_node.h"

void object_perception::checkAndDeleteNegative(vector<Point2f> &obj, vector<Point2f> &scene){
    Rect ws_(wsParam[0],wsParam[1],wsParam[2],wsParam[3]);
    if(obj.size()>3 && obj.size()==scene.size()){
        for(size_t i=0;i<obj.size();i++){
            if(!ws_.contains(obj[i]) || !ws_.contains(scene[i]) ){
                obj.erase(obj.begin()+i);
                scene.erase(scene.begin()+i);
                if (i>0)
                    i = i-1;
                else
                    i = 0;
            }
        }
    }
}
void object_perception::transformContour(Mat H_im, vector<Point> contouri, vector<Point> &contourj){

    vector<Point2f> contourk(contouri.size()), contourl(contouri.size());
    for(size_t p=0;p<contouri.size();p++){
        contourk[p].x= (float)contouri[p].x;
        contourk[p].y= (float)contouri[p].y;
    }
    perspectiveTransform( contourk, contourl, H_im);
    contourj.clear();
    for(size_t p=0;p<contourl.size();p++){
        Point pt;
        pt.x = (int)contourl[p].x;
        pt.y = (int)contourl[p].y;
        contourj.push_back(pt);
    }
}

bool object_perception::finalCheckTransform(Mat workImage, vector<Point> contouri){

//    H_im = findHomography( obj, scene, CV_RANSAC );

    vector< vector<Point> > contourFinal;
    contourFinal.push_back(contourObj);
    transformContour(H_im, contourObj, contourFinal[0]);
    contourFinal.push_back(contouri);
    double frac01 = 10, frac02 = 10;
    checkContours(contourFinal, frac01, frac02);
//    ROS_INFO_STREAM("From image "<<frac01<<" "<<frac02);
#if 0
    eC e1;
    e1.err01 = frac01;
    e1.err02 = frac02;
    errorVec.push_back(e1);
#endif
    if(frac01<areaTh && frac02<areaTh)
    	return true;
    else
        return false;
}

void object_perception::checkContours(vector< vector<Point> > contourFinal, double &frac01, double &frac02){
    Mat dst, absd = Mat::zeros(original.size(),original.type());
//    contourScenes.push_back(contourFinal);
    vector<Mat> s;
//    double frac01 = 10, frac02 = 10;
    drawContours(absd,contourFinal,0,Scalar(0,0,255),CV_FILLED);
    split(absd,s);
    int countb4 = countNonZero(s[2]);
//    if((double)countb4/areaOri >0.9){
    drawContours(absd,contourFinal,1,Scalar(255,0,0),CV_FILLED);
#if 0
    Mat toShow;
    resize(absd,toShow,Size(absd.cols/4,absd.rows/4));
    imshow("1st result",toShow);
#endif
    split(absd,s);
    int countafter = countNonZero(s[2]);
//    ROS_INFO_STREAM("1st "<<countb4<<" "<<countafter);
    if(countb4>0)
        frac01 = (double)countafter/(double)countb4;
    else
        frac01 = 1;
    countb4 = countNonZero(s[0]);
   drawContours(absd,contourFinal,0,Scalar(0,0,255),CV_FILLED);
#if 0
    resize(absd,toShow,Size(absd.cols/4,absd.rows/4));
    imshow("2nd result",toShow);
    waitKey(1);
#endif
    split(absd,s);
    countafter = countNonZero(s[0]);
//    ROS_INFO_STREAM(countb4<<" "<<countafter);
    if(countb4>0)
        frac02 = (double)countafter/(double)countb4;
    else
        frac02 = 1;
    
//    contourScenes.erase(contourScenes.end());
//    H_im = estimateRigidTransform(obj, scene, true);
//    warpPerspective(original,dst,H_im,original.size(),INTER_LINEAR);
//    absdiff(dst,workImage,absd);

//    ROS_INFO_STREAM(sum(absd));
//    ROS_INFO_STREAM(frac01<<" "<<frac02);
//    }
//    else    return false;
}


void object_perception::findPoseContour(Mat &src, vector<Point> contouri){
    Mat workImage = src.clone();
    prepareImage(src, workImage, contouri);
#if 0
//    writeImage("workImage%d_%d",workImage);    
    imshow("original",original);
    imshow("now",workImage);
    waitKey(1);
#endif
#if 1
    
    if(!smallObj){
        int minHessian = 400;

        SurfFeatureDetector detector( minHessian );

        std::vector<KeyPoint> keypoints_object, keypoints_scene;

        detector.detect( original, keypoints_object );
        detector.detect( workImage, keypoints_scene );

  //-- Step 2: Calculate descriptors (feature vectors)
        SurfDescriptorExtractor extractor;

        Mat descriptors_object, descriptors_scene;

        extractor.compute( original, keypoints_object, descriptors_object );
        extractor.compute( workImage, keypoints_scene, descriptors_scene );

  //-- Step 3: Matching descriptor vectors using FLANN matcher
        FlannBasedMatcher matcher;
        std::vector< DMatch > matches;
        std::vector<Point2f> obj;
        std::vector<Point2f> scene;
        try{
            matcher.match( descriptors_object, descriptors_scene, matches );

            double max_dist = 0; double min_dist = 100;
            for( int i = 0; i < descriptors_object.rows; i++ )
            { double dist = matches[i].distance;
                if( dist < min_dist ) min_dist = dist;
                if( dist > max_dist ) max_dist = dist;
            }
            std::vector< DMatch > good_matches;
            for( int i = 0; i < descriptors_object.rows; i++ )
            { if( matches[i].distance < 3*min_dist )
             { good_matches.push_back( matches[i]); }
            }
  //-- Localize the object
            for( int i = 0; i < good_matches.size(); i++ )
            {
    //-- Get the keypoints from the good matches
                obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
                scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
            }
            checkAndDeleteNegative(obj, scene);
        }
        catch(cv::Exception& e){
            ROS_ERROR_STREAM("could not find homography due to "<<e.what());
        }
        if(obj.size()>3){
            H_im = findHomography( obj, scene, CV_RANSAC );
            if(finalCheckTransform(workImage, contouri))
                objFound = true;
            else
                objFound = false;
        }
        else{
            objFound = false;
            H_im = Mat::zeros(3,3,CV_32F);
        }
//        ROS_INFO_STREAM("object Found "<<objFound);
    }
    else{
        vector<Point> newaxis(3,Point());
        estimateNewAxis_small(contouri,newaxis);
        
        //H_im = findHomography(axis,newaxis,0);
        H_im = Mat::eye(3,3,CV_32F);
        try{
            Mat E = estimateRigidTransform(axis,newaxis,1);
//        ROS_INFO_STREAM(E);
            E.copyTo(H_im(Rect(0,0,3,2)));
        }
        catch(cv::Exception &e){
            ROS_ERROR_STREAM("from small object pose estimation. TRY: vim object_feature.cpp +190 "<<endl<<axis<<endl<<newaxis);
            H_im = Mat::zeros(3,3,CV_32F);
        }
        
//        ROS_INFO_STREAM("From image based pose estimation: \n"<<H_im);
//        ROS_INFO_STREAM(H_im);
//        H_im = E.clone();
        if(finalCheckTransform(workImage, contouri))
            objFound = true;
        else
            objFound = false;
    }
#endif
}

void object_perception::estimateNewAxis_small(vector<Point> contouri,vector<Point> &newaxis){
        vector<int> chullIdx;
        convexHull(contouri,chullIdx,true);
        double max_len = -1;
        int pos = -1;
        vector<double> distVec;
        for(size_t i=0;i<chullIdx.size();i++){
            Point a = contouri[chullIdx[i%chullIdx.size()]];
            Point b = contouri[chullIdx[(i+1)%chullIdx.size()]];

            double dist = sqrt(pow(a.x - b.x,2.0) + pow(a.y - b.y,2.0));
            distVec.push_back(dist);
            if(dist>max_len){
                max_len = dist;
                pos = i;
            }
        }
        Point2f a = contouri[chullIdx[pos%chullIdx.size()]];
        Point2f b = contouri[chullIdx[(pos+1)%chullIdx.size()]];
    //line(src,a,b,Scalar(0,0,255), 1,8,0);

        Point2f k;
        for(size_t i=0;i<chullIdx.size();i++){
            k = contouri[chullIdx[i%chullIdx.size()]];
            if(k!=a && k!=b)
                i = chullIdx.size();
        }

        Point2f c = (a + b);
        c.x /= 2.0;
        c.y /= 2.0;
        //circle(src,c,2,Scalar(255,0,0),1,8,0);
        Point2f d = a - b;
        float n = norm(d);
        d.x /= n;
        d.y /= n;
        Point2f dp = d;
        float temp = -d.y;
        d.y = d.x;
        d.x = temp;
        Point2f e = c+d*n;

        Point2f dir01 = c-e;
        Point2f dir02 = c-k;

        Point2f f = c+dp*n;
        if(dir01.dot(dir02)<0){
            d.x = -d.x;
            d.y = -d.y;
            dp.x = -dp.x;
            dp.y = -dp.y;
            e = c+d*n;
            f = c+dp*n;
        }
        //vector<Point> newaxis(3,Point());
        newaxis[0].x = (int)c.x;
        newaxis[0].y = (int)c.y;
        newaxis[1].x = (int)f.x;
        newaxis[1].y = (int)f.y;
        newaxis[2].x = (int)e.x;
        newaxis[2].y = (int)e.y;
#if 0
        vector< vector<Point> > contourFinal;
        contourFinal.push_back(newaxis);
        contourFinal.push_back(axis);
        Mat src = Mat::zeros(original.rows,original.cols,original.type());
        for(size_t i=0;i<2;i++){
            line(src,contourFinal[i][0],contourFinal[i][1],Scalar(0,0,255),1,8,0);
            line(src,contourFinal[i][0],contourFinal[i][2],Scalar(255,0,0),1,8,0);
        }
        imshow("test",src);
        waitKey(1);
#endif
}

void object_perception::prepareImage(Mat src, Mat &work, vector<Point> contouri){
    Mat maskWork = Mat::zeros(src.size(),src.type());
    work = maskWork.clone();
    vector< vector<Point> > contourTemp;
    contourTemp.push_back(contouri);
    drawContours(maskWork,contourTemp,0,Scalar(255,255,255),CV_FILLED);
    vector<Mat> chs;
    split(maskWork,chs);
    Mat mask = chs[0].clone();
    src.copyTo(work,mask);
#if 0
    imshow("original",src);
    imshow("onlyContour",work);
    waitKey(-1);
#endif
}

