//
// Created by korisd on 11/16/20.
//
#include <cv_bridge/cv_bridge.h>

#include <utility>
#include "ROSImageGrabber.h"
#include "orb_slam3/System.h"

ROSMonoImageGrabber::ROSMonoImageGrabber(ORB_SLAM3::System *pSLAM, ros::NodeHandle &nh, const std::string &topic)
    : ROSImageGrabber(pSLAM), _nh(nh)
{
    _mono_image_sub = _nh.subscribe(topic, 100, &ROSMonoImageGrabber::ImageCb, this);
}


void ROSMonoImageGrabber::ImageCb(const sensor_msgs::ImageConstPtr &msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
}

ROSStereoImageGrabber::ROSStereoImageGrabber(ORB_SLAM3::System *pSLAM, ros::NodeHandle &nh,
                                             const std::string &left_topic, const std::string &right_topic,
                                             bool rectify)
    : ROSImageGrabber(pSLAM),
      _nh(nh),
      _stereo_left_sub(_nh, left_topic, 1),
      _stereo_right_sub(_nh, right_topic, 1),
      _sync(sync_pol(10), _stereo_left_sub, _stereo_right_sub),
      _rectify(rectify)
{
    _sync.registerCallback(boost::bind(&ROSStereoImageGrabber::ImageCb, this, _1, _2));
}

ROSStereoImageGrabber::ROSStereoImageGrabber(ORB_SLAM3::System *pSLAM, ros::NodeHandle &nh,
                                             const std::string &left_topic, const std::string &right_topic,
                                             bool rectify, cv::Mat m1l, cv::Mat m2l, cv::Mat m1r, cv::Mat m2r)
    : ROSImageGrabber(pSLAM),
      _nh(nh),
      _stereo_left_sub(_nh, left_topic, 1),
      _stereo_right_sub(_nh, right_topic, 1),
      _sync(sync_pol(10), _stereo_left_sub, _stereo_right_sub),
      _rectify(rectify),
      _m1l(std::move(m1l)),
      _m2l(std::move(m2l)),
      _m1r(std::move(m1r)),
      _m2r(std::move(m2r))
      
{
    _sync.registerCallback(boost::bind(&ROSStereoImageGrabber::ImageCb, this, _1, _2));
}

void ROSStereoImageGrabber::ImageCb(const sensor_msgs::ImageConstPtr &left_msg,
                                    const sensor_msgs::ImageConstPtr &right_msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(left_msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(right_msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    if(_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,_m1l,_m2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,_m1r,_m2r,cv::INTER_LINEAR);
        mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }
}