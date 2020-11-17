//
// Created by korisd on 11/16/20.
//

#ifndef ORB_SLAM_3_ROS_ROSIMAGEGRABBER_H
#define ORB_SLAM_3_ROS_ROSIMAGEGRABBER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


namespace ORB_SLAM3 {
    class System;
}

class ROSImageGrabber
{
public:
    explicit ROSImageGrabber(ORB_SLAM3::System *pSLAM) : mpSLAM(pSLAM) {}
    virtual ~ROSImageGrabber() {}
    
protected:
    ORB_SLAM3::System *mpSLAM;
};

class ROSMonoImageGrabber : public ROSImageGrabber
{
public:
    ROSMonoImageGrabber(ORB_SLAM3::System *pSLAM, ros::NodeHandle &nh, const std::string &topic);
private:
    void ImageCb(const sensor_msgs::ImageConstPtr& msg);
    
    ros::NodeHandle& _nh;
    ros::Subscriber _mono_image_sub;
};

class ROSStereoImageGrabber : public ROSImageGrabber
{
public:
    ROSStereoImageGrabber(ORB_SLAM3::System *pSLAM, ros::NodeHandle &nh, const std::string &left_topic,
                          const std::string &right_topic, bool rectify);
    ROSStereoImageGrabber(ORB_SLAM3::System *pSLAM, ros::NodeHandle &nh, const std::string &left_topic,
                          const std::string &right_topic, bool rectify, cv::Mat m1l, cv::Mat m2l,
                          cv::Mat m1r, cv::Mat m2r);
private:
    void ImageCb(const sensor_msgs::ImageConstPtr& left_msg, const sensor_msgs::ImageConstPtr& right_msg);
    ros::NodeHandle& _nh;
    
    message_filters::Subscriber<sensor_msgs::Image> _stereo_left_sub;
    message_filters::Subscriber<sensor_msgs::Image> _stereo_right_sub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> _sync;
    bool _rectify;
    cv::Mat _m1l;
    cv::Mat _m2l;
    cv::Mat _m1r;
    cv::Mat _m2r;
    
};

#endif //ORB_SLAM_3_ROS_ROSIMAGEGRABBER_H
