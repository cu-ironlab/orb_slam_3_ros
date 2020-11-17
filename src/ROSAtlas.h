//
// Created by korisd on 11/15/20.
//

#ifndef ORB_SLAM_3_ROS_ROSATLAS_H
#define ORB_SLAM_3_ROS_ROSATLAS_H

#include "orb_slam3/Atlas.h"
#include <ros/ros.h>

class KeyFrame;

class ROSAtlas : public ORB_SLAM3::Atlas
{
public:
    ROSAtlas(ros::NodeHandle &nh, const std::string &transform_topic, int initFKid);
    void AddKeyFrame(ORB_SLAM3::KeyFrame *pKF) override;
protected:
    ros::NodeHandle&    _nh;
    ros::Publisher      _transform_pub;
};


#endif //ORB_SLAM_3_ROS_ROSATLAS_H
