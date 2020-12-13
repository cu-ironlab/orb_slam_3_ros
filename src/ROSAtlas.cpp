//
// Created by korisd on 11/15/20.
//

#include "ROSAtlas.h"
#include "orb_slam3/KeyFrame.h"
#include "orb_slam3/Converter.h"
#include <geometry_msgs/TransformStamped.h>

ROSAtlas::ROSAtlas(ros::NodeHandle &nh, const std::string &transform_topic, int initFKid)
:
    ORB_SLAM3::Atlas(initFKid), _nh(nh)
{
    _transform_pub = _nh.advertise<geometry_msgs::TransformStamped>(transform_topic, 10);
}

void ROSAtlas::AddKeyFrame(ORB_SLAM3::KeyFrame *pKF)
{
    geometry_msgs::TransformStamped msg;
    cv::Mat rotation    = pKF->GetRotation();
    vector<float> quat  = ORB_SLAM3::Converter::toQuaternion(rotation);
    cv::Mat translation = pKF->GetCameraCenter();
    msg.header.stamp    = ros::Time(pKF->mTimeStamp);
    msg.header.frame_id = "orb_origin";
    msg.child_frame_id  = "orb_slam_3";
    msg.transform.translation.x = translation.at<float>(0);
    msg.transform.translation.y = translation.at<float>(1);
    msg.transform.translation.z = translation.at<float>(2);
    msg.transform.rotation.x = quat[0];
    msg.transform.rotation.y = quat[1];
    msg.transform.rotation.z = quat[2];
    msg.transform.rotation.w = quat[3];
    _transform_pub.publish( msg );

    this->Atlas::AddKeyFrame(pKF);
}