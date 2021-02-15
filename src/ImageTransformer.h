//
// Created by korisd on 11/16/20.
//

#ifndef ORB_SLAM_3_ROS_IMAGETRANSFORMER_H
#define ORB_SLAM_3_ROS_IMAGETRANSFORMER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <utility>

namespace ORB_SLAM3 {
    class System;
}

namespace tf2 {
    class Transform;
}

class ImageTransformer
{
public:
    explicit ImageTransformer(
        ros::NodeHandle&                    nh,
        std::shared_ptr <ORB_SLAM3::System> pSLAM,
        std::string                         tf_parent,
        std::string                         tf_child,
        const std::string&                  features_topic,
        bool                                use_viewer = false
    );
    
    void PublishFeaturesImage();
protected:
    static tf2::Transform TransformFromMat(const cv::Mat& position_mat);
    
    image_transport::ImageTransport     mono_image_transport_;
    image_transport::Publisher          features_image_pub_;
    std::shared_ptr <ORB_SLAM3::System> pSLAM_;
    const std::string                   tf_parent_;
    const std::string                   tf_child_;
    bool                                use_viewer_;
};

class MonoImageTransformer : public ImageTransformer
{
public:
    MonoImageTransformer(
        ros::NodeHandle& nh,
        std::shared_ptr<ORB_SLAM3::System> pSLAM,
        const std::string& in_topic,
        const std::string& out_topic,
        const std::string& tf_parent,
        const std::string& tf_child,
        const std::string& features_image_topic,
        bool               use_viewer = false
    );
private:
    void ImageCb(const sensor_msgs::ImageConstPtr& msg);
    
    image_transport::Subscriber     mono_image_sub_;
    ros::Publisher                  position_transform_pub_;
};

#endif //ORB_SLAM_3_ROS_IMAGETRANSFORMER_H
