//
// Created by korisd on 11/16/20.
//
#include <cv_bridge/cv_bridge.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Header.h>
#include <utility>

#include "ImageTransformer.h"
#include "orb_slam3/System.h"



//==============================================================================
// BASE CLASS UTILITIES
//==============================================================================
ImageTransformer::ImageTransformer(
    ros::NodeHandle&                    nh,
    std::shared_ptr <ORB_SLAM3::System> pSLAM,
    std::string                         tf_parent,
    std::string                         tf_child,
    const std::string&                  features_topic,
    bool                                use_viewer
)
:
    mono_image_transport_(nh                  ),
    pSLAM_               (std::move(pSLAM)    ),
    tf_parent_           (std::move(tf_parent)),
    tf_child_            (std::move(tf_child )),
    use_viewer_          (use_viewer          )
{
    if (!use_viewer)
        features_image_pub_ = mono_image_transport_.advertise(features_topic, 1);
}

tf2::Transform ImageTransformer::TransformFromMat(const cv::Mat& position_mat)
{
    cv::Mat rotation(3,3,CV_32F);
    cv::Mat translation(3,1,CV_32F);
    
    rotation = position_mat.rowRange(0,3).colRange(0,3);
    translation = position_mat.rowRange(0,3).col(3);
    
    
    tf2::Matrix3x3 tf_camera_rotation (rotation.at<float> (0,0), rotation.at<float> (0,1), rotation.at<float> (0,2),
                                       rotation.at<float> (1,0), rotation.at<float> (1,1), rotation.at<float> (1,2),
                                       rotation.at<float> (2,0), rotation.at<float> (2,1), rotation.at<float> (2,2)
    );
    
    tf2::Vector3 tf_camera_translation (translation.at<float> (0), translation.at<float> (1), translation.at<float> (2));
    
    //Coordinate transformation matrix from orb coordinate system to ros coordinate system
    const tf2::Matrix3x3 tf_orb_to_ros (0, 0, 1,
                                        -1, 0, 0,
                                        0,-1, 0);
    
    //Transform from orb coordinate system to ros coordinate system on camera coordinates
    tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
    tf_camera_translation = tf_orb_to_ros*tf_camera_translation;
    
    //Inverse matrix
    tf_camera_rotation = tf_camera_rotation.transpose();
    tf_camera_translation = -(tf_camera_rotation*tf_camera_translation);
    
    //Transform from orb coordinate system to ros coordinate system on map coordinates
    tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
    tf_camera_translation = tf_orb_to_ros*tf_camera_translation;
    
    return tf2::Transform (tf_camera_rotation, tf_camera_translation);
}

void ImageTransformer::PublishFeaturesImage()
{
   if (!use_viewer_)
   {
       std_msgs::Header header;
       auto mat                                     = pSLAM_->GetFeaturesImage();
       header.stamp                                 = ros::Time::now();
       header.frame_id                              = tf_child_;
       const sensor_msgs::ImagePtr features_img_msg = cv_bridge::CvImage(header, "bgr8", mat).toImageMsg();
       
       features_image_pub_.publish(features_img_msg);
   }
}

//==============================================================================
// MONO IMAGE GRABBER
//==============================================================================

MonoImageTransformer::MonoImageTransformer(
    ros::NodeHandle& nh,
    shared_ptr<ORB_SLAM3::System> pSLAM,
    const std::string& in_topic,
    const std::string& out_topic,
    const std::string& tf_parent,
    const std::string& tf_child,
    const std::string& features_image_topic,
    bool               use_viewer
)
:
    ImageTransformer(nh,
                     std::move(pSLAM),
                     tf_parent,
                     tf_child,
                     features_image_topic,
                     use_viewer)
{
    mono_image_sub_         = mono_image_transport_.subscribe(in_topic, 1, &MonoImageTransformer::ImageCb, this, image_transport::TransportHints("compressed"));
    position_transform_pub_ = nh.advertise<geometry_msgs::TransformStamped>(out_topic, 1);
}

void MonoImageTransformer::ImageCb(const sensor_msgs::ImageConstPtr& msg)
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

    auto position_mat = pSLAM_->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());

    // if Mat wasn't empty
    if (!position_mat.empty())
    {
        // Get the Transform from the CV Mat
        auto transform = TransformFromMat(position_mat);
    
        // Build the Message
        tf2::Stamped<tf2::Transform> tf_position_stamped{transform, ros::Time::now(), tf_parent_};
        geometry_msgs::TransformStamped tf_msg = tf2::toMsg(tf_position_stamped);
        tf_msg.child_frame_id = tf_child_;
        
        position_transform_pub_.publish(tf_msg);
        
        // publish features image (check for "if it should" done in the underlying function)
        PublishFeaturesImage();
    }
}

