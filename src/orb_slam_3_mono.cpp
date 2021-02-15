//
// Created by korisd on 2/15/21.
//
#include<ros/ros.h>

#include"orb_slam3/System.h"
#include "ImageTransformer.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "orb_mono");
    ros::start();
    ros::NodeHandle nh{};
    
    // Commandline parameters
    std::string vocab_loc;
    std::string orb_params_loc;
    std::string in_topic;
    std::string out_topic;
    std::string tf_parent;
    std::string tf_child;
    std::string features_image_topic;
    bool        use_viewer;
    
    // Flag for proceeding through initialization
    bool proceed = true;
    
    /*
     * Read position of orb vocab
     */
    if (!nh.getParam("file/vocab", vocab_loc))
    {
        ROS_ERROR("file/vocab parameter not set.");
        proceed = false;
    }
    
    if (proceed && !nh.getParam("file/orb_params", orb_params_loc))
    {
        ROS_ERROR("file/orb_params parameter not set.");
        proceed = false;
    }
    
    if (proceed && !nh.getParam("input/mono_cam_topic", in_topic))
    {
        ROS_ERROR("input/mono_cam_topic parameter not set.");
        proceed = false;
    }
    
    if (proceed && !nh.getParam("output/transform_topic", out_topic))
    {
        ROS_ERROR("output/transform_topic parameter not set.");
        proceed = false;
    }
    
    if (proceed && !nh.getParam("tf/parent", tf_parent))
    {
        ROS_ERROR("tf/parent parameter not set.");
        proceed = false;
    }
    
    if (proceed && !nh.getParam("tf/child", tf_child))
    {
        ROS_ERROR("tf/child parameter not set.");
        proceed = false;
    }
    
    
    if (proceed && !nh.getParam("use_viewer", use_viewer))
    {
        ROS_ERROR("use_viewer parameter not set.");
        proceed = false;
    }
    
    if (proceed &&!nh.getParam("output/features_image_topic", features_image_topic))
    {
        ROS_ERROR("output/features_image_topic parameter not set.");
        proceed = false;
    }
    
    if (proceed)
    {
        auto SLAM = std::make_shared<ORB_SLAM3::System>(
            vocab_loc,
            orb_params_loc,
            ORB_SLAM3::System::MONOCULAR,
            use_viewer
        );
        
        auto image_transformer = std::make_unique<MonoImageTransformer>(
            nh,
            SLAM,
            in_topic,
            out_topic,
            tf_parent,
            tf_child,
            features_image_topic,
            use_viewer
        );
        
        ros::spin();

        SLAM->Shutdown();
        ros::shutdown();
    }
    
    return proceed ? 0 : -1;
}