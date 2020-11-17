/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"orb_slam3/System.h"
#include "ROSAtlas.h"
#include "ROSImageGrabber.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();
    ros::NodeHandle nh;
    ros::NodeHandle local_nh{"~"};
    
    /// proceed flag allows us to maintain a singular exit point. If it ever becomes false, the whole program should bail out.
    bool proceed = true;
    
    /**
     * Read the Required Parameters from Parameter Server
     */
    std::string vocab_loc;
    if (!local_nh.getParam("vocab_file", vocab_loc))
    {
        ROS_ERROR("Vocab file parameter not set.");
        proceed = false;
    }
    
    std::string all_params_loc;
    if (proceed && !local_nh.getParam("all_params", all_params_loc))
    {
        ROS_ERROR("All params file parameter not set.");
        proceed = false;
    }
    
    std::string sensor_setup;
    if (proceed && !local_nh.getParam("sensor_setup", sensor_setup))
    {
        ROS_ERROR("Sensor setup parameter not set.");
        proceed = false;
    }
    
    ORB_SLAM3::System::eSensor sensors;
    if (sensor_setup == "mono")
        sensors = ORB_SLAM3::System::MONOCULAR;
    else if (sensor_setup == "mono-inertial")
    {
        sensors = ORB_SLAM3::System::IMU_MONOCULAR;
        ROS_ERROR("Mono with IMU not yet supported in this ROS package.");
        proceed = false;
    }
    else if (sensor_setup == "stereo")
        sensors = ORB_SLAM3::System::STEREO;
    else if (sensor_setup == "stereo-inertial")
    {
        sensors = ORB_SLAM3::System::IMU_STEREO;
        ROS_ERROR("Stereo with IMU not yet supported in this ROS package.");
        proceed = false;
    }
    else if (sensor_setup == "RGBD")
    {
        sensors = ORB_SLAM3::System::RGBD;
        ROS_ERROR("RGBD not yet supported in this ROS package.");
        proceed = false;
    }
    else
    {
        ROS_ERROR("Bad sensor setup type passed");
        proceed = false;
    }
    
    
    /**
     * Start the setup now that the required parameters are loaded
     */
    
    if( proceed )
    {
        /**
         * Create a ROS specific Atlas because it is what publishes our position
         */
        std::string transform_out_topic{"orb_slam/transform"};
        local_nh.getParam("transform_out_topic", transform_out_topic);
        auto atlas = new ROSAtlas(nh, transform_out_topic, 0);
        
        /**
         * Create the Pangolin windows to see key frames and image output
         */
        bool use_viewer{true};
        local_nh.getParam("use_viewer", use_viewer);
        
        
        /// Create SLAM system. It initializes all system threads and gets ready to process frames.
        /// TODO: replace threads with ROS Timers
        ORB_SLAM3::System SLAM(atlas, vocab_loc, all_params_loc, sensors, use_viewer);
    
        /**
         * Get the appropriate image grabber based on our sensor setup.
         */
        std::unique_ptr<ROSImageGrabber> image_grabber;
        switch(sensors)
        {
            default: ROS_ERROR("NOT YET SUPPORTED"); proceed = false;
            case ORB_SLAM3::System::MONOCULAR:
            {
                std::string mono_in_topic{"camera/image_raw"};
                local_nh.getParam("mono_cam_in_topic", mono_in_topic);
                image_grabber = std::make_unique<ROSMonoImageGrabber>(&SLAM, nh, mono_in_topic);
                break;
            }
            case ORB_SLAM3::System::STEREO:
            {
                std::string stereo_in_left_topic{"camera/left/image_raw"};
                std::string stereo_in_right_topic{"camera/right/image_raw"};
                bool should_rectify = false;
                
                local_nh.getParam("stereo_in_left_topic", stereo_in_left_topic);
                local_nh.getParam("stereo_in_right_topic", stereo_in_right_topic);
                local_nh.getParam("should_rectify", should_rectify);
                
                if (should_rectify)
                {
                    cv::Mat m1l, m2l, m1r, m2r;
                    cv::FileStorage fsSettings(all_params_loc, cv::FileStorage::READ);
                    if (!fsSettings.isOpened())
                    {
                        ROS_ERROR("Wrong path given for settings file.");
                        proceed = false;
                    }
    
                    if( proceed )
                    {
                        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
                        fsSettings["LEFT.K"] >> K_l;
                        fsSettings["RIGHT.K"] >> K_r;
    
                        fsSettings["LEFT.P"] >> P_l;
                        fsSettings["RIGHT.P"] >> P_r;
    
                        fsSettings["LEFT.R"] >> R_l;
                        fsSettings["RIGHT.R"] >> R_r;
    
                        fsSettings["LEFT.D"] >> D_l;
                        fsSettings["RIGHT.D"] >> D_r;
    
                        int rows_l = fsSettings["LEFT.height"];
                        int cols_l = fsSettings["LEFT.width"];
                        int rows_r = fsSettings["RIGHT.height"];
                        int cols_r = fsSettings["RIGHT.width"];
    
                        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() ||
                            D_l.empty() ||
                            D_r.empty() ||
                            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
                        {
                            ROS_ERROR("Calibration parameters to rectify stereo are missing!");
                            proceed = false;
                        }
    
                        if( proceed )
                        {
                            cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3),
                                                        cv::Size(cols_l, rows_l),
                                                        CV_32F, m1l, m2l);
                            cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3),
                                                        cv::Size(cols_r, rows_r),
                                                        CV_32F, m1r, m2r);
                            image_grabber = std::make_unique<ROSStereoImageGrabber>(
                                &SLAM, nh, stereo_in_left_topic, stereo_in_right_topic, should_rectify,
                                m1l, m2l, m1r, m2r
                            );
                        }
                    }
                }
                else
                {
                    image_grabber = std::make_unique<ROSStereoImageGrabber>(
                        &SLAM, nh, stereo_in_left_topic, stereo_in_right_topic, should_rectify
                    );
                }
                break;
            }
        }
        
        if( proceed )
            ros::spin();
    
        // Stop all threads
        SLAM.Shutdown();

        ros::shutdown();
    }
    return proceed ? 0 : -1;
}
