/*
* Copyright (c) 2016 Carnegie Mellon University, Author <gdubey@andrew.cmu.edu>
*
* For License information please see the LICENSE file in the root directory.
*
*/

//Outputs a point cloud using disparity images.
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "std_msgs/Header.h"
#include <cv_bridge/cv_bridge.h>
#include "opencv2/core/core.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <vector>
#include <fstream>
#include <stdio.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <opencv_apps/FlowArrayStamped.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <deque>
//#include <algorithm>
#include <numeric>
#include "flow_opencv.hpp"

class vel_estimator{
public:
    vel_estimator(ros::NodeHandle& nh);
    bool got_cam_info;

    ros::Subscriber cam_info_sub_;
    ros::Subscriber im_sub_;
    ros::Publisher twist_pub_;

    void callback(const opencv_apps::FlowArrayStamped::ConstPtr &msg_flow,
                  const sensor_msgs::Imu::ConstPtr &msg_imu,
                  const sensor_msgs::LaserScan::ConstPtr &msg_laser);
    void callback2(const sensor_msgs::Image::ConstPtr &msg_im,
                                 const sensor_msgs::Imu::ConstPtr &msg_imu,
                                 const sensor_msgs::LaserScan::ConstPtr &msg_laser);
    void getCamInfo(const sensor_msgs::CameraInfo::ConstPtr &msg_info);
    void im_callback(const sensor_msgs::Image::ConstPtr &msg_im);

    double fx_,fy_,cx_,cy_;
    unsigned int width,height;
    ros::Time prev_time;

    int N;

    std::deque<double> speed_samples_;
    std::deque<double> vx_samples_;
    std::deque<double> vy_samples_;
    double updated_speed;
    double alpha;
    double vel_sigma;
    bool w_corr;

    OpticalFlowOpenCV* of_obj;


};

void vel_estimator::getCamInfo(const sensor_msgs::CameraInfo::ConstPtr& msg_info)
{
    if(got_cam_info)
        return;
    image_geometry::PinholeCameraModel model_;model_.fromCameraInfo ( msg_info );
    ROS_INFO_ONCE("Cam Info Recvd Fx Fy Cx Cy: %f %f , %f %f",model_.fx(),model_.fy(),model_.cx(),model_.cy());
    cx_ = model_.cx();
    cy_ = model_.cy();
    fx_ = model_.fx();
    fy_ = model_.fy();
    width = msg_info->width;
    height = msg_info->height;
    got_cam_info = true;
    prev_time = msg_info->header.stamp;
    of_obj = new OpticalFlowOpenCV(fx_,fy_,-1,width,height);
}

void vel_estimator::im_callback(const sensor_msgs::Image::ConstPtr &msg_im)
{
    cv::Mat cv_im;
    cv_bridge::CvImageConstPtr cv_br_im;
    cv_br_im = cv_bridge::toCvShare(msg_im);
    cv_im = cv_br_im->image;
    float flow_x,flow_y;
    const uint32_t sec = (msg_im->header.stamp.sec);
    int usec = (msg_im->header.stamp.nsec/1000);
    int quality = of_obj->calcFlow(cv_im.data,sec,usec,flow_x,flow_y);
    ROS_INFO("Quality: %d FLOW: %f %f",quality,flow_x,flow_y);
    double vx = flow_x;
    double vy = flow_y;
    geometry_msgs::TwistStamped twist;
    twist.header = msg_im->header;
    twist.twist.linear.x = vx;
    twist.twist.linear.y = vy;
    twist.twist.linear.z = sqrt(vx*vx+vy*vy);
    twist_pub_.publish(twist);
}

void vel_estimator::callback(const opencv_apps::FlowArrayStamped::ConstPtr &msg_flow,
                             const sensor_msgs::Imu::ConstPtr &msg_imu,
                             const sensor_msgs::LaserScan::ConstPtr &msg_laser)
{
    if(!got_cam_info)
    {
        ROS_INFO_THROTTLE(1,"Cam Info not received, not processing");
        return;
    }

    ROS_INFO_ONCE("Main CB");
    ros::Time t = ros::Time::now();
    static bool first_image = true;
    double dt;
    if(first_image)
    {
        prev_time = msg_flow->header.stamp;
        first_image = false;
        dt = 1.0;
    }
    else
        dt = (msg_flow->header.stamp - prev_time).toSec();
    prev_time = msg_flow->header.stamp;

    if(msg_flow->flow.size() < 3)
        return;

    Eigen::MatrixXd vel_vec(msg_flow->flow.size(),2);
//    ROS_INFO_THROTTLE(2,"FLOW SIZE: %d time: %f",msg_flow->flow.size(), dt);

    double Z = msg_laser->ranges[0];
//    Z = 1.0;

    bool debug = 0;

    double wx = -msg_imu->angular_velocity.y;
    double wy = -msg_imu->angular_velocity.x;
    double wz = -msg_imu->angular_velocity.z * 0;
    for(size_t i = 0; i < msg_flow->flow.size() ; i++)
    {
        //FOR LK
        if(0)
        {
            vel_vec(i,0) = (msg_flow->flow[i].velocity.x/dt - wy*fy_ - wz*(msg_flow->flow[i].point.y-cy_))*Z/fx_;
            vel_vec(i,1) = (msg_flow->flow[i].velocity.y/dt - wx*fx_ + wz*(msg_flow->flow[i].point.x-cx_))*Z/fy_;
        }
        else
        {
        //FOR SIMPLE_FLOW
            vel_vec(i,0) = (msg_flow->flow[i].velocity.x/dt + wy*fy_ - wz*(msg_flow->flow[i].point.y-cy_))*Z/fx_;
            vel_vec(i,1) = (msg_flow->flow[i].velocity.y/dt + wx*fx_ + wz*(msg_flow->flow[i].point.x-cx_))*Z/fy_;
        }
        if(debug)
        {
            vel_vec(i,0) = msg_flow->flow[i].velocity.x/dt;
            vel_vec(i,1) = msg_flow->flow[i].velocity.y/dt;
        }
    }
    Eigen::MatrixXd mean = vel_vec.colwise().mean();
    double vx = mean(0);
    double vy = mean(1);

#define USE_FILTER
#ifdef USE_FILTER
    double current_speed = sqrt(vx*vx+vy*vy);

        if (speed_samples_.size() == N)
        {
            if ((fabs(current_speed-updated_speed) < vel_sigma) || (updated_speed == 0.0))
            {
                speed_samples_.pop_front();
                vx_samples_.pop_front();
                vy_samples_.pop_front();

                vx_samples_.push_back(vx);
                vy_samples_.push_back(vy);
                speed_samples_.push_back(current_speed);
            }
            else
            {
                std::deque<double> vx_temp_,vy_temp_;
                vx_temp_.resize(N);
                vy_temp_.resize(N);
                std::transform(vx_samples_.begin(), vx_samples_.end(), vx_samples_.begin(),std::bind1st(std::multiplies<double>(),alpha)); //decay
                std::transform(vy_samples_.begin(), vy_samples_.end(), vy_samples_.begin(),std::bind1st(std::multiplies<double>(),alpha));
                std::transform( vx_samples_.begin(), vx_samples_.end(), vx_samples_.begin(), vx_temp_.begin(), std::multiplies<double>() ); //
                std::transform( vy_samples_.begin(), vy_samples_.end(), vy_samples_.begin(), vy_temp_.begin(), std::multiplies<double>() );
                std::transform( vx_temp_.begin(), vx_temp_.end(), vy_temp_.begin(), speed_samples_.begin(), std::plus<double>());
                std::transform( speed_samples_.begin(), speed_samples_.end(), speed_samples_.begin(), static_cast<double (*)(double)>(std::sqrt));
            }
            double total_ = std::accumulate(speed_samples_.begin(), speed_samples_.end(), 0.0);
            updated_speed = total_/N;
            total_ = std::accumulate(vx_samples_.begin(), vx_samples_.end(), 0.0);
            vx = total_/N;
            total_ = std::accumulate(vy_samples_.begin(), vy_samples_.end(), 0.0);
            vy = total_/N;


        }
        else
        {
            speed_samples_.push_back(current_speed);
            vx_samples_.push_back(vx);
            vy_samples_.push_back(vy);
            vx = 0.0;
            vy = 0.0;
        }
#endif
    geometry_msgs::TwistStamped twist;
    twist.header = msg_flow->header;
    twist.twist.linear.x = vx;
    twist.twist.linear.y = vy;
    twist.twist.linear.z = sqrt(vx*vx+vy*vy);
//    std::cout<<std::endl<<twist.twist.linear.z;

    if(debug)//debug
    {
        twist.twist.linear.x = vx;
        twist.twist.linear.y = vy;
        twist.twist.angular.x = wx*fx_;
        twist.twist.angular.y = wy*fy_;
    }

    if(std::isfinite(twist.twist.linear.z));// && (dt >= 1/100.0))
    {
        twist_pub_.publish(twist);
//        if(twist.twist.linear.z > 1.0)
//            ROS_INFO("xy %f %f time: %f",mean(0),mean(1),dt);
//        ROS_INFO("xyz %f %f %f",wx,wy,wz);
        static float dist;// = 0;
        dist += twist.twist.linear.z *dt;
        ROS_INFO("Dist: %f Seq: %d",dist, msg_flow->header.seq);
    }
    return;
}

void vel_estimator::callback2(const sensor_msgs::Image::ConstPtr &msg_im,
                             const sensor_msgs::Imu::ConstPtr &msg_imu,
                             const sensor_msgs::LaserScan::ConstPtr &msg_laser)
{
    if(!got_cam_info)
    {
        ROS_INFO_THROTTLE(1,"Cam Info not received, not processing");
        return;
    }

    ROS_INFO_ONCE("Main CB2");

    double Z = msg_laser->ranges[0];

    double wx = -msg_imu->angular_velocity.y;
    double wy = -msg_imu->angular_velocity.x;
    double wz = -msg_imu->angular_velocity.z;

    ros::Time t = ros::Time::now();
    static bool first_image = true;
    double dt;
    if(first_image)
    {
        prev_time = msg_im->header.stamp;
        first_image = false;
        dt = 1.0;
    }
    else
        dt = (msg_im->header.stamp - prev_time).toSec();
    prev_time = msg_im->header.stamp;

    cv::Mat cv_im;
    cv_bridge::CvImageConstPtr cv_br_im;
    cv_br_im = cv_bridge::toCvShare(msg_im);
    cv_im = cv_br_im->image;
    float flow_x,flow_y;
    const uint32_t sec = (msg_im->header.stamp.sec);
    int usec = (msg_im->header.stamp.nsec/1000);
    int quality = of_obj->calcFlow(cv_im.data,sec,usec,flow_x,flow_y);

//    vel_vec(i,0) = (msg_flow->flow[i].velocity.x/dt - wy*fy_ - wz*(msg_flow->flow[i].point.y-cy_))*Z/fx_;
//    vel_vec(i,1) = (msg_flow->flow[i].velocity.y/dt - wx*fx_ + wz*(msg_flow->flow[i].point.x-cx_))*Z/fy_;

    double vx,vy;
    if(w_corr)
    {
        //Signs are correct
        vx = (flow_x/dt + wy*fy_)*Z/fx_;
        vy = (flow_y/dt - wx*fx_)*Z/fy_;
//        vx = (flow_x/dt )*Z/fx_ + wy;
//        vy = (flow_y/dt )*Z/fy_ - wx;

    }
    else
    {
        vx = (flow_x/dt)*Z/fx_;
        vy = (flow_y/dt)*Z/fy_;
    }

//    dt = sec+usec/10e6;

    ROS_INFO("Quality: %d FLOW: %f %f dt: %d.%d = %f",quality,flow_x,flow_y,sec,usec,dt);


#define USE_FILTER2
#ifdef USE_FILTER2
    double current_speed = sqrt(vx*vx+vy*vy);

        if (speed_samples_.size() == N)
        {
            if ((fabs(current_speed-updated_speed) < vel_sigma) || (updated_speed == 0.0))
            {
                speed_samples_.pop_front();
                vx_samples_.pop_front();
                vy_samples_.pop_front();

                vx_samples_.push_back(vx);
                vy_samples_.push_back(vy);
                speed_samples_.push_back(current_speed);
            }
            else
            {
                std::deque<double> vx_temp_,vy_temp_;
                vx_temp_.resize(N);
                vy_temp_.resize(N);
                std::transform(vx_samples_.begin(), vx_samples_.end(), vx_samples_.begin(),std::bind1st(std::multiplies<double>(),alpha)); //decay
                std::transform(vy_samples_.begin(), vy_samples_.end(), vy_samples_.begin(),std::bind1st(std::multiplies<double>(),alpha));
                std::transform( vx_samples_.begin(), vx_samples_.end(), vx_samples_.begin(), vx_temp_.begin(), std::multiplies<double>() ); //
                std::transform( vy_samples_.begin(), vy_samples_.end(), vy_samples_.begin(), vy_temp_.begin(), std::multiplies<double>() );
                std::transform( vx_temp_.begin(), vx_temp_.end(), vy_temp_.begin(), speed_samples_.begin(), std::plus<double>());
                std::transform( speed_samples_.begin(), speed_samples_.end(), speed_samples_.begin(), static_cast<double (*)(double)>(std::sqrt));
            }
            double total_ = std::accumulate(speed_samples_.begin(), speed_samples_.end(), 0.0);
            updated_speed = total_/N;
            total_ = std::accumulate(vx_samples_.begin(), vx_samples_.end(), 0.0);
            vx = total_/N;
            total_ = std::accumulate(vy_samples_.begin(), vy_samples_.end(), 0.0);
            vy = total_/N;


        }
        else
        {
            speed_samples_.push_back(current_speed);
            vx_samples_.push_back(vx);
            vy_samples_.push_back(vy);
            vx = 0.0;
            vy = 0.0;
        }
#endif
    geometry_msgs::TwistStamped twist;
    twist.header = msg_im->header;
    twist.twist.linear.x = vx;
    twist.twist.linear.y = vy;
    twist.twist.linear.z = sqrt(vx*vx+vy*vy);
    twist.twist.angular.x = wx;
    twist.twist.angular.y = wy;
    twist.twist.angular.z = wz;


    if(std::isfinite(twist.twist.linear.z));// && (dt >= 1/100.0))
    {
        twist_pub_.publish(twist);
//        if(twist.twist.linear.z > 1.0)
//            ROS_INFO("xy %f %f time: %f",mean(0),mean(1),dt);
//        ROS_INFO("xyz %f %f %f",wx,wy,wz);
        static float dist;// = 0;
        dist += twist.twist.linear.z *dt;
        ROS_INFO("Dist: %f Seq: %d",dist, msg_im->header.seq);
    }
    return;
}

vel_estimator::vel_estimator(ros::NodeHandle& nh){
    twist_pub_ = nh.advertise<geometry_msgs::TwistStamped>("twist", 10);
    cam_info_sub_ = nh.subscribe("/camera1/camera_info", 1,&vel_estimator::getCamInfo,this);
//    im_sub_ = nh.subscribe("/camera1/image_rect", 1,&vel_estimator::im_callback,this);
    updated_speed=0;
    alpha =0.95;
    vel_sigma=0.5;
    w_corr = true;
    N =10;
    nh.getParam("vel_sigma",vel_sigma);
    nh.getParam("buffer",N);
    nh.getParam("alpha",alpha);
    nh.getParam("w_corr",w_corr);
    ROS_INFO("into constr");
    got_cam_info = false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vel_estimator");
    //	cv::initModule_nonfree();//THIS LINE IS IMPORTANT for using surf and sift features of opencv
    ros::NodeHandle nh("~");
    vel_estimator d(nh);
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera1/image_rect", 1);
    message_filters::Subscriber<opencv_apps::FlowArrayStamped> flow_sub(nh, "/flownode/flows", 1);
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "/mavros/imu/data", 1);
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub(nh, "/sf30/range", 1);


    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Imu , sensor_msgs::LaserScan> MySyncPolicy;
//    typedef message_filters::sync_policies::ApproximateTime<opencv_apps::FlowArrayStamped, sensor_msgs::Imu , sensor_msgs::LaserScan> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
//    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), flow_sub, imu_sub, laser_sub);
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, imu_sub, laser_sub);
//    sync.registerCallback(boost::bind(&vel_estimator::callback,&d, _1, _2, _3));
    sync.registerCallback(boost::bind(&vel_estimator::callback2,&d, _1, _2, _3));
    ros::spin();
    return 0;
}
