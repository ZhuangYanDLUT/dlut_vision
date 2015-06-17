#include<iostream>
#include"ros/ros.h"
#include "nav_msgs/Odometry.h"
//用于打开pcd
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include "pcl_ros/transforms.h"


#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace testPointCloud
{
    using namespace message_filters;
    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;
    class tp
    {
        public:
            tp(void ):
                cloud_sum(new pcl::PointCloud<pcl::PointXYZ>),
                count_point(0)
            {
                point_sub_.subscribe(n,"/multisense/organized_image_points2",5);
                vo_sub_.subscribe(n,"/stereo_odometer/odometry",1000);

                my_sync_.reset(new AppSync(MySyncPolicy(5),point_sub_, vo_sub_));
                my_sync_->registerCallback(boost::bind(&tp::callback, this, _1, _2));
            }
        private:
            void callback(const sensor_msgs::PointCloud2ConstPtr& msg,const nav_msgs::OdometryConstPtr& vo);

            ros::NodeHandle n;
            ros::Subscriber m_subscriber_;
            tf::TransformListener mylistener;
            tf::StampedTransform transform;
            tf::Transform vo_meas_;

            message_filters::Subscriber<sensor_msgs::PointCloud2> point_sub_;
            message_filters::Subscriber<nav_msgs::Odometry> vo_sub_;
            typedef message_filters::Synchronizer<MySyncPolicy> AppSync;
            boost::shared_ptr<AppSync> my_sync_;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sum;

            pcl::PCDWriter writer;
            int count_point;
    };

    void tp::callback(const sensor_msgs::PointCloud2ConstPtr& msg, const nav_msgs::OdometryConstPtr& vo)
    {
        ROS_INFO("Callback %s",vo->header.frame_id.c_str());

        sensor_msgs::PointCloud2 transformed_PointCloud2;
        poseMsgToTF(vo->pose.pose, vo_meas_);
        tf::Transform vo_meas_tran;


        //坐标轴变化
        double Rx, Ry, Rz, tx, ty, tz;
        vo_meas_.getBasis().getEulerYPR(Rz, Ry, Rx);
        ROS_INFO("%f,%f,%f",Rx,Ry,Rz);
        tx = vo_meas_.getOrigin().x();
        ty = vo_meas_.getOrigin().y();
        tz = vo_meas_.getOrigin().z();

        tf::Quaternion q(Rz,Ry,Rx);
        vo_meas_tran.setOrigin(tf::Vector3(tx,ty,tz));
        vo_meas_tran.setRotation(q);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_fil (new pcl::PointCloud<pcl::PointXYZ>);


        pcl_ros::transformPointCloud("odom", vo_meas_tran.inverse(), *msg, transformed_PointCloud2);
        pcl::fromROSMsg(transformed_PointCloud2, *cloud);
        /*
         *Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
         *transformation_matrix (0,3) = tx; transformation_matrix (1,3) = ty;transformation_matrix (2,3) = tz;
         *pcl::transformPointCloud (*cloud, *cloud, transformation_matrix);
         */

        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(0.01f,0.01f,0.01f);
        sor.filter(*cloud_fil);
        *cloud_sum += *cloud_fil;

        if(count_point == 6)
            writer.write("sum.pcd",*cloud_sum);

        count_point++;
    }
};
int main(int argc,char** argv)
{
    ros::init(argc,argv,"test_point");
    testPointCloud::tp tp;
    ros::spin();
}
