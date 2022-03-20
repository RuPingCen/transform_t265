/*
 * 从T265相机发布里程计数据的Topic上接收数据，转发为path类型，并存储为txt格式数据
 * cenruping@vip.qq.com
   
 */
#include <ros/ros.h>
#include <ros/package.h>

#include <map>
#include <set>
#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <iostream>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
 
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <fstream>

using namespace std;
using namespace Eigen;

Eigen::Isometry3d mocap_initial_frame;

nav_msgs::Path  path_msg;
ros::Publisher path_pub;
string sub_topic,pub_topic;
string save_path;
string save_model;
bool is_save_path;
string fixed_frame_id = "world";
string child_frame_id = "odom";
vector<string> global_path;

void T265CameraOdomCallback(const nav_msgs::OdometryConstPtr& msg) ;
void savePathToFile(string& save_path,geometry_msgs::PoseStamped& tcw_msg,vector<double>& PP);

int main(int argc, char** argv) 
{
 
    ros::init(argc, argv, "T265Transform");
    ros::NodeHandle nh("~");

    nh.param<std::string>("sub_topic", sub_topic, "/camera/odom/sample");
    nh.param<std::string>("pub_topic", pub_topic, "t265_camera/path");
    nh.param<bool>("is_save_path", is_save_path, false);
    string path_packa = ros::package::getPath("transform_t265");
    nh.param<std::string>("save_path", save_path, path_packa+"/t265_path.txt");
    nh.param<std::string>("save_model", save_model, "tum");



    nh.param<std::string>("fixed_frame_id", fixed_frame_id, "world");
    nh.param<std::string>("child_frame_id", child_frame_id, "odom");
 

    ROS_INFO_STREAM("sub_topic:   "<<sub_topic);
    ROS_INFO_STREAM("pub_topic:   "<<pub_topic);
    ROS_INFO_STREAM("is_save_path:   "<<is_save_path);
    ROS_INFO_STREAM("save_path:   "<<save_path);
    ROS_INFO_STREAM("save_model:   "<<save_model);
 
    if (access(save_path.c_str(), 0) == 0)
    {
        if (remove(save_path.c_str()) == 0)
        {
            ROS_WARN_STREAM("file is exist, now remove it!");
        }
        else
        {
            ROS_ERROR_STREAM("file remove error ...");
        }
    }

    ofstream foutC(save_path, ios::app);
    foutC.setf(ios::fixed, ios::floatfield);

    if(save_model == "euroc")
    {
        ROS_INFO_STREAM("timestamp(s) tx ty tz qw qx qy qz ");
        foutC << "# "<< save_model << " type: timestamp(s) tx ty tz qw qx qy qz "<< endl;
    }
    else if(save_model == "tum")
    {
        ROS_INFO_STREAM("timestamp(s) tx ty tz qx qy qz qw ");
        foutC << "# "<<save_model << " type: timestamp(s) tx ty tz qx qy qz qw "<< endl;
    }
    else if(save_model == "openvins")
    {
        ROS_INFO_STREAM("timestamp(s) tx ty tz qx qy qz qw Pr11 Pr12 Pr13 Pr22 Pr23 Pr33 Pt11 Pt12 Pt13 Pt22 Pt23 Pt33");
        foutC << "# "<<save_model << " type: timestamp(s) tx ty tz qx qy qz qw Pr11 Pr12 Pr13 Pr22 Pr23 Pr33 Pt11 Pt12 Pt13 Pt22 Pt23 Pt33"<< endl;
    }



    foutC.close();

    ros::Subscriber T265_odom_sub = nh.subscribe(sub_topic, 10, T265CameraOdomCallback);
    path_pub= nh.advertise<nav_msgs::Path> (pub_topic, 10); 
    ros::spin();
    
    return  1;
       
}
 
void T265CameraOdomCallback(const nav_msgs::OdometryConstPtr& msg) 
{
    tf::TransformBroadcaster tf_pub;
    
    static bool first_mocap_odom_msg = true;

  // If this is the first mocap odometry messsage, set
  // the initial frame.
  if (first_mocap_odom_msg) 
  {
    Quaterniond orientation;
    Vector3d translation;
    tf::pointMsgToEigen(msg->pose.pose.position, translation);
    tf::quaternionMsgToEigen(msg->pose.pose.orientation, orientation);
 
    mocap_initial_frame.linear() = orientation.toRotationMatrix();
    mocap_initial_frame.translation() = translation;
    first_mocap_odom_msg = false;
  }

  // Transform the ground truth.
  Quaterniond orientation;
  Vector3d translation;
 
  tf::pointMsgToEigen(msg->pose.pose.position, translation);
  tf::quaternionMsgToEigen(msg->pose.pose.orientation, orientation);

  Eigen::Isometry3d T_b_v_gt;
  T_b_v_gt.linear() = orientation.toRotationMatrix();
  T_b_v_gt.translation() = translation;
  Eigen::Isometry3d T_b_w_gt = mocap_initial_frame.inverse() * T_b_v_gt;
 
   
//   {
//     tf::Transform T_b_w_gt_tf;
//     tf::transformEigenToTF(T_b_w_gt, T_b_w_gt_tf);
//     tf_pub.sendTransform(tf::StampedTransform(
//           T_b_w_gt_tf, msg->header.stamp, fixed_frame_id, child_frame_id+"_mocap"));
//   }
  
    nav_msgs::Odometry mocap_odom_msg;
    tf::poseEigenToMsg(T_b_w_gt, mocap_odom_msg.pose.pose);
 
    geometry_msgs::PoseStamped tcw_msg; 
    tcw_msg.header=msg->header;
    tcw_msg.header.frame_id = fixed_frame_id;
    //tcw_msg.pose = mocap_odom_msg.pose.pose;
    tcw_msg.pose = msg->pose.pose;
    path_msg.header = msg->header;
    path_msg.header.frame_id = fixed_frame_id;
    path_msg.poses.push_back(tcw_msg); 
    path_pub.publish(path_msg);

    vector<double> PP;
    PP.reserve(12);
    PP.push_back(msg->pose.covariance[0]);
    PP.push_back(msg->pose.covariance[1]);
    PP.push_back(msg->pose.covariance[2]);
    PP.push_back(msg->pose.covariance[7]);
    PP.push_back(msg->pose.covariance[8]);
    PP.push_back(msg->pose.covariance[14]);
    PP.push_back(msg->pose.covariance[21]);
    PP.push_back(msg->pose.covariance[22]);
    PP.push_back(msg->pose.covariance[23]);
    PP.push_back(msg->pose.covariance[28]);
    PP.push_back(msg->pose.covariance[29]);
    PP.push_back(msg->pose.covariance[35]);

    if(is_save_path)
    {
        savePathToFile(save_path,tcw_msg,PP);	
    }
  return;
}
void savePathToFile(string& save_path,geometry_msgs::PoseStamped& tcw_msg,vector<double>& PP)
{
	// write result to file
        ofstream foutC(save_path, ios::app);
        foutC.setf(ios::fixed, ios::floatfield);
        foutC.precision(9);

        foutC << tcw_msg.header.stamp.toSec() << " ";
        foutC.precision(5);
        if(save_model == "euroc")
        {
        foutC << tcw_msg.pose.position.x << " "
              << tcw_msg.pose.position.y << " "
              << tcw_msg.pose.position.z << " "
              << tcw_msg.pose.orientation.w << " "
              << tcw_msg.pose.orientation.x << " "
              << tcw_msg.pose.orientation.y << " "
              << tcw_msg.pose.orientation.z << endl;
        }
        else if(save_model == "tum")
       {
        foutC << tcw_msg.pose.position.x << " "
              << tcw_msg.pose.position.y << " "
              << tcw_msg.pose.position.z << " "
              << tcw_msg.pose.orientation.x << " "
              << tcw_msg.pose.orientation.y << " "
              << tcw_msg.pose.orientation.z << " "
              << tcw_msg.pose.orientation.w << endl;
        }
        else if(save_model == "openvins")
       {
        foutC << tcw_msg.pose.position.x << " "
              << tcw_msg.pose.position.y << " "
              << tcw_msg.pose.position.z << " "
              << tcw_msg.pose.orientation.x << " "
              << tcw_msg.pose.orientation.y << " "
              << tcw_msg.pose.orientation.z << " "
              << tcw_msg.pose.orientation.w << " "
              << PP[0]<< " "<< PP[1]<< " "<< PP[2]<< " "<< PP[3]<< " "<< PP[4]<< " "<< PP[5]<< " "
              << PP[6]<< " "<< PP[7]<< " "<< PP[8]<< " "<< PP[9]<< " "<< PP[10]<< " "<< PP[11]<<endl;
        }
        foutC.close();
}
