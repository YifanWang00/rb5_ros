#include <ros/ros.h>
#include <ros/console.h>

#include "tf_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "april_detection/AprilTagDetection.h"
#include "april_detection/AprilTagDetectionArray.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


void detectionArrayCallback(april_detection::AprilTagDetectionArray detArray_msg){
    tf2_ros::TransformBroadcaster tfBroadcaster;
    
    for (const auto& detection : detArray_msg.detections) {
        int marker_id = detection.id;
        std::string marker_frame = "marker_" + std::to_string(marker_id);
        ROS_INFO("Receieved detection of marker_%d", marker_id);

    
        tf2_ros::Buffer tfBuffer_camera2marker;
        tf2_ros::TransformListener tfListener_camera2marker(tfBuffer_camera2marker);
        geometry_msgs::TransformStamped camera_to_tag_transform;
        try {
            camera_to_tag_transform = tfBuffer_camera2marker.lookupTransform("camera", marker_frame, ros::Time(0), ros::Duration(1.0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }

        tf2_ros::Buffer tfBuffer_map2marker;
        tf2_ros::TransformListener tfListener_map2marker(tfBuffer_map2marker);
        geometry_msgs::TransformStamped tag_to_world_transform;
        try {
            tag_to_world_transform = tfBuffer_map2marker.lookupTransform("map", marker_frame, ros::Time(0), ros::Duration(1.0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return;
        }        

        geometry_msgs::TransformStamped camera_to_world_transform;
        camera_to_world_transform.header.stamp = ros::Time::now();
        camera_to_world_transform.header.frame_id = "map";
        camera_to_world_transform.child_frame_id = "camera";
        tf2::Transform tag_to_world_tf2;
        tf2::fromMsg(tag_to_world_transform.transform, tag_to_world_tf2);
        tf2::Transform camera_to_tag_tf2;
        tf2::fromMsg(camera_to_tag_transform.transform, camera_to_tag_tf2);
        ROS_INFO("Calculating camera_to_world TF...");
        camera_to_world_transform.transform = tf2::toMsg(
            tag_to_world_tf2 * camera_to_tag_tf2.inverse()
        );

        tfBroadcaster.sendTransform(camera_to_world_transform);
    }
}


int main(int argc, char *argv[]){
    ros::init(argc, argv, "tf_broadcaster");
    ros::NodeHandle n;
    ros::NodeHandle private_nh("~");
    
    ros::Subscriber apriltag_sub;
    apriltag_sub = n.subscribe("/apriltag_detection_array", 1, detectionArrayCallback);
    
    ros::spin();
    return 0;
}