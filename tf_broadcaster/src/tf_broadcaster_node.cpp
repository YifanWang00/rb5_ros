#include <ros/ros.h>
#include <ros/console.h>

#include "tf_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "rb5_message/rb5_message.h"
#include "april_detection/AprilTagDetection.h"
#include "april_detection/AprilTagDetectionArray.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ros::Subscriber apriltag_sub;
ros::Publisher cam2map_pub;
rb5_message::rb5_message state_msg;
tf2::Transform map_to_cam_tf2;


void detectionArrayCallback(april_detection::AprilTagDetectionArray detArray_msg){
    tf2_ros::TransformBroadcaster tfBroadcaster;
    
    for (const auto& detection : detArray_msg.detections) {
        int marker_id = detection.id;
        std::string marker_frame = "marker_" + std::to_string(marker_id);
        // ROS_INFO("Receieved detection of marker_%d", marker_id);

    
        tf2_ros::Buffer tfBuffer_tag_to_cam;
        tf2_ros::TransformListener tfListener_tag_to_cam(tfBuffer_tag_to_cam);
        geometry_msgs::TransformStamped tag_to_cam_msg;
        try {
            tag_to_cam_msg = tfBuffer_tag_to_cam.lookupTransform(marker_frame, "camera", ros::Time(0), ros::Duration(1.0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }

        tf2_ros::Buffer tfBuffer_map_to_tag;
        tf2_ros::TransformListener tfListener_map_2_tag(tfBuffer_map_to_tag);
        geometry_msgs::TransformStamped map_to_tag_msg;
        try {
            map_to_tag_msg = tfBuffer_map_to_tag.lookupTransform("map", marker_frame, ros::Time(0), ros::Duration(1.0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return;
        }        

        // Create a quaternion that rotates 90 degrees around the Y-axis
        tf2::Quaternion rotation_quaternion;
        rotation_quaternion.setRPY(0, M_PI_2, 0);

        // Create a transformation matrix to represent the rotation
        tf2::Transform rotation_transform;
        rotation_transform.setRotation(rotation_quaternion);

        geometry_msgs::TransformStamped map_to_cam_transform;
        map_to_cam_transform.header.stamp = ros::Time::now();
        map_to_cam_transform.header.frame_id = "map";
        map_to_cam_transform.child_frame_id = "robot";
        tf2::Transform map_to_tag_tf2;
        tf2::fromMsg(map_to_tag_msg.transform, map_to_tag_tf2);
        tf2::Transform tag_to_cam_tf2;
        tf2::Transform tag_to_cam_tf2_corrected;
        tf2::fromMsg(tag_to_cam_msg.transform, tag_to_cam_tf2);
        tag_to_cam_tf2_corrected = rotation_transform * tag_to_cam_tf2;
        map_to_cam_tf2 = map_to_tag_tf2 * tag_to_cam_tf2;
        // map_to_cam_tf2 = map_to_tag_tf2 * tag_to_cam_tf2_corrected;
        map_to_cam_transform.transform = tf2::toMsg(map_to_cam_tf2);

        tfBroadcaster.sendTransform(map_to_cam_transform);
    }
}

void publishCameraToMapTransform(const ros::TimerEvent&) {
    /*
    As robot tf is rotated, the correspondence between robot tf and real world robot is:
    robot tf: (x, y, z) ===> real-world robot: (-z, y, -x)
    */
    ROS_INFO("111111");
    // state_msg.data[0] = static_cast<float>(-map_to_cam_tf2.getOrigin().z());
    state_msg.data[0] = static_cast<float>(map_to_cam_tf2.getOrigin().x());
    if(std::isnan(state_msg.data[0])) {
        return;
    }
    state_msg.data[1] = static_cast<float>(map_to_cam_tf2.getOrigin().y());
    if(std::isnan(state_msg.data[1])) {
        return;
    }
    // state_msg.data[2] = static_cast<float>(-map_to_cam_tf2.getOrigin().x());
    state_msg.data[2] = static_cast<float>(-map_to_cam_tf2.getOrigin().z());
    if(std::isnan(state_msg.data[2])) {
        return;
    }
    ROS_INFO("Publishing msg (%f, %f, %f)", state_msg.data[0], state_msg.data[1], state_msg.data[2]);
    
    cam2map_pub.publish(state_msg);
}


int main(int argc, char *argv[]){
    ros::init(argc, argv, "tf_broadcaster");
    ros::NodeHandle n;
    ros::NodeHandle private_nh("~");
    
    apriltag_sub = n.subscribe("/apriltag_detection_array", 1, detectionArrayCallback);
    cam2map_pub = n.advertise<rb5_message::rb5_message>("/rb5_state_topic", 10);

    ros::Timer timer = n.createTimer(ros::Duration(1), publishCameraToMapTransform);
    
    ros::spin();
    return 0;
}