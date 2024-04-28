#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class PointCloudTransformer {
public:
    PointCloudTransformer() {
        // Initialize ROS node handle
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");
        // Get parameters from the parameter server or use defaults
        private_nh.param("input_topic", input_topic_, std::string("/livox/lidar"));
        private_nh.param("output_topic", output_topic_, std::string("/livox/lidar_inv"));
        private_nh.param("input_frame", input_frame_, std::string("livox_frame"));
        private_nh.param("output_frame", output_frame_, std::string("livox_frame_inv"));
        // Publisher for the transformed PointCloud
        pub_ = nh.advertise<sensor_msgs::PointCloud2>(output_topic_, 10);
        // Subscriber to the original PointCloud
        sub_ = nh.subscribe(input_topic_, 10, &PointCloudTransformer::callback, this);
        // Initialize transform
        transform_.header.frame_id = input_frame_;
        transform_.child_frame_id = output_frame_;
        transform_.transform.rotation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), M_PI)); // 180 degrees around Z-axis
    }

    void callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        sensor_msgs::PointCloud2 out_msg;
        // Set the transform timestamp to the message timestamp
        transform_.header.stamp = msg->header.stamp;
        // Transform the PointCloud
        tf2::doTransform(*msg, out_msg, transform_);
        // Set the new frame ID
        out_msg.header.frame_id = output_frame_;
        // Publish the transformed PointCloud
        pub_.publish(out_msg);
        // Optionally, broadcast the new frame
        static tf2_ros::TransformBroadcaster br;
        br.sendTransform(transform_);
    }

private:
    ros::Publisher pub_;
    ros::Subscriber sub_;
    geometry_msgs::TransformStamped transform_;
    std::string input_topic_, output_topic_, input_frame_, output_frame_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_transform_node");
    PointCloudTransformer transformer;
    ros::spin();
    return 0;
}
