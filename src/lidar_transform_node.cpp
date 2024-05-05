#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

class PointCloudTransformer {
public:
    PointCloudTransformer() {
        ros::NodeHandle nh;

        if (ros::param::get("~frame_override", frame_override)) {
            ROS_INFO("Overriding frame_id to %s", frame_override.c_str());
        } else {
            ROS_INFO("Preserving input frame_id");
        }

        pc_pub = nh.advertise<sensor_msgs::PointCloud2>("livox/lidar/transformed", 10);
        pc_sub = nh.subscribe("livox/lidar", 10, &PointCloudTransformer::pc_callback, this);
    }

private:
    std::string frame_override;
    ros::Publisher pc_pub;
    ros::Subscriber pc_sub;

    void pc_callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        // Rotate the point cloud along the z-axis by 180 degrees
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform(0, 0) = -1.0;
        transform(1, 1) = -1.0;
        sensor_msgs::PointCloud2 msg_rotated;
        pcl_ros::transformPointCloud(transform, *msg, msg_rotated);

        // Remove points within box x = -1.0~0.0, y = -0.4~+0.4, z = -inf~+inf
        pcl::PCLPointCloud2::Ptr pc_in = boost::make_shared<pcl::PCLPointCloud2>();
        pcl_conversions::toPCL(msg_rotated, *pc_in);
        pcl::CropBox<pcl::PCLPointCloud2> crop_filter;
        crop_filter.setNegative(true);
        crop_filter.setMin({-1.0, -0.4, -1.0, 0.0});
        crop_filter.setMax({0.0, 0.4, 1.0, 0.0});
        crop_filter.setInputCloud(pc_in);
        pcl::PCLPointCloud2 pc_out;
        crop_filter.filter(pc_out);

        sensor_msgs::PointCloud2 transformed_pc;
        pcl_conversions::fromPCL(pc_out, transformed_pc);
        transformed_pc.header = msg->header;

        if (!frame_override.empty()) {
            transformed_pc.header.frame_id = frame_override;
        }

        pc_pub.publish(transformed_pc);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_transform_node");
    PointCloudTransformer transformer;
    ros::spin();
    return 0;
}
