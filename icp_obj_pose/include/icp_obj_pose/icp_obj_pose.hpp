#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>

class ICPNode : public rclcpp::Node {
public:
    ICPNode() : Node("icp_node") {
        // Declare parameters
        this->declare_parameter<std::string>("input_topic", "/input_pointcloud");
        this->declare_parameter<std::string>("reference_ply_filepath", "/path/to/your/file.ply");

        // Retrieve parameters
        std::string input_topic, file_path;
        this->get_parameter("input_topic", input_topic);
        this->get_parameter("reference_ply_filepath", file_path);

        // Publishers
        // auto qos = rclcpp::QoS::transient_local;
        pub_target_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("target_pointcloud", 10);
        pub_source_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("source_pointcloud", 10);
        pub_aligned_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("aligned_pointcloud", 10);
        pub_transform_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("icp_transform", 10);

        // Subscriber to input point cloud
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, 10, std::bind(&ICPNode::pointCloudCallback, this, std::placeholders::_1)
        );

        // Load the target point cloud from the .ply file
        target_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(file_path, *target_cloud_) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load .ply file: %s", file_path.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Initialized ICPNode, Subscribed to topic %s", input_topic.c_str());
    }

private:
    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;

    // Publishers for visualizing point clouds and transformations
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_source_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_target_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_aligned_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_transform_;

    // Point clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;

    // Callback function for receiving the source point cloud
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Started CALL");
        // Convert ROS PointCloud2 message to PCL point cloud
        source_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *source_cloud_);
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        voxel_grid.setInputCloud(source_cloud_);
        voxel_grid.setLeafSize(0.05f, 0.05f, 0.05f);  // Adjust voxel size if necessary
        voxel_grid.filter(*filtered_cloud);
        

        // Publish the source point cloud for visualization
        pub_source_->publish(*msg);
        RCLCPP_INFO(this->get_logger(), "MID CALL");

        // Perform ICP
        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        icp_.setInputSource(filtered_cloud);
        icp_.setMaximumIterations(1);
        icp_.setInputTarget(target_cloud_);
        icp_.align(*aligned_cloud);
        RCLCPP_INFO(this->get_logger(), "MId-Late CALL");

        // if (icp_.hasConverged()) {
        //     // Publish the aligned point cloud
        //     sensor_msgs::msg::PointCloud2 aligned_msg;
        //     pcl::toROSMsg(*aligned_cloud, aligned_msg);
        //     aligned_msg.header.frame_id = "map";
        //     pub_aligned_->publish(aligned_msg);

        //     // Extract and publish the transformation matrix as a PoseStamped
        //     Eigen::Matrix4f transformation = icp_.getFinalTransformation();
        //     publishTransform(transformation);
        // } else {
        //     RCLCPP_WARN(this->get_logger(), "ICP did not converge.");
        // }

        // Publish the target point cloud for visualization
        sensor_msgs::msg::PointCloud2::SharedPtr target_msg, aligned_msg, filtered_msg;
        target_msg.reset(new sensor_msgs::msg::PointCloud2());
        aligned_msg.reset(new sensor_msgs::msg::PointCloud2());
        filtered_msg.reset(new sensor_msgs::msg::PointCloud2());
        
        pcl::toROSMsg(*aligned_cloud, *aligned_msg);
        aligned_msg->header.frame_id = "world";
        pub_target_->publish(*aligned_msg);
        target_msg->header.frame_id = "world";
        pcl::toROSMsg(*target_cloud_, *target_msg);
        pub_target_->publish(*target_msg);

        filtered_msg->header.frame_id = "world";
        pcl::toROSMsg(*filtered_cloud, *filtered_msg);
        pub_target_->publish(*filtered_msg);
        
        RCLCPP_INFO(this->get_logger(), "Ran ICP CALL");
    }

    // Helper function to publish the transformation matrix as PoseStamped
    void publishTransform(const Eigen::Matrix4f& transformation) {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.frame_id = "map";
        pose_msg.header.stamp = this->get_clock()->now();

        // Extract translation and rotation from the transformation matrix
        pose_msg.pose.position.x = transformation(0, 3);
        pose_msg.pose.position.y = transformation(1, 3);
        pose_msg.pose.position.z = transformation(2, 3);

        Eigen::Matrix3f rotation_matrix = transformation.block<3, 3>(0, 0);
        Eigen::Quaternionf quaternion(rotation_matrix);
        pose_msg.pose.orientation.x = quaternion.x();
        pose_msg.pose.orientation.y = quaternion.y();
        pose_msg.pose.orientation.z = quaternion.z();
        pose_msg.pose.orientation.w = quaternion.w();

        pub_transform_->publish(pose_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ICPNode>());
    rclcpp::shutdown();
    return 0;
}