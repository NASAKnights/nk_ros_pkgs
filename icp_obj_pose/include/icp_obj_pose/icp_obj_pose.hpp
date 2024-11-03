#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

class ICPNode : public rclcpp::Node {
public:
    ICPNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : 
    Node("icp_node", options)
    ,tf_buffer_(this->get_clock())
    ,tf_listener_(tf_buffer_)
    {
        initializeParameters();
        getParameters();
        
        // Retrieve parameters
        std::string input_topic, file_path;
        this->get_parameter("input_topic", input_topic);
        this->get_parameter("reference_ply_filepath", file_path);

        // Publishers
        // auto qos = rclcpp::QoS::transient_local;
        pub_object_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("object_pointcloud", 10);
        pub_source_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("source_pointcloud", 10);
        pub_aligned_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("aligned_pointcloud", 10);
        pub_transform_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("tf", 10);

        // Subscriber to input point cloud
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, 10, std::bind(&ICPNode::pointCloudCallback, this, std::placeholders::_1)
        );

        // Load the target point cloud from the .ply file
        object_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(file_path, *object_cloud_) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load .ply file: %s", file_path.c_str());
            throw std::runtime_error("PCL FAILED TO LOAD");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Initialized ICPNode, Subscribed to topic %s", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Object has points: %d", object_cloud_->size());
        if(_voxel_size > 0.0)
        {
            pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
            voxel_grid.setInputCloud(object_cloud_);
            voxel_grid.setLeafSize(_voxel_size, _voxel_size, _voxel_size);  // Adjust voxel size if necessary
            voxel_grid.filter(*object_cloud_);
            RCLCPP_INFO(this->get_logger(), "Voxelized object has points: %d", object_cloud_->size());
        }
    }

private:
    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    Eigen::Matrix4f _object_pose;
    // Publishers for visualizing point clouds and transformations
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_source_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_object_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_aligned_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr pub_transform_;

    // Point clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    geometry_msgs::msg::TransformStamped last_camera_tf_;
    std::string _camera_frame_name, _object_frame_name;

    float _x_min, _x_max, _y_min, _y_max, _z_min, _z_max, _voxel_size;
    int _hue_min, _hue_max, _saturation_min, _saturation_max, _value_min, _value_max, _colorVariance;

    void getParameters()
    {
        // Declare parameters
        this->declare_parameter<std::string>("input_topic", "/input_pointcloud");
        this->declare_parameter<std::string>("reference_ply_filepath", "/path/to/your/file.ply");
        this->declare_parameter<std::string>("camera_frame", "camera_1");
        this->declare_parameter<std::string>("object_name", "note");
        this->declare_parameter<float>("voxel_size", 0.0);

        // Declare parameters for XYZ bounds
        this->declare_parameter<float>("x_min", -INFINITY);
        this->declare_parameter<float>("x_max", INFINITY);
        this->declare_parameter<float>("y_min", -INFINITY);
        this->declare_parameter<float>("y_max", INFINITY);
        this->declare_parameter<float>("z_min", -INFINITY);
        this->declare_parameter<float>("z_max", INFINITY);
        
        // Declare parameters for HSV bounds
        this->declare_parameter<float>("hue_min", 0.0);
        this->declare_parameter<float>("hue_max", 360.0);
        this->declare_parameter<float>("saturation_min", 0.0);
        this->declare_parameter<float>("saturation_max", 1.0);
        this->declare_parameter<float>("value_min", 0.0);
        this->declare_parameter<float>("value_max", 1.0);

    }
    void initializeParameters()
    {
        // Retrieve parameters for XYZ bounds
        this->get_parameter("x_min", _x_min);
        this->get_parameter("x_max", _x_max);
        this->get_parameter("y_min", _y_min);
        this->get_parameter("y_max", _y_max);
        this->get_parameter("z_min", _z_min);
        this->get_parameter("z_max", _z_max);
        
        // Retrieve parameters for HSV bounds
        this->get_parameter("hue_min", _hue_min);
        this->get_parameter("hue_max", _hue_max);
        this->get_parameter("saturation_min", _saturation_min);
        this->get_parameter("saturation_max", _saturation_max);
        this->get_parameter("value_min", _value_min);
        this->get_parameter("value_max", _value_max);

        this->get_parameter("voxel_size", _voxel_size);
        this->get_parameter("camera_frame", _camera_frame_name);
        this->get_parameter("object_name", _object_frame_name);

    }
    // Callback function for receiving the source point cloud
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Convert ROS PointCloud2 message to PCL point cloud
        auto test = pcl::PointCloud<pcl::PointXYZRGB>::Ptr();
        pcl::fromROSMsg(*msg, *test);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        filterColorAndDistance(test, filtered_cloud);

        updateEstimate();
        // source_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        // pcl::fromROSMsg(*msg, *source_cloud_);
        // pcl::fromROSMsg(*msg, *source_cloud_);
        // filterColor(source_cloud_);
        // filterDistance(source_cloud_);
        if(_voxel_size > 0.0)
        {
            pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
            voxel_grid.setInputCloud(filtered_cloud);
            voxel_grid.setLeafSize(_voxel_size, _voxel_size, _voxel_size);  // Adjust voxel size if necessary
            voxel_grid.filter(*filtered_cloud);
        }
        
        // Perform ICP
        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        icp_.setInputSource(object_cloud_);
        icp_.setMaximumIterations(10);
        icp_.setInputTarget(filtered_cloud);
        icp_.align(*aligned_cloud);

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
        sensor_msgs::msg::PointCloud2::SharedPtr object_msg, aligned_msg;
        object_msg.reset(new sensor_msgs::msg::PointCloud2());
        aligned_msg.reset(new sensor_msgs::msg::PointCloud2());
        
        pcl::toROSMsg(*filtered_cloud, *object_msg);
        object_msg->header.frame_id = _camera_frame_name;
        pub_object_->publish(*object_msg);
        
        pcl::toROSMsg(*aligned_cloud, *aligned_msg);
        aligned_msg->header.frame_id = _camera_frame_name;
        pub_aligned_->publish(*aligned_msg);

        _object_pose = icp_.transformation_;
        publishTransform(_object_pose);

    }

    void filterColorAndDistance(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud)
    {
        // Create a new point cloud to store the filtered results
        out_cloud->clear();
        out_cloud->reserve(in_cloud->size());
        cv::Mat rgb(1, 1, CV_8UC3);
        cv::Mat hsv(1, 1, CV_8UC3);

        // Iterate through each point in the input cloud
        for (const auto& point : in_cloud->points) {
            // Check conditions
            if (point.x >= _x_min && point.x <= _x_max &&
                point.y >= _y_min && point.y <= _y_max &&
                point.z >= _z_min && point.z <= _z_max) 
            {
                rgb.at<cv::Vec3b>(0, 0) = cv::Vec3b(point.b, point.g, point.r); // OpenCV uses BGR format

                // Convert RGB to HSV
                cv::cvtColor(rgb, hsv, cv::COLOR_RGB2HSV);

                // Extract HSV values
                int hue = hsv.at<cv::Vec3b>(0, 0)[0];
                float saturation = hsv.at<cv::Vec3b>(0, 0)[1]; 
                float value = hsv.at<cv::Vec3b>(0, 0)[2];      
                // Check color conditions (both RGB and HSV)
                if (hue >= _hue_min && hue <= _hue_max &&
                    saturation >= _saturation_min && saturation <= _saturation_max &&
                    value >= _value_min && value <= _value_max) 
                {
                    // If conditions are met, add the point to the output cloud
                    out_cloud->emplace_back(point.x, point.y, point.z);
                }

            }
        }
        out_cloud->header = in_cloud->header; // Copy the header from the input cloud
    }

    void filterDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        // Filter point cloud by color range (e.g., keeping only red-ish points)
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, 2.0);  // Adjust distance limits as needed
        pass.filter(*cloud);
    }

    /**
     * Update the estimated pose of the object based on the assumption that the camera
     * has moved and the target has not
     */
    void updateEstimate()
    {
        try {
            // Find transform of camera frame from prev to current in world frame
            auto current_tf = tf_buffer_.lookupTransform("world", "camera_link", tf2::TimePointZero);

            if (last_camera_tf_.header.stamp.sec != 0) {
                // Compute the relative transformation from the last known transform
                tf2::Transform last_tf, current_tf2, relative_tf;
                tf2::fromMsg(last_camera_tf_.transform, last_tf);
                tf2::fromMsg(current_tf.transform, current_tf2);
                relative_tf = last_tf.inverse() * current_tf2;

                // Update the object pose with the relative transformation
                Eigen::Matrix4f relative_transform;
                // tf2::;
                // _object_pose = relative_transform * _object_pose;
            }
            last_camera_tf_ = current_tf;  // Update the last known transform
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        }
    }


    // Helper function to publish the transformation matrix as PoseStamped
    void publishTransform(const Eigen::Matrix4f& transformation) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.child_frame_id = _object_frame_name;
        tf_msg.header.frame_id = _camera_frame_name;
        tf_msg.header.stamp = this->get_clock()->now();

        // Extract translation and rotation from the transformation matrix
        tf_msg.transform.translation.x = transformation(0, 3);
        tf_msg.transform.translation.y = transformation(1, 3);
        tf_msg.transform.translation.z = transformation(2, 3);

        Eigen::Matrix3f rotation_matrix = transformation.block<3, 3>(0, 0);
        Eigen::Quaternionf quaternion(rotation_matrix);
        tf_msg.transform.rotation.x = quaternion.x();
        tf_msg.transform.rotation.y = quaternion.y();
        tf_msg.transform.rotation.z = quaternion.z();
        tf_msg.transform.rotation.w = quaternion.w();

        pub_transform_->publish(tf_msg);
    }
};

RCLCPP_COMPONENTS_REGISTER_NODE(ICPNode)

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ICPNode>());
    rclcpp::shutdown();
    return 0;
}