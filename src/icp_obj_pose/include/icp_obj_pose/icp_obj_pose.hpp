#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#define TIME_FUNCTION(logger, func_name)                                                           \
    {                                                                                              \
        auto start = std::chrono::high_resolution_clock::now();                                    \
        func_name;                                                                                 \
        auto end = std::chrono::high_resolution_clock::now();                                      \
        auto duration =                                                                            \
            std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();            \
        RCLCPP_INFO(logger, "%s duration: %ld ms", #func_name, duration);                          \
    }

namespace icp_object_pose
{
class ICPNode : public rclcpp::Node
{
public:
    ICPNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("icp_node", options)
        , tf_buffer_(this->get_clock())
        , tf_listener_(tf_buffer_)
        , _object_pose(Eigen::Matrix4f::Identity())
    {
        initializeParameters();
        getParameters();

        // Retrieve parameters
        std::string file_path;
        this->get_parameter("reference_ply_filepath", file_path);

        // Set up subscribers for depth and color image topics
        depth_subscriber_.subscribe(this, _pointcloud_topic);
        image_subscriber_.subscribe(this, _image_topic);

        // Synchronize depth and image topics, with a queue size of 1
        sync_.reset(new message_filters::Synchronizer<MySyncPolicy>(
            MySyncPolicy(1), depth_subscriber_, image_subscriber_));
        sync_->registerCallback(
            std::bind(&ICPNode::callback, this, std::placeholders::_1, std::placeholders::_2));

        pub_filtered_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_pointcloud", 10);
        pub_aligned_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("aligned_pointcloud", 10);
        _transform_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Load the target point cloud from the .ply file
        object_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        if(pcl::io::loadPLYFile<pcl::PointXYZRGB>(file_path, *object_cloud_) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load .ply file: %s", file_path.c_str());
            throw std::runtime_error("PCL FAILED TO LOAD");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Initialized ICPNode, Subscribed to pointcloud topic: %s",
                    _pointcloud_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Initialized ICPNode, Subscribed to image topic: %s",
                    _image_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Object has %d points.", object_cloud_->size());
        if(_voxel_size > 0.0)
        {
            pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
            voxel_grid.setInputCloud(object_cloud_);
            voxel_grid.setLeafSize(_voxel_size, _voxel_size,
                                   _voxel_size); // Adjust voxel size if necessary
            voxel_grid.filter(*object_cloud_);
            RCLCPP_INFO(this->get_logger(), "Voxelized object has points: %d",
                        object_cloud_->size());
        }
        // Configure ICP
        icp_.setRANSACIterations(50);
        icp_.setRANSACOutlierRejectionThreshold(0.01);
        icp_.setMaximumIterations(500);
        icp_.setEuclideanFitnessEpsilon(0.01);
        icp_.setInputSource(object_cloud_);
    }

private:
    using MySyncPolicy =
        message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2,
                                                        sensor_msgs::msg::Image>;
    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2>   depth_subscriber_;
    message_filters::Subscriber<sensor_msgs::msg::Image>         image_subscriber_;

    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    Eigen::Matrix4f                                                _object_pose;

    // Publishers for visualizing point clouds and transformations
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_aligned_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>              _transform_broadcaster;

    // Point clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr                         object_cloud_;
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp_;
    tf2_ros::Buffer                                                tf_buffer_;
    tf2_ros::TransformListener                                     tf_listener_;
    geometry_msgs::msg::TransformStamped                           last_camera_tf_;
    std::string _camera_frame_name, _object_frame_name, _pointcloud_topic, _image_topic;

    float _x_min, _x_max, _y_min, _y_max, _z_min, _z_max, _voxel_size;
    int   _hue_min, _hue_max, _saturation_min, _saturation_max, _value_min, _value_max;

    void initializeParameters()
    {
        // Declare parameters
        this->declare_parameter<std::string>("input_pointcloud_topic", "/input_pointcloud");
        this->declare_parameter<std::string>("input_image_topic", "/input_image");
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
        this->declare_parameter<int>("hue_min", 0);
        this->declare_parameter<int>("hue_max", 179);
        this->declare_parameter<int>("saturation_min", 0);
        this->declare_parameter<int>("saturation_max", 255);
        this->declare_parameter<int>("value_min", 0);
        this->declare_parameter<int>("value_max", 255);
    }
    void getParameters()
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
        this->get_parameter("input_pointcloud_topic", _pointcloud_topic);
        this->get_parameter("input_image_topic", _image_topic);
    }

    void filterColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
    {
        // Iterate over the points and mark non-matching ones as NaN
        for(auto& point : *cloud)
        {
            int red   = point.r;
            int green = point.g;
            int blue  = point.b;

            cv::Mat rgb_pixel(1, 1, CV_8UC3, cv::Scalar(blue, green, red));
            cv::Mat hsv_pixel;
            cv::cvtColor(rgb_pixel, hsv_pixel, cv::COLOR_BGR2HSV);

            cv::Vec3b hsv = hsv_pixel.at<cv::Vec3b>(0, 0);
            int       hue = hsv[0], saturation = hsv[1], value = hsv[2];

            // If outside range, mark point as NaN
            if(hue < _hue_min || hue > _hue_max || saturation < _saturation_min ||
               saturation > _saturation_max || value < _value_min || value > _value_max)
            {
                point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
            }
        }

        // Remove NaN points to produce the filtered cloud in place
        cloud->is_dense = false; // Mark the cloud as sparse due to NaN points
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    }
    void filterColor(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const cv::Mat& image,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud)
    {
        filtered_cloud->clear();

        // Ensure the cloud is organized and has matching dimensions with the image
        if(!cloud->isOrganized())
        {
            throw std::runtime_error(
                "Point cloud must be ordered! Change Camera Launch Configuration!");
        }

        // Resize the image to match the cloud dimensions if necessary
        cv::Mat resized_image;
        if(image.rows != cloud->height || image.cols != cloud->width)
        {
            cv::resize(image, resized_image, cv::Size(cloud->width, cloud->height), 0.0, 0.0,
                       cv::INTER_NEAREST);
        }
        else
        {
            resized_image = image;
        }

        cv::Mat hsv_image;
        cv::cvtColor(resized_image, hsv_image, cv::COLOR_BGR2HSV);

        // Iterate through each point in the organized point cloud
        for(int v = 0; v < cloud->height; ++v)
        {
            for(int u = 0; u < cloud->width; ++u)
            {
                const auto& point = cloud->at(u, v);

                // Get the HSV color from the image at pixel (u, v)
                cv::Vec3b hsv        = hsv_image.at<cv::Vec3b>(v, u);
                int       hue        = hsv[0];
                int       saturation = hsv[1];
                int       value      = hsv[2];

                // Check if the HSV values fall within the specified range
                if(hue >= _hue_min && hue <= _hue_max && saturation >= _saturation_min &&
                   saturation <= _saturation_max && value >= _value_min && value <= _value_max)
                {
                    filtered_cloud->push_back(point);
                }
            }
        }
    }

    void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
                  const sensor_msgs::msg::Image::ConstSharedPtr&       image_msg)
    {
        // Convert PointCloud2 message to PCL point cloud
        RCLCPP_INFO_THROTTLE(get_logger(), *(get_clock()), 1000, "Input has %d points.",
                             cloud_msg->data.size());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr depth_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(*cloud_msg, *depth_cloud);

        // Convert ROS image to OpenCV image
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        }
        catch(cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Apply voxel grid filter if specified
        if(_voxel_size > 0.0)
        {
            TIME_FUNCTION(get_logger(), voxelCloud(depth_cloud));
        }
        // Filter points based on color in the image
        TIME_FUNCTION(get_logger(), filterColor(depth_cloud));

        if(depth_cloud->size() > 0)
        {

            // Perform ICP alignment
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_cloud(
                new pcl::PointCloud<pcl::PointXYZRGB>());
            TIME_FUNCTION(get_logger(), alignCloud(depth_cloud, aligned_cloud));

            // Publish on topics and TF Tree
            TIME_FUNCTION(get_logger(),
                          publishResults(depth_cloud, aligned_cloud, cloud_msg->header));
        }
        else
        {
            RCLCPP_INFO_THROTTLE(get_logger(), *(get_clock()), 1000, "Empty Filtered Cloud");
        }
    }

    void alignCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_cloud)
    {
        icp_.setInputTarget(filtered_cloud);
        icp_.align(*aligned_cloud, _object_pose);
        _object_pose = icp_.getFinalTransformation();
    }

    void voxelCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
    {
        pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
        voxel_grid.setInputCloud(cloud);
        voxel_grid.setLeafSize(_voxel_size, _voxel_size, _voxel_size);
        voxel_grid.filter(*cloud);
    }

    void publishResults(pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_cloud,
                        std_msgs::msg::Header                  header)
    {
        sensor_msgs::msg::PointCloud2 filtered_msg, aligned_msg;

        pcl::toROSMsg(*filtered_cloud, filtered_msg);
        filtered_msg.header = header;
        pub_filtered_->publish(filtered_msg);

        pcl::toROSMsg(*aligned_cloud, aligned_msg);
        aligned_msg.header = header;
        pub_aligned_->publish(aligned_msg);

        publishTransform(_object_pose);
    }

    /**
     * Update the estimated pose of the object based on the assumption that the camera
     * has moved and the target has not
     */
    void updateEstimate()
    {
        try
        {
            // Find transform of camera frame from prev to current in world frame
            auto current_tf =
                tf_buffer_.lookupTransform("world", "camera_link", tf2::TimePointZero);

            if(last_camera_tf_.header.stamp.sec != 0)
            {
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
            last_camera_tf_ = current_tf; // Update the last known transform
        }
        catch(tf2::TransformException& ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        }
    }

    // Helper function to publish the transformation matrix as PoseStamped
    void publishTransform(const Eigen::Matrix4f& transformation)
    {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.child_frame_id  = _object_frame_name;
        tf_msg.header.frame_id = "rs_cam_1_depth_optical_frame";
        tf_msg.header.stamp    = this->get_clock()->now();

        // Extract translation and rotation from the transformation matrix
        tf_msg.transform.translation.x = transformation(0, 3);
        tf_msg.transform.translation.y = transformation(1, 3);
        tf_msg.transform.translation.z = transformation(2, 3);

        Eigen::Matrix3f    rotation_matrix = transformation.block<3, 3>(0, 0);
        Eigen::Quaternionf quaternion(rotation_matrix);
        tf_msg.transform.rotation.x = quaternion.x();
        tf_msg.transform.rotation.y = quaternion.y();
        tf_msg.transform.rotation.z = quaternion.z();
        tf_msg.transform.rotation.w = quaternion.w();

        _transform_broadcaster->sendTransform(tf_msg);
    }
};

} // namespace icp_obj_pose

RCLCPP_COMPONENTS_REGISTER_NODE(icp_object_pose::ICPNode)

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<icp_object_pose::ICPNode>());
    rclcpp::shutdown();
    return 0;
}