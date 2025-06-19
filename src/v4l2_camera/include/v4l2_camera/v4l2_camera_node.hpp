#pragma once

class CalibrationHandler;
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <atomic>
#include <chrono>
#include <cv_bridge/cv_bridge.hpp>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <libudev.h>
#include <linux/videodev2.h>
#include <mutex>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>
#include <yaml-cpp/yaml.h>
/**
 * @class V4L2CameraNode
 * @brief A ROS 2 node for capturing and publishing images from V4L2-compatible devices,
 *        including support for automatic resolution detection, camera info publishing,
 *        and calibration feedback publishing.
 */
class V4L2CameraNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor. Initializes the camera, declares parameters, and sets up publishers.
     */
    V4L2CameraNode();

    /**
     * @brief Destructor. Cleans up device resources.
     */
    ~V4L2CameraNode();

    std::shared_ptr<CalibrationHandler> _calibration_server;

private:
    struct Buffer
    {
        void*  start;
        size_t length;
    };

    std::string                                                _device_path;
    std::string                                                _device_id, serial_number;
    int                                                        _width, _height;
    int                                                        _fd = -1;
    std::vector<Buffer>                                        _buffers;
    rclcpp::TimerBase::SharedPtr                               _timer;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr      _publisher;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr _camera_info_pub;
    sensor_msgs::msg::CameraInfo                               _camera_info;
    std::string                                                _calib_path;
    std::string                                                _serial_number;
    std::atomic_bool _calibration_in_progress{false}; ///< Flag for calibration status
    cv::Mat          _latest_frame;
    std::mutex       _frame_mutex;
    bool             _using_mjpeg = false;
    // V4L2 methods

    /**
     * @brief Opens and configures the camera device.
     */
    void open_device();

    /**
     * @brief Stops camera capture and cleans up.
     */
    void stop_capture();

    /**
     * @brief Main frame capture loop.
     */
    void capture_loop();

    /**
     * @brief Detects the highest available resolution and frame rate.
     * @return 0 on success, -1 on failure
     */
    int get_max_resolution_and_fps(const std::string& device_path, int& width, int& height,
                                   int& fps);

    // Utilities

    /**
     * @brief Resolves a device ID to a V4L2 device path.
     */
    std::string resolve_device_id();

    /**
     * @brief Loads calibration info from a YAML file if available.
     */
    void load_camera_info(const std::string& device_id);

    /**
     * @brief Publishes a calibration feedback image to a debug topic.
     */
    void publish_calibration_image(const cv::Mat& image);

    std::string get_serial_from_udev(const std::string& dev_path)
    {
        struct udev* udev = udev_new();
        if(!udev)
            return "";

        struct udev_device* dev = udev_device_new_from_subsystem_sysname(
            udev, "video4linux", std::filesystem::path(dev_path).filename().c_str());
        if(!dev)
        {
            udev_unref(udev);
            return "";
        }

        // Climb up to the USB device parent
        struct udev_device* parent =
            udev_device_get_parent_with_subsystem_devtype(dev, "usb", "usb_device");
        if(!parent)
        {
            udev_device_unref(dev);
            udev_unref(udev);
            return "";
        }

        const char* serial = udev_device_get_property_value(parent, "ID_SERIAL_SHORT");

        std::string result;
        if(serial)
            result = serial;

        udev_device_unref(dev);
        udev_unref(udev);
        return result;
    }

    friend class CalibrationHandler;
};
