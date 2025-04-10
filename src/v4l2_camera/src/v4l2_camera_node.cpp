// File: v4l2_camera_node.cpp
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
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
#include <unistd.h>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "v4l2_camera/action/calibrate_camera.hpp"

using CalibrateCamera = v4l2_camera::action::CalibrateCamera;

using namespace std::chrono_literals;
namespace fs = std::filesystem;

class V4L2CameraNode : public rclcpp::Node
{
public:
    V4L2CameraNode()
        : Node("v4l2_camera_node")
    {
        this->declare_parameter("device", "");
        this->declare_parameter("device_id", "");
        this->declare_parameter("width", 0);  // default to 0 → auto
        this->declare_parameter("height", 0); // default to 0 → auto
        this->declare_parameter("calibration_base_path", "/.ros/calibration");

        std::string device_param    = this->get_parameter("device").as_string();
        std::string device_id_param = this->get_parameter("device_id").as_string();
        calib_base_path_            = this->get_parameter("calibration_base_path").as_string();

        if(!device_param.empty())
        {
            device_path_ = device_param;
        }
        else if(!device_id_param.empty())
        {
            device_path_ = resolve_device_id(device_id_param);
        }
        else
        {
            RCLCPP_FATAL(this->get_logger(), "No device or device_id specified.");
            rclcpp::shutdown();
            return;
        }

        width_  = this->get_parameter("width").as_int();
        height_ = this->get_parameter("height").as_int();
        int fps = 0;

        if(width_ <= 0 || height_ <= 0)
        {
            if(get_max_resolution_and_fps(device_path_, width_, height_, fps) != 0)
            {
                RCLCPP_FATAL(this->get_logger(),
                             "Failed to determine max resolution for device: %s",
                             device_path_.c_str());
                rclcpp::shutdown();
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Using max resolution: %dx%d", width_, height_);
        }

        publisher_       = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);

        load_camera_info(device_id_param);
        open_device();
        timer_ =
            this->create_wall_timer(1000ms / fps, std::bind(&V4L2CameraNode::capture_loop, this));
        calibration_image_pub_ =
            this->create_publisher<sensor_msgs::msg::Image>("/calibration", 10);
        calibration_server_ = rclcpp_action::create_server<CalibrateCamera>(
            this, "calibrate_camera",
            std::bind(&V4L2CameraNode::handle_goal, this, std::placeholders::_1,
                      std::placeholders::_2),
            std::bind(&V4L2CameraNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&V4L2CameraNode::handle_accepted, this, std::placeholders::_1));
    }

    int get_max_resolution_and_fps(const std::string& device_path, int& width, int& height,
                                   int& fps)
    {
        int fd = open(device_path.c_str(), O_RDWR);
        if(fd < 0)
            return -1;

        struct v4l2_fmtdesc fmt = {};
        fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.index               = 0;

        int max_score = 0;
        while(ioctl(fd, VIDIOC_ENUM_FMT, &fmt) == 0)
        {
            struct v4l2_frmsizeenum size = {};
            size.pixel_format            = fmt.pixelformat;
            size.index                   = 0;
            while(ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &size) == 0)
            {
                if(size.type == V4L2_FRMSIZE_TYPE_DISCRETE)
                {
                    struct v4l2_frmivalenum ival = {};
                    ival.pixel_format            = fmt.pixelformat;
                    ival.width                   = size.discrete.width;
                    ival.height                  = size.discrete.height;
                    ival.index                   = 0;
                    while(ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &ival) == 0)
                    {
                        if(ival.type == V4L2_FRMIVAL_TYPE_DISCRETE)
                        {
                            int area        = size.discrete.width * size.discrete.height;
                            int current_fps = ival.discrete.denominator / ival.discrete.numerator;
                            int score       = area * current_fps;
                            if(score > max_score)
                            {
                                max_score = score;
                                width     = size.discrete.width;
                                height    = size.discrete.height;
                                fps       = current_fps;
                            }
                        }
                        ival.index++;
                    }
                }
                size.index++;
            }
            fmt.index++;
        }

        close(fd);
        return max_score > 0 ? 0 : -1;
    }

    ~V4L2CameraNode()
    {
        stop_capture();
    }

private:
    struct Buffer
    {
        void*  start;
        size_t length;
    };

    std::string                                                device_path_;
    int                                                        width_, height_;
    int                                                        fd_ = -1;
    std::vector<Buffer>                                        buffers_;
    rclcpp::TimerBase::SharedPtr                               timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr      publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    sensor_msgs::msg::CameraInfo                               camera_info_;
    std::string                                                calib_base_path_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr      calibration_image_pub_;

    std::string resolve_device_id(const std::string& device_id)
    {
        for(const auto& entry : fs::directory_iterator("/sys/class/video4linux"))
        {
            std::string   dev_name      = entry.path().filename();
            std::string   device_dir    = entry.path().string() + "/device";
            std::string   modalias_file = device_dir + "/modalias";
            std::ifstream infile(modalias_file);
            std::string   line;
            if(infile.is_open() && std::getline(infile, line))
            {
                if(line.find(device_id) != std::string::npos)
                {
                    std::string full_path = "/dev/" + dev_name;
                    RCLCPP_INFO(this->get_logger(), "Resolved device_id '%s' to %s",
                                device_id.c_str(), full_path.c_str());
                    return full_path;
                }
            }
        }
        RCLCPP_ERROR(this->get_logger(), "Device with ID '%s' not found.", device_id.c_str());
        return "";
    }

    void load_camera_info(const std::string& device_id)
    {
        if(device_id.empty())
            return;
        std::string calib_file = calib_base_path_ + "/calib_" + device_id + ".yaml";
        if(!fs::exists(calib_file))
        {
            RCLCPP_WARN(this->get_logger(), "Calibration file not found: %s", calib_file.c_str());
            return;
        }
        try
        {
            YAML::Node calib              = YAML::LoadFile(calib_file);
            camera_info_.width            = calib["image_width"].as<int>();
            camera_info_.height           = calib["image_height"].as<int>();
            camera_info_.distortion_model = calib["distortion_model"].as<std::string>();
            camera_info_.d = calib["distortion_coefficients"]["data"].as<std::vector<double>>();
            camera_info_.k = calib["camera_matrix"]["data"].as<std::array<double, 9>>();
            camera_info_.r = calib["rectification_matrix"]["data"].as<std::array<double, 9>>();
            camera_info_.p = calib["projection_matrix"]["data"].as<std::array<double, 12>>();
            RCLCPP_INFO(this->get_logger(), "Loaded calibration for device_id %s",
                        device_id.c_str());
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load camera info: %s", e.what());
        }
    }

    void open_device()
    {
        fd_ = open(device_path_.c_str(), O_RDWR | O_NONBLOCK);
        if(fd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open %s", device_path_.c_str());
            return;
        }

        struct v4l2_format fmt  = {};
        fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width       = width_;
        fmt.fmt.pix.height      = height_;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        fmt.fmt.pix.field       = V4L2_FIELD_ANY;
        if(ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to set format.");
            close(fd_);
            fd_ = -1;
            return;
        }

        struct v4l2_requestbuffers req = {};
        req.count                      = 4;
        req.type                       = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory                     = V4L2_MEMORY_MMAP;
        if(ioctl(fd_, VIDIOC_REQBUFS, &req) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to request buffers.");
            close(fd_);
            fd_ = -1;
            return;
        }

        buffers_.resize(req.count);
        for(size_t i = 0; i < req.count; ++i)
        {
            struct v4l2_buffer buf = {};
            buf.type               = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory             = V4L2_MEMORY_MMAP;
            buf.index              = i;
            if(ioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to query buffer %zu", i);
                continue;
            }

            buffers_[i].length = buf.length;
            buffers_[i].start =
                mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
        }

        for(size_t i = 0; i < req.count; ++i)
        {
            struct v4l2_buffer buf = {};
            buf.type               = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory             = V4L2_MEMORY_MMAP;
            buf.index              = i;
            ioctl(fd_, VIDIOC_QBUF, &buf);
        }

        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(fd_, VIDIOC_STREAMON, &type);
        RCLCPP_INFO(this->get_logger(), "Camera stream started on %s", device_path_.c_str());
    }

    void capture_loop()
    {
        if(fd_ < 0)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Camera not open, retrying...");
            open_device();
            return;
        }

        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd_, &fds);
        struct timeval tv = {0};
        tv.tv_sec         = 0;
        tv.tv_usec        = 500000;

        int r = select(fd_ + 1, &fds, NULL, NULL, &tv);
        if(r <= 0)
        {
            RCLCPP_WARN(this->get_logger(), "No camera data. Retrying...");
            return;
        }

        struct v4l2_buffer buf = {};
        buf.type               = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory             = V4L2_MEMORY_MMAP;
        if(ioctl(fd_, VIDIOC_DQBUF, &buf) < 0)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to dequeue buffer.");
            return;
        }

        cv::Mat yuyv(height_, width_, CV_8UC2, buffers_[buf.index].start);
        cv::Mat bgr;
        cv::cvtColor(yuyv, bgr, cv::COLOR_YUV2BGR_YUYV);

        auto stamp        = this->get_clock()->now();
        auto msg          = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bgr).toImageMsg();
        msg->header.stamp = stamp;
        publisher_->publish(*msg);
        {
            std::lock_guard<std::mutex> lock(_frame_mutex);
            _latest_frame = bgr.clone();
        }
        camera_info_.header.stamp = stamp;
        camera_info_pub_->publish(camera_info_);

        ioctl(fd_, VIDIOC_QBUF, &buf);
    }

    void stop_capture()
    {
        if(fd_ >= 0)
        {
            enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            ioctl(fd_, VIDIOC_STREAMOFF, &type);
            for(auto& b : buffers_)
            {
                munmap(b.start, b.length);
            }
            close(fd_);
            fd_ = -1;
        }
    }
    void publish_calibration_image(const cv::Mat& image)
    {
        if(!calibration_image_pub_)
            return;
        auto msg          = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
        msg->header.stamp = this->get_clock()->now();
        calibration_image_pub_->publish(*msg);
    }

    rclcpp_action::Server<CalibrateCamera>::SharedPtr calibration_server_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&               uuid,
                                            std::shared_ptr<const CalibrateCamera::Goal> goal)
    {
        if(_calibration_in_progress.load())
        {
            RCLCPP_WARN(this->get_logger(), "Calibration already in progress.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(this->get_logger(), "Accepted calibration goal (%s board)",
                    goal->board_type.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<CalibrateCamera>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Calibration canceled.");
        _calibration_in_progress.store(false);
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<CalibrateCamera>> goal_handle)
    {
        _calibration_in_progress.store(true);
        std::thread{std::bind(&V4L2CameraNode::execute_calibration, this, goal_handle)}.detach();
    }

    void execute_calibration(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<CalibrateCamera>> goal_handle)
    {
        if(goal_handle->get_goal()->board_type == "chessboard")
        {
            execute_chessboard_calibration(goal_handle);
        }
        else if(goal_handle->get_goal()->board_type == "charuco")
        {
            execute_charuco_calibration(goal_handle);
        }
        else
        {
            auto result     = std::make_shared<CalibrateCamera::Result>();
            result->success = false;
            result->message = "Unsupported board type: " + goal_handle->get_goal()->board_type;
            goal_handle->abort(result);
        }
    }
    void execute_chessboard_calibration(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<CalibrateCamera>> goal_handle)
    {
        const auto goal     = goal_handle->get_goal();
        auto       feedback = std::make_shared<CalibrateCamera::Feedback>();
        auto       result   = std::make_shared<CalibrateCamera::Result>();

        std::vector<std::vector<cv::Point3f>> object_points;
        std::vector<std::vector<cv::Point2f>> image_points;
        std::vector<cv::Mat>                  collected_images;

        cv::Size board_size(goal->cols, goal->rows);
        cv::Size image_size;

        int frames_needed = goal->num_frames;
        int accepted      = 0;

        RCLCPP_INFO(this->get_logger(), "Capturing %d calibration frames...", frames_needed);

        rclcpp::Rate rate(30);
        while(rclcpp::ok() && accepted < frames_needed && _calibration_in_progress.load())
        {
            cv::Mat frame, gray;
            {
                std::lock_guard<std::mutex> lock(_frame_mutex);
                if(_latest_frame.empty())
                    continue;
                frame = _latest_frame.clone();
            }

            image_size = frame.size();
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

            std::vector<cv::Point2f> corners;
            bool                     found =
                cv::findChessboardCorners(gray, board_size, corners,
                                          cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK |
                                              cv::CALIB_CB_NORMALIZE_IMAGE);

            feedback->frames_captured = accepted;
            if(!found)
            {
                feedback->accepted = false;
                feedback->reason   = "No chessboard found";
                goal_handle->publish_feedback(feedback);
                rate.sleep();
                publish_calibration_image(frame);
                continue;
            }

            float area           = static_cast<float>(cv::contourArea(corners));
            feedback->area       = area;
            feedback->accepted   = true;
            feedback->reason     = "Accepted";
            feedback->x_offset   = static_cast<int>(cv::mean(corners)[0] - (image_size.width / 2));
            feedback->y_offset   = static_cast<int>(cv::mean(corners)[1] - (image_size.height / 2));
            feedback->skew_score = 1.0; // TODO: Compute real skew metric

            goal_handle->publish_feedback(feedback);

            std::vector<cv::Point3f> objp;
            for(int i = 0; i < goal->rows; ++i)
                for(int j = 0; j < goal->cols; ++j)
                    objp.emplace_back(j * goal->square_size, i * goal->square_size, 0);

            object_points.push_back(objp);
            image_points.push_back(corners);
            ++accepted;
            cv::drawChessboardCorners(frame, board_size, corners, found);
            publish_calibration_image(frame);
            rate.sleep();
        }

        if(accepted < frames_needed)
        {
            result->success = false;
            result->message = "Calibration aborted before sufficient frames were collected.";
            goal_handle->abort(result);
            return;
        }

        cv::Mat              camera_matrix, dist_coeffs;
        std::vector<cv::Mat> rvecs, tvecs;
        double err = cv::calibrateCamera(object_points, image_points, image_size, camera_matrix,
                                         dist_coeffs, rvecs, tvecs);

        std::string     calib_path = calib_base_path_ + "/calib_" + goal->board_type + ".yaml";
        cv::FileStorage fs(calib_path, cv::FileStorage::WRITE);
        fs << "image_width" << image_size.width;
        fs << "image_height" << image_size.height;
        fs << "camera_matrix" << camera_matrix;
        fs << "distortion_coefficients" << dist_coeffs;
        fs.release();

        result->success = true;
        result->message = "Calibration complete with reprojection error: " + std::to_string(err);
        goal_handle->succeed(result);
    }
    void execute_charuco_calibration(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<CalibrateCamera>> goal_handle)
    {
        const auto goal     = goal_handle->get_goal();
        auto       feedback = std::make_shared<CalibrateCamera::Feedback>();
        auto       result   = std::make_shared<CalibrateCamera::Result>();

        int                                   accepted = 0;
        std::vector<cv::Mat>                  collected_images;
        std::vector<std::vector<cv::Point2f>> all_corners;
        std::vector<std::vector<int>>         all_ids;

        cv::Ptr<cv::aruco::Dictionary> dictionary =
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(
            goal->cols, goal->rows, goal->square_size, goal->square_size * 0.7, dictionary);

        rclcpp::Rate rate(30);
        cv::Size     image_size;

        while(rclcpp::ok() && accepted < goal->num_frames && _calibration_in_progress.load())
        {
            cv::Mat frame, gray;
            {
                std::lock_guard<std::mutex> lock(_frame_mutex);
                if(_latest_frame.empty())
                    continue;
                frame = _latest_frame.clone();
            }

            image_size = frame.size();
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

            std::vector<int>                      marker_ids;
            std::vector<std::vector<cv::Point2f>> marker_corners;
            cv::aruco::detectMarkers(gray, dictionary, marker_corners, marker_ids);
            if(marker_ids.empty())
            {
                feedback->accepted = false;
                feedback->reason   = "No ArUco markers found";
                goal_handle->publish_feedback(feedback);
                publish_calibration_image(frame);
                rate.sleep();
                continue;
            }

            cv::Mat charuco_corners, charuco_ids;
            cv::aruco::interpolateCornersCharuco(marker_corners, marker_ids, gray, board,
                                                 charuco_corners, charuco_ids);

            if(charuco_ids.total() < 4)
            {
                feedback->accepted = false;
                feedback->reason   = "Too few Charuco corners";
                goal_handle->publish_feedback(feedback);
                publish_calibration_image(frame);
                rate.sleep();
                continue;
            }

            feedback->accepted        = true;
            feedback->frames_captured = ++accepted;
            feedback->reason          = "Accepted";
            feedback->area            = static_cast<float>(cv::contourArea(charuco_corners));
            feedback->x_offset =
                static_cast<int>(cv::mean(charuco_corners)[0] - (image_size.width / 2));
            feedback->y_offset =
                static_cast<int>(cv::mean(charuco_corners)[1] - (image_size.height / 2));
            feedback->skew_score = 1.0; // TODO: real skew calc

            all_corners.emplace_back(charuco_corners.begin<cv::Point2f>(),
                                     charuco_corners.end<cv::Point2f>());
            all_ids.emplace_back(charuco_ids.begin<int>(), charuco_ids.end<int>());
            collected_images.push_back(gray.clone());

            cv::aruco::drawDetectedMarkers(frame, marker_corners, marker_ids);
            goal_handle->publish_feedback(feedback);
            publish_calibration_image(frame);
            rate.sleep();
        }

        if(accepted < goal->num_frames)
        {
            result->success = false;
            result->message = "Calibration aborted before sufficient frames were collected.";
            goal_handle->abort(result);
            return;
        }

        cv::Mat camera_matrix, dist_coeffs;
        double  error = cv::aruco::calibrateCameraCharuco(all_corners, all_ids, board, image_size,
                                                          camera_matrix, dist_coeffs);

        std::string     calib_file = calib_base_path_ + "/calib_" + goal->board_type + ".yaml";
        cv::FileStorage fs(calib_file, cv::FileStorage::WRITE);
        fs << "image_width" << image_size.width;
        fs << "image_height" << image_size.height;
        fs << "camera_matrix" << camera_matrix;
        fs << "distortion_coefficients" << dist_coeffs;
        fs.release();

        result->success = true;
        result->message =
            "Charuco calibration successful with reprojection error: " + std::to_string(error);
        goal_handle->succeed(result);
    }

    std::mutex       _frame_mutex;
    cv::Mat          _latest_frame;
    std::atomic_bool _calibration_in_progress{false};
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<V4L2CameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}