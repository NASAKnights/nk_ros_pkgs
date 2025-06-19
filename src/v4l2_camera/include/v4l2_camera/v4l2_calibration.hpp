#pragma once

#include <cv_bridge/cv_bridge.hpp>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <linux/videodev2.h>
#include <memory>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include "v4l2_camera/action/calibrate_camera.hpp"
class V4L2CameraNode;
struct FrameScores
{
    float x_norm;
    float y_norm;
    float norm_area;
    float skew_score;
};

/**
 * @class CalibrationHandler
 * @brief Manages the calibration action server for V4L2CameraNode, including
 *        execution of chessboard and Charuco calibration procedures.
 */
class CalibrationHandler
{
public:
    using CalibrateCamera = v4l2_camera::action::CalibrateCamera;
    using GoalHandle      = rclcpp_action::ServerGoalHandle<CalibrateCamera>;

    /**
     * @brief Construct a new CalibrationHandler
     * @param node Pointer to the associated V4L2CameraNode
     */
    CalibrationHandler(V4L2CameraNode* node);

    /**
     * @brief Initialize and start the calibration action server
     */
    void start();

private:
    V4L2CameraNode*                                   _node;
    rclcpp_action::Server<CalibrateCamera>::SharedPtr _server;
    std::vector<FrameScores>                          _accepted_frames;

    // Action server callbacks
    rclcpp_action::GoalResponse   handle_goal(const rclcpp_action::GoalUUID&               uuid,
                                              std::shared_ptr<const CalibrateCamera::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
    void                          handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
    void                          execute(const std::shared_ptr<GoalHandle> goal_handle);
    void                          publish_calibration_image(const cv::Mat& image);

    // Calibration methods
    void run_chessboard(const std::shared_ptr<GoalHandle> goal_handle);
    void run_charuco(const std::shared_ptr<GoalHandle> goal_handle);

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _calibration_image_pub;
    FrameScores ExtractFrameParams(const std::vector<cv::Point2f>& corners,
                                   const cv::Size& image_size, const cv::Size& board_size)
    {
        FrameScores       p{};
        const cv::Scalar  mean = cv::mean(corners);
        const cv::Point2f center(mean[0], mean[1]);

        p.x_norm = 2.0f * (center.x / image_size.width - 0.5f);  // [-1, 1]
        p.y_norm = 2.0f * (center.y / image_size.height - 0.5f); // [-1, 1]

        std::vector<cv::Point2f> hull;
        cv::convexHull(corners, hull);
        p.norm_area = std::fabs(cv::contourArea(hull)) / (image_size.width * image_size.height);

        const auto dx   = corners[board_size.width - 1] - corners[0];
        const auto dy   = corners[(board_size.height - 1) * board_size.width] - corners[0];
        float      skew = std::abs(std::cos(std::atan2(dx.y, dx.x) - std::atan2(dy.y, dy.x)));
        p.skew_score    = 1.0f - skew;

        return p;
    }

    bool IsGoodSample(const FrameScores& current, const std::vector<FrameScores>& history)
    {
        constexpr float kMinXDist    = 0.1f;
        constexpr float kMinYDist    = 0.1f;
        constexpr float kMinAreaDist = 0.05f;
        constexpr float kMinSkewDist = 0.05f;

        for(const auto& prev : history)
        {
            if(std::abs(current.x_norm - prev.x_norm) < kMinXDist &&
               std::abs(current.y_norm - prev.y_norm) < kMinYDist &&
               std::abs(current.norm_area - prev.norm_area) < kMinAreaDist &&
               std::abs(current.skew_score - prev.skew_score) < kMinSkewDist)
            {
                return false; // too similar to previous sample
            }
        }

        return true;
    }

    void UpdateFeedbackScore(CalibrateCamera::Feedback& feedback)
    {
        // Constants
        constexpr int kGridDiv  = 6;
        constexpr int kAreaBins = 3;
        constexpr int kSkewBins = 3;

        // --- Spatial coverage: fill 6x6 grid
        std::set<std::pair<int, int>> unique_cells;
        for(const auto& f : _accepted_frames)
        {
            int gx = static_cast<int>((f.x_norm * 0.5f + 0.5f) * kGridDiv);
            int gy = static_cast<int>((f.y_norm * 0.5f + 0.5f) * kGridDiv);
            gx     = std::clamp(gx, 0, kGridDiv - 1);
            gy     = std::clamp(gy, 0, kGridDiv - 1);
            unique_cells.emplace(gx, gy);
        }

        // Count how many of those fall in left/right or top/bottom
        int left = 0, right = 0, top = 0, bottom = 0;
        for(const auto& [gx, gy] : unique_cells)
        {
            if(gx < kGridDiv / 2)
                ++left;
            else
                ++right;
            if(gy < kGridDiv / 2)
                ++top;
            else
                ++bottom;
        }

        float half_total   = (kGridDiv * kGridDiv) / 2.0f;
        feedback.x_percent = {left / half_total, right / half_total};
        feedback.y_percent = {top / half_total, bottom / half_total};

        // --- Area diversity
        std::set<int> area_bins;
        for(const auto& f : _accepted_frames)
        {
            int bin = std::clamp(static_cast<int>(f.norm_area * kAreaBins), 0, kAreaBins - 1);
            area_bins.insert(bin);
        }
        feedback.area_diversity = static_cast<float>(area_bins.size()) / kAreaBins;

        // --- Skew diversity
        std::set<int> skew_bins;
        for(const auto& f : _accepted_frames)
        {
            int bin = std::clamp(static_cast<int>(f.skew_score * kSkewBins), 0, kSkewBins - 1);
            skew_bins.insert(bin);
        }
        feedback.skew_diversity = static_cast<float>(skew_bins.size()) / kSkewBins;

        // --- Final quality score
        feedback.quality_score = 0.4f * feedback.skew_diversity + 0.4f * feedback.area_diversity +
                                 0.1f * (feedback.x_percent[0] + feedback.x_percent[1]) * 0.5f +
                                 0.1f * (feedback.y_percent[0] + feedback.y_percent[1]) * 0.5f;
    }

    void write_ros_camera_yaml(const std::string& filepath, const cv::Size& image_size,
                               const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs)
    {
        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << "image_width" << YAML::Value << image_size.width;
        out << YAML::Key << "image_height" << YAML::Value << image_size.height;
        out << YAML::Key << "camera_name" << YAML::Value << "default_camera";
        out << YAML::Key << "distortion_model" << YAML::Value << "plumb_bob";

        out << YAML::Key << "camera_matrix" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "rows" << YAML::Value << 3;
        out << YAML::Key << "cols" << YAML::Value << 3;
        out << YAML::Key << "data" << YAML::Value
            << std::vector<double>(camera_matrix.begin<double>(), camera_matrix.end<double>());
        out << YAML::EndMap;

        out << YAML::Key << "distortion_coefficients" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "rows" << YAML::Value << 1;
        out << YAML::Key << "cols" << YAML::Value << dist_coeffs.cols;
        out << YAML::Key << "data" << YAML::Value
            << std::vector<double>(dist_coeffs.begin<double>(), dist_coeffs.end<double>());
        out << YAML::EndMap;

        out << YAML::Key << "rectification_matrix" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "rows" << YAML::Value << 3;
        out << YAML::Key << "cols" << YAML::Value << 3;
        out << YAML::Key << "data" << YAML::Value
            << std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        out << YAML::EndMap;

        out << YAML::Key << "projection_matrix" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "rows" << YAML::Value << 3;
        out << YAML::Key << "cols" << YAML::Value << 4;
        out << YAML::Key << "data" << YAML::Value
            << std::vector<double>{camera_matrix.at<double>(0, 0),
                                   0.0,
                                   camera_matrix.at<double>(0, 2),
                                   0.0,
                                   0.0,
                                   camera_matrix.at<double>(1, 1),
                                   camera_matrix.at<double>(1, 2),
                                   0.0,
                                   0.0,
                                   0.0,
                                   1.0,
                                   0.0};
        out << YAML::EndMap;
        out << YAML::EndMap;

        std::ofstream fout(filepath);
        fout << out.c_str();
        fout.close();
    }
};
