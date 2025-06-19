#include "v4l2_camera/v4l2_calibration.hpp"
#include "v4l2_camera/v4l2_camera_node.hpp"

using namespace std::chrono_literals;
using CalibrateCamera = v4l2_camera::action::CalibrateCamera;

CalibrationHandler::CalibrationHandler(V4L2CameraNode* node)
    : _node(node)
{
    _calibration_image_pub = _node->create_publisher<sensor_msgs::msg::Image>("calibration", 10);
}

void CalibrationHandler::start()
{
    _server = rclcpp_action::create_server<CalibrateCamera>(
        _node->shared_from_this(), "calibrate_camera",
        std::bind(&CalibrationHandler::handle_goal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&CalibrationHandler::handle_cancel, this, std::placeholders::_1),
        std::bind(&CalibrationHandler::handle_accepted, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse
CalibrationHandler::handle_goal(const rclcpp_action::GoalUUID&               uuid,
                                std::shared_ptr<const CalibrateCamera::Goal> goal)
{
    if(_node->_calibration_in_progress.load())
    {
        RCLCPP_WARN(_node->get_logger(), "Calibration already in progress.");
        return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(_node->get_logger(), "Accepted calibration goal (%s board)",
                goal->board_type.c_str());
    std::this_thread::sleep_for(1s);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
CalibrationHandler::handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
{
    RCLCPP_INFO(_node->get_logger(), "Calibration canceled.");
    _node->_calibration_in_progress.store(false);
    return rclcpp_action::CancelResponse::ACCEPT;
}

void CalibrationHandler::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
    _node->_calibration_in_progress.store(true);
    std::thread{std::bind(&CalibrationHandler::execute, this, goal_handle)}.detach();
}

void CalibrationHandler::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
    auto goal = goal_handle->get_goal();
    if(goal->board_type == "chessboard")
    {
        run_chessboard(goal_handle);
    }
    else if(goal->board_type == "charuco")
    {
        run_charuco(goal_handle);
    }
    else
    {
        auto result     = std::make_shared<CalibrateCamera::Result>();
        result->success = false;
        result->message = "Unsupported board type: " + goal->board_type;
        goal_handle->abort(result);
    }
    _node->_calibration_in_progress.store(false);
}

void CalibrationHandler::run_chessboard(const std::shared_ptr<GoalHandle> goal_handle)
{
    const auto goal     = goal_handle->get_goal();
    auto       feedback = std::make_shared<CalibrateCamera::Feedback>();
    auto       result   = std::make_shared<CalibrateCamera::Result>();
    // Constants for binning
    constexpr int kSkewBins = 5;
    constexpr int kAreaBins = 5;
    constexpr int kGridDiv  = 6;

    std::set<int>                 _skew_bins;
    std::set<int>                 _area_bins;
    std::set<std::pair<int, int>> _xy_cells;

    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;

    cv::Size board_size(goal->cols, goal->rows);
    cv::Size image_size;

    int          accepted = 0;
    rclcpp::Rate rate(30);

    while(rclcpp::ok() && _node->_calibration_in_progress.load())
    {
        cv::Mat frame, gray;
        {
            std::lock_guard<std::mutex> lock(_node->_frame_mutex);
            if(_node->_latest_frame.empty())
                continue;
            frame = _node->_latest_frame.clone();
        }

        image_size = frame.size();
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        bool                     found = cv::findChessboardCorners(
            gray, board_size, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

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
        FrameScores params = ExtractFrameParams(corners, image_size, board_size);

        if(!IsGoodSample(params, _accepted_frames))
        {
            feedback->accepted = false;
            feedback->reason   = "Too similar to previous sample";
            goal_handle->publish_feedback(feedback);
            publish_calibration_image(frame);
            rate.sleep();
            continue;
        }

        _accepted_frames.push_back(params); // Store for future comparisons

        UpdateFeedbackScore(*feedback);

        std::vector<cv::Point3f> objp;
        for(int i = 0; i < goal->rows; ++i)
            for(int j = 0; j < goal->cols; ++j)
                objp.emplace_back(j * goal->square_size, i * goal->square_size, 0);

        object_points.push_back(objp);
        image_points.push_back(corners);
        ++accepted;

        cv::drawChessboardCorners(frame, board_size, corners, found);
        goal_handle->publish_feedback(feedback);
        publish_calibration_image(frame);
        rate.sleep();
    }

    // if(accepted < frames_needed)
    // {
    //     result->success = false;
    //     result->message = "Calibration aborted before sufficient frames were collected.";
    //     goal_handle->abort(result);
    //     return;
    // }

    cv::Mat              camera_matrix, dist_coeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    double err = cv::calibrateCamera(object_points, image_points, image_size, camera_matrix,
                                     dist_coeffs, rvecs, tvecs);

    write_ros_camera_yaml(_node->_calib_path, image_size, camera_matrix, dist_coeffs);

    result->success = true;
    result->message = "Calibration complete with reprojection error: " + std::to_string(err);
    goal_handle->succeed(result);
}

void CalibrationHandler::run_charuco(const std::shared_ptr<GoalHandle> goal_handle)
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

    while(rclcpp::ok() && _node->_calibration_in_progress.load())
    {
        cv::Mat frame, gray;
        {
            std::lock_guard<std::mutex> lock(_node->_frame_mutex);
            if(_node->_latest_frame.empty())
                continue;
            frame = _node->_latest_frame.clone();
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
        feedback->area_diversity  = static_cast<float>(cv::contourArea(charuco_corners));
        feedback->x_percent       = {-1.0, 1.0};
        feedback->y_percent       = {-1.0, 1.0};
        feedback->skew_diversity  = 1.0;

        all_corners.emplace_back(charuco_corners.begin<cv::Point2f>(),
                                 charuco_corners.end<cv::Point2f>());
        all_ids.emplace_back(charuco_ids.begin<int>(), charuco_ids.end<int>());
        collected_images.push_back(gray.clone());

        cv::aruco::drawDetectedMarkers(frame, marker_corners, marker_ids);
        goal_handle->publish_feedback(feedback);
        publish_calibration_image(frame);
        rate.sleep();
    }

    // if(accepted < goal->num_frames)
    // {
    //     result->success = false;
    //     result->message = "Calibration aborted before sufficient frames were collected.";
    //     goal_handle->abort(result);
    //     return;
    // }

    cv::Mat camera_matrix, dist_coeffs;
    double  error = cv::aruco::calibrateCameraCharuco(all_corners, all_ids, board, image_size,
                                                      camera_matrix, dist_coeffs);

    write_ros_camera_yaml(_node->_calib_path, image_size, camera_matrix, dist_coeffs);

    result->success = true;
    result->message =
        "Charuco calibration successful with reprojection error: " + std::to_string(error);
    goal_handle->succeed(result);
}

void CalibrationHandler::publish_calibration_image(const cv::Mat& image)
{
    if(!_calibration_image_pub)
        return;
    auto msg          = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
    msg->header.stamp = _node->get_clock()->now();
    _calibration_image_pub->publish(*msg);
}
