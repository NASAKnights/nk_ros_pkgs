// File: calibration_node.cpp
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

// class CalibrationNode : public rclcpp::Node
// {
// public:
//     CalibrationNode()
//         : Node("calibration_node")
//     {
//         this->declare_parameter("pattern_type", "chessboard");
//         this->declare_parameter("chessboard_rows", 6);
//         this->declare_parameter("chessboard_cols", 9);
//         this->declare_parameter("square_size", 0.025);
//         this->declare_parameter("charuco_dict", "DICT_5X5_1000");
//         this->declare_parameter("capture_topic", "/image_raw");
//         this->declare_parameter("output_file", "calibration.yaml");

//         std::string topic = this->get_parameter("capture_topic").as_string();
//         sub_              = this->create_subscription<sensor_msgs::msg::Image>(
//             topic, 10, std::bind(&CalibrationNode::image_callback, this, std::placeholders::_1));

//         cv::namedWindow("Calibration View", cv::WINDOW_AUTOSIZE);
//     }

// private:
//     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
//     std::vector<std::vector<cv::Point2f>>                    image_points_;
//     std::vector<std::vector<cv::Point3f>>                    object_points_;
//     std::string                                              pattern_type_ = "chessboard";
//     int                                                      rows_, cols_;
//     double                                                   square_size_;
//     std::string                                              output_file_;

//     void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
//     {
//         cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

//         if(pattern_type_ == "chessboard")
//             detect_chessboard(frame);
//         else
//             detect_charuco(frame);
//     }

//     void detect_chessboard(const cv::Mat& frame)
//     {
//         this->get_parameter("chessboard_rows", rows_);
//         this->get_parameter("chessboard_cols", cols_);
//         this->get_parameter("square_size", square_size_);
//         this->get_parameter("output_file", output_file_);

//         cv::Size                 pattern_size(cols_, rows_);
//         std::vector<cv::Point2f> corners;
//         bool                     found = cv::findChessboardCorners(frame, pattern_size, corners);

//         if(found)
//         {
//             cv::drawChessboardCorners(frame, pattern_size, corners, found);
//             image_points_.push_back(corners);
//             object_points_.emplace_back();

//             for(int i = 0; i < rows_; ++i)
//             {
//                 for(int j = 0; j < cols_; ++j)
//                 {
//                     object_points_.back().emplace_back(j * square_size_, i * square_size_, 0);
//                 }
//             }

//             if(image_points_.size() >= 10)
//             {
//                 calibrate(frame.size());
//             }
//         }

//         cv::imshow("Calibration View", frame);
//         cv::waitKey(1);
//     }

//     void detect_charuco(const cv::Mat& frame)
//     {
//         int         squares_x, squares_y;
//         double      square_len, marker_len;
//         std::string dict_name;
//         this->get_parameter("charuco_squares_x", squares_x);
//         this->get_parameter("charuco_squares_y", squares_y);
//         this->get_parameter("charuco_square_length", square_len);
//         this->get_parameter("charuco_marker_length", marker_len);
//         this->get_parameter("charuco_dict", dict_name);
//         this->get_parameter("output_file", output_file_);

//         dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
//         board_      = cv::aruco::CharucoBoard::create(squares_x, squares_y, square_len,
//         marker_len,
//                                                       dictionary_);

//         std::vector<int>                      marker_ids;
//         std::vector<std::vector<cv::Point2f>> marker_corners;
//         cv::aruco::detectMarkers(frame, dictionary_, marker_corners, marker_ids);

//         if(marker_ids.size() > 0)
//         {
//             cv::aruco::drawDetectedMarkers(frame, marker_corners, marker_ids);

//             cv::Mat charuco_corners, charuco_ids;
//             cv::aruco::interpolateCornersCharuco(marker_corners, marker_ids, frame, board_,
//                                                  charuco_corners, charuco_ids);

//             if(charuco_ids.total() > 4)
//             {
//                 all_charuco_corners_.push_back(charuco_corners);
//                 all_charuco_ids_.push_back(charuco_ids);
//                 cv::aruco::drawDetectedCornersCharuco(frame, charuco_corners, charuco_ids);

//                 if(all_charuco_corners_.size() >= 10)
//                 {
//                     calibrate_charuco(frame.size());
//                 }
//             }
//         }

//         cv::imshow("Calibration View", frame);
//         cv::waitKey(1);
//     }

//     void calibrate(const cv::Size& image_size)
//     {
//         cv::Mat              camera_matrix, dist_coeffs;
//         std::vector<cv::Mat> rvecs, tvecs;

//         double rms = cv::calibrateCamera(object_points_, image_points_, image_size,
//         camera_matrix,
//                                          dist_coeffs, rvecs, tvecs);

//         RCLCPP_INFO(this->get_logger(), "Calibration RMS error: %f", rms);

//         YAML::Emitter out;
//         out << YAML::BeginMap;
//         out << YAML::Key << "image_width" << YAML::Value << image_size.width;
//         out << YAML::Key << "image_height" << YAML::Value << image_size.height;
//         out << YAML::Key << "camera_matrix" << YAML::Value << YAML::BeginMap;
//         out << YAML::Key << "data" << YAML::Value
//             << std::vector<double>(camera_matrix.begin<double>(), camera_matrix.end<double>());
//         out << YAML::EndMap;
//         out << YAML::Key << "distortion_coefficients" << YAML::Value << YAML::BeginMap;
//         out << YAML::Key << "data" << YAML::Value
//             << std::vector<double>(dist_coeffs.begin<double>(), dist_coeffs.end<double>());
//         out << YAML::EndMap;
//         out << YAML::Key << "distortion_model" << YAML::Value << "plumb_bob";
//         out << YAML::Key << "rectification_matrix" << YAML::Value << YAML::BeginMap;
//         out << YAML::Key << "data" << YAML::Value << std::vector<double>(9, 0.0);
//         out << YAML::EndMap;
//         out << YAML::Key << "projection_matrix" << YAML::Value << YAML::BeginMap;
//         out << YAML::Key << "data" << YAML::Value << std::vector<double>(12, 0.0);
//         out << YAML::EndMap;
//         out << YAML::EndMap;

//         std::ofstream file(this->get_parameter("output_file").as_string());
//         file << out.c_str();
//         file.close();

//         RCLCPP_INFO(this->get_logger(), "Calibration saved to %s", output_file_.c_str());
//         rclcpp::shutdown();
//     }
// };

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<CalibrationNode>());
    rclcpp::shutdown();
    return 0;
}
