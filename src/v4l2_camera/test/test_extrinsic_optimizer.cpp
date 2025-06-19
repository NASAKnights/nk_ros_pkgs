#include "v4l2_camera/extrinsic_calibration.hpp"
#include <gtest/gtest.h>
#include <iostream>
#include <random>
#include <unordered_map>

TEST(CameraCalibrationOptimizerTest, SyntheticTwoCameraSetup)
{
    CameraCalibrationOptimizer optimizer;

    std::unordered_map<std::string, CameraIntrinsics> intrinsics;
    CameraIntrinsics                                  cam_intrin;
    cam_intrin.K = (cv::Mat_<double>(3, 3) << 600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0);
    cam_intrin.D = cv::Mat::zeros(1, 5, CV_64F);
    intrinsics["cam0"] = cam_intrin;
    intrinsics["cam1"] = cam_intrin;

    std::vector<cv::Point3f> tag_corners_world = {
        {0.0, 0.0, 0.0}, {0.1651, 0.0, 0.0}, {0.1651, 0.1651, 0.0}, {0.0, 0.1651, 0.0}};

    std::unordered_map<std::string, std::vector<CameraObservation>> observations;
    std::unordered_map<std::string, cv::Vec3d>                      ground_truth_rotation;
    std::unordered_map<std::string, cv::Vec3d>                      ground_truth_translation;

    cv::Vec3d rvec0                  = {0.0, 0.0, 0.0};
    cv::Vec3d tvec0                  = {0.0, 0.0, 1.0};
    ground_truth_rotation["cam0"]    = rvec0;
    ground_truth_translation["cam0"] = tvec0;

    cv::Vec3d rvec1                  = {0.0, 0.1, 0.0};
    cv::Vec3d tvec1                  = {0.5, 0.0, 1.2};
    ground_truth_rotation["cam1"]    = rvec1;
    ground_truth_translation["cam1"] = tvec1;

    std::default_random_engine       rng;
    std::normal_distribution<double> noise(0.0, 0.5);

    for(const auto& [name, rvec] : ground_truth_rotation)
    {
        cv::Mat R;
        cv::Rodrigues(rvec, R);
        CameraObservation obs;
        obs.camera_name = name;

        for(const auto& pt : tag_corners_world)
        {
            cv::Mat pt3    = (cv::Mat_<double>(3, 1) << pt.x, pt.y, pt.z);
            cv::Mat pt_cam = R * pt3 + cv::Mat(ground_truth_translation[name]);

            double z = pt_cam.at<double>(2);
            if(z <= 0.001)
                continue;

            double u = cam_intrin.K.at<double>(0, 0) * pt_cam.at<double>(0) / z +
                       cam_intrin.K.at<double>(0, 2);
            double v = cam_intrin.K.at<double>(1, 1) * pt_cam.at<double>(1) / z +
                       cam_intrin.K.at<double>(1, 2);

            obs.image_points.emplace_back(static_cast<float>(u + noise(rng)),
                                          static_cast<float>(v + noise(rng)));
            obs.world_points.push_back(pt);
        }

        observations[name].push_back(obs);
    }

    optimizer.set_intrinsics(intrinsics);
    optimizer.set_observations(observations);

    std::unordered_map<std::string, CameraExtrinsic> initial_guesses;
    for(const auto& [name, rvec] : ground_truth_rotation)
    {
        CameraExtrinsic ext;
        ext.rotation_vector   = rvec;
        ext.translation       = ground_truth_translation[name];
        initial_guesses[name] = ext;
    }
    optimizer.set_initial_guesses(initial_guesses);

    optimizer.optimize();
    auto results = optimizer.get_extrinsics();

    for(const auto& [name, ext] : results)
    {
        std::cout << "Camera: " << name << "\n";
        std::cout << "  Rotation vector: " << ext.rotation_vector << "\n";
        std::cout << "  Translation:     " << ext.translation << "\n";

        const auto& expected_rvec = ground_truth_rotation[name];
        const auto& expected_tvec = ground_truth_translation[name];

        for(int i = 0; i < 3; ++i)
        {
            EXPECT_NEAR(ext.rotation_vector[i], expected_rvec[i], 0.1)
                << "Rotation mismatch for " << name;
            EXPECT_NEAR(ext.translation[i], expected_tvec[i], 0.1)
                << "Translation mismatch for " << name;
        }
    }
    std::cout << "All cameras processed successfully." << std::endl;
}
