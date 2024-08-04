#pragma once

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

class MapVisualizer {
public:
    MapVisualizer(const cv::Mat& map_image, const Eigen::Vector2f& map_origin, float image_resolution);

    void displayPointOnMap(const float& x, const float& y);
    std::vector<LaserPoint> getOverlappingPoints(const Eigen::Vector3f& position, const std::vector<LaserPoint>& points);
    void displayLaserPoints(const std::vector<LaserPoint>& points);

    static void onMouse(int event, int x, int y, int, void* userdata);

private:
    cv::Mat map_image;
    Eigen::Vector2f map_origin;
    float image_resolution;

    std::pair<int, int> mapToPixel(const float& x, const float& y);
    std::pair<float, float> pixelToMap(const int& x, const int& y);
};