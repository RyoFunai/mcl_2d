#include "mcl_2d/cv_visualizer.hpp"

MapVisualizer::MapVisualizer(const cv::Mat& map_image, const Eigen::Vector2f& map_origin, float image_resolution)
    : map_image(map_image), map_origin(map_origin), image_resolution(image_resolution) {}

void MapVisualizer::displayPointOnMap(const float& x, const float& y) {
    auto [pixel_x, pixel_y] = mapToPixel(x, y);
    auto [map_x, map_y] = pixelToMap(pixel_x, pixel_y);

    cv::Mat color_map_image;
    cv::cvtColor(map_image, color_map_image, cv::COLOR_GRAY2BGR);

    std::string text = "(" + std::to_string(map_x) + ", " + std::to_string(map_y) + ")";
    cv::putText(color_map_image, text, cv::Point(pixel_x, pixel_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
    cv::circle(color_map_image, cv::Point(pixel_x, pixel_y), 4, cv::Scalar(0, 255, 0), -1);

    auto [o_x, o_y] = mapToPixel(0.f, 0.f);
    cv::circle(color_map_image, cv::Point(o_x, o_y), 2, cv::Scalar(0, 0, 255), -1);

    cv::imshow("Map Image", color_map_image);
    cv::waitKey(0);
}

std::vector<LaserPoint> MapVisualizer::getOverlappingPoints(const Eigen::Vector3f& position, const std::vector<LaserPoint>& points) {
    std::vector<LaserPoint> overlapping_points;

    for (const auto& point : points) {
        auto [image_pt_x, image_pt_y] = mapToPixel(point.x, point.y);

        if (image_pt_x >= 0 && image_pt_x < map_image.cols && image_pt_y >= 0 && image_pt_y < map_image.rows) {
            if (map_image.at<uchar>(image_pt_y, image_pt_x) == 0) {
                overlapping_points.push_back(point);
            }
        }
    }

    return overlapping_points;
}

void MapVisualizer::displayLaserPoints(const std::vector<LaserPoint>& points) {
    cv::Mat color_map_image;
    if (map_image.channels() == 1) {
        cv::cvtColor(map_image, color_map_image, cv::COLOR_GRAY2BGR);
    } else {
        color_map_image = map_image.clone();
    }

    for (const auto& point : points) {
        auto [pixel_x, pixel_y] = mapToPixel(point.x, point.y);
        if (pixel_x >= 0 && pixel_x < color_map_image.cols && pixel_y >= 0 && pixel_y < color_map_image.rows) {
            cv::circle(color_map_image, cv::Point(pixel_x, pixel_y), 1, cv::Scalar(0, 0, 255), -1);
        }
    }

    cv::imshow("Laser Points", color_map_image);
    cv::waitKey(0);
}

void MapVisualizer::onMouse(int event, int x, int y, int, void* userdata) {
    if (event == cv::EVENT_MOUSEMOVE) {
        cv::Mat* image = reinterpret_cast<cv::Mat*>(userdata);
        if (x >= 0 && x < image->cols && y >= 0 && y < image->rows) {
            uchar pixel = image->at<uchar>(y, x);
            RCLCPP_INFO(rclcpp::get_logger("map_visualizer"), "Pixel value at (%d, %d): %d", x, y, static_cast<int>(pixel));
        }
    }
}

std::pair<int, int> MapVisualizer::mapToPixel(const float& x, const float& y) {
    int pixel_x = static_cast<int>((x - map_origin.x()) / image_resolution);
    int pixel_y = static_cast<int>((y - map_origin.y()) / image_resolution);
    return std::make_pair(pixel_x, pixel_y);
}

std::pair<float, float> MapVisualizer::pixelToMap(const int& x, const int& y) {
    float map_x = (x * image_resolution) + map_origin.x();
    float map_y = (y * image_resolution) + map_origin.y();
    return std::make_pair(map_x, map_y);
}