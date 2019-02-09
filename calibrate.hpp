//
// Created by fatih on 2/4/19.
//

#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace ar
{
    std::vector<cv::Point2f>
    find_corners(const cv::Mat& rgb, const cv::Size& sz, bool fast = false);

    YAML::Node
    serialize(const cv::Mat &i, const cv::Mat &dist, const cv::Size &sz);

    std::pair<cv::Mat, cv::Mat>
    deserialize(const YAML::Node& node);

    cv::Mat
    draw_corners(const cv::Mat& rgb, const cv::Size& sz, const std::vector<cv::Point2f>& points);

    inline std::vector<cv::Point3f> get_3d_points(int row, int col)
    {
        std::vector<cv::Point3f> points;
        for (int i = 0; i < row; ++i)
        {
            for (int j = 0; j < col; ++j)
            {
                points.emplace_back(j, i, 0);
            }
        }
        return points;
    }
}