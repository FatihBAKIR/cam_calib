//
// Created by fatih on 2/4/19.
//

#include "calibrate.hpp"

namespace ar {
    static thread_local cv::Mat gray;
    std::vector<cv::Point2f> find_corners(const cv::Mat &rgb, const cv::Size &sz, bool fast) {
        cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);

        std::vector<cv::Point2f> res;
        bool found = !fast ? cv::findChessboardCorners(gray, sz, res) : cv::findChessboardCorners(gray, sz, res, cv::CALIB_CB_FAST_CHECK);

        if (found) {
            cornerSubPix(gray, res, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 50, 0.00001));
        }
        return res;
    }

    YAML::Node serialize(const cv::Mat &im, const cv::Mat &dist, const cv::Size &sz)
    {
        cv::Point2d principal;
        double fovx, fovy, focal_len, aspect;
        cv::calibrationMatrixValues(im, sz, 0, 0, fovx, fovy, focal_len, principal, aspect);

        YAML::Node fov;
        fov.push_back(fovx);
        fov.push_back(fovy);

        YAML::Node prin;
        prin.push_back(principal.x);
        prin.push_back(principal.y);

        YAML::Node disto;
        disto.push_back(dist.at<double>(0));
        disto.push_back(dist.at<double>(1));
        disto.push_back(dist.at<double>(2));
        disto.push_back(dist.at<double>(3));

        std::vector<std::vector<double>> raw_mat;
        for (int i = 0; i < im.rows; ++i)
        {
            raw_mat.emplace_back();
            for (int j = 0; j < im.cols; ++j)
            {
                raw_mat.back().emplace_back(im.at<double>(i, j));
            }
        }

        YAML::Node intrin;
        intrin["fov"] = fov;
        intrin["focal_len"] = focal_len;
        intrin["aspect"] = aspect;
        intrin["principal"] = prin;
        intrin["raw"] = raw_mat;

        YAML::Node res;
        res["intrin"] = intrin;
        res["dist"] = disto;

        return res;
    }

    std::pair<cv::Mat, cv::Mat> deserialize(const YAML::Node &node) {
        auto raw_mat = node["intrin"]["raw"];
        cv::Mat int_mat(raw_mat.size(), raw_mat[0].size(), CV_64FC1);
        for (int i = 0; i < int_mat.rows; ++i)
        {
            auto r = node["intrin"]["raw"][i].as<std::vector<double>>();
            for (int j = 0; j < int_mat.cols; ++j)
            {
                int_mat.at<double>(i, j) = r[j];
            }
        }

        auto dist_coeff = node["dist"].as<std::vector<double>>();
        cv::Mat m{dist_coeff};

        return {int_mat, m};
    }

    cv::Mat draw_corners(const cv::Mat &rgb, const cv::Size& sz, const std::vector<cv::Point2f> &points) {
        if (points.size() < sz.width * sz.height)
        {
            return rgb;
        }

        cv::Mat res = rgb.clone();
        cv::drawChessboardCorners(res, sz, points, true);
        return res;
    }
}
