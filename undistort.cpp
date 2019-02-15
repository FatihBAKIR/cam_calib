//
// Created by fatih on 2/8/19.
//

#include <opencv2/opencv.hpp>
#include "calibrate.hpp"

#include <experimental/filesystem>
#include <yaml-cpp/yaml.h>

namespace fs = std::experimental::filesystem;

YAML::Node read_config(const fs::path& path)
{
    std::ifstream cam_file{path};
    if (!cam_file)
    {
        throw std::runtime_error("couldn't read file");
    }
    std::string s{std::istreambuf_iterator<char>(cam_file), std::istreambuf_iterator<char>{}};
    YAML::Node cam = YAML::Load(s);
    return cam;
}

int main(int argc, char** argv)
{
    auto im_path = argv[1];
    auto cam_path = argv[2];

    auto conf = read_config(cam_path);
    auto im = cv::imread(im_path);

    auto [intrin, coeff] = ar::deserialize(conf);

    std::cout << intrin << '\n';

    cv::Mat u1, u2;
    //cv::initUndistortRectifyMap(intrin, coeff, cv::Mat(), intrin, { 2896, 2896 }, CV_16SC2, u1, u2);

    cv::Mat out;
    //cv::remap(im, out, u1, u2, cv::INTER_LINEAR);
    cv::undistort(im, out, intrin, coeff);

    cv::namedWindow("im", CV_WINDOW_NORMAL);
    cv::resizeWindow("im", 1000, 1000);
    cv::imshow("im", out);
    cv::waitKey(0);
}