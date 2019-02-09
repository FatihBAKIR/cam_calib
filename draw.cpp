//
// Created by fatih on 2/4/19.
//

#include <opencv2/opencv.hpp>
#include "calibrate.hpp"
#include "draw.hpp"

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
    cv::VideoCapture cap(0);
    if (!cap.isOpened())
    {
        throw std::runtime_error("couldn't open camera");
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 960);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 540);
    cap.set(cv::CAP_PROP_AUTO_WB, 1);
    cap.set(cv::CAP_PROP_FPS, 40.00);
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
    cap.set(cv::CAP_PROP_CONTRAST, 128);
    cap.set(cv::CAP_PROP_SATURATION, 128);

    auto config = read_config(argv[1]);
    auto [intrin, coeff] = ar::deserialize(config);

    ar::draw_thread drawer(intrin, coeff);

    const auto checker_sz = cv::Size(5, 7);

    const auto world_points = ar::get_3d_points(checker_sz.height, checker_sz.width);

    drawer.run();
    while (!drawer.running());

    bool use_prev = false;
    cv::Mat frame, rvec, tvec;
    while(drawer.running())
    {
        cap >> frame;

        auto corners = ar::find_corners(frame, checker_sz, true);
        auto prv = use_prev;
        use_prev = false;
        if (corners.size() == world_points.size())
        {
            if (cv::solvePnPRansac(world_points, corners, intrin, coeff, rvec, tvec, prv, use_prev))
            {
                drawer.push_frame(frame, rvec, tvec);
                use_prev = true;
            }
        }
        else
        {
            drawer.push_fail(frame);
        }
    }
}