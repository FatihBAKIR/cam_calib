//
// Created by fatih on 2/4/19.
//

#include <experimental/filesystem>
#include <opencv2/opencv.hpp>

namespace fs = std::experimental::filesystem;

int main(int argc, char** argv)
{
    if (argc < 2){
        std::cout << "need the output dir\n";
        return 1;
    }
    cv::VideoCapture cap(0);
    if (!cap.isOpened())
    {
        throw std::runtime_error("couldn't open camera");
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 960);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 540);

    std::vector<cv::Mat> frames;

    cv::namedWindow("edges",1);
    for(;;)
    {
        cv::Mat frame;
        cap >> frame;
        cv::imshow("edges", frame);

        auto key = cv::waitKey(30);
        if (key == 67 || key == 99 || key == 32)
        {
            frames.push_back(std::move(frame));
            std::cerr << "frame\n";
        }
        else if (key == 27)
        {
            break;
        }
    }

    fs::path root{argv[1]};
    int x = 1;
    for (auto& im : frames)
    {
        auto p = root / ("frame_" + std::to_string(x++) + ".png");
        cv::imwrite(p.string(), im);
    }

    std::cout << root << '\n';
}