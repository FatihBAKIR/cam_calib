#include <opencv2/opencv.hpp>

#include <experimental/filesystem>
#include <vector>

#include "calibrate.hpp"

namespace fs = std::experimental::filesystem;

std::vector<fs::path> get_ims(const fs::path& p)
{
    std::vector<fs::path> res;
    for (auto & entry : fs::directory_iterator(p))
    {
        res.emplace_back(entry);
    }
    return res;
}

int main(int argc, char** argv)
{
    bool show_ims = false;
    auto ims = get_ims(argv[1]);
    show_ims = argc > 2 ? std::string(argv[2]) == "-v" : false;

    std::vector<cv::Mat> mats(ims.size());
    std::transform(ims.begin(), ims.end(), mats.begin(), [](auto& file){
        return cv::imread(file.string());
    });

    const auto checker_sz = cv::Size(5, 7);

    std::vector<std::vector<cv::Point2f>> corners(ims.size());
    std::transform(mats.begin(), mats.end(), corners.begin(), [&](auto& mat){
        return ar::find_corners(mat, checker_sz);
    });

    const auto points = ar::get_3d_points(checker_sz.height, checker_sz.width);

    std::vector<std::vector<cv::Point3f>> obj_points(ims.size(), points);

    cv::Mat intrin, dist_coeffs;
    std::vector<cv::Mat> trans_vectors, rot_vectors;

    cv::calibrateCamera(obj_points, corners, mats[0].size(),
            intrin, dist_coeffs,
            rot_vectors, trans_vectors);

    double tot_err = 0;

    for (int i = 0; i < mats.size(); ++i)
    {
        std::vector<cv::Point2f> im_points;
        cv::projectPoints(obj_points[i], rot_vectors[i], trans_vectors[i],
                intrin, dist_coeffs, im_points);

        if (show_ims)
        {
            auto im = mats[i].clone();
            cv::drawChessboardCorners(im, checker_sz, corners[i], true);
            cv::drawChessboardCorners(im, checker_sz, im_points, true);
            cv::imshow("im", im);
            if (cv::waitKey(0) == 27)
            {
                show_ims = false;
            }
        }

        auto err = norm(cv::Mat(corners[i]), cv::Mat(im_points), cv::NORM_L2);
        tot_err += err * err;
    }

    auto out = ar::serialize(intrin, dist_coeffs, mats[0].size());
    out["sq_error"] = tot_err;
    std::cout << out << '\n';
}
