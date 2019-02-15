//
// Created by fatih on 2/4/19.
//

#pragma once

#include <opencv2/opencv.hpp>
#include <atomic>
#include <deque>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace ar
{
    class draw_thread
    {
    public:
        draw_thread(const cv::Mat& intrin, const cv::Mat& coeff, const cv::Size& sz)
            : m_intrin(intrin), m_coeff(coeff), m_sz{sz} {
            cv::Point2d principal;
            double fovx, fovy, focal_len, aspect;
            cv::calibrationMatrixValues(m_intrin, m_sz, 0, 0, fovx, fovy, focal_len, principal, aspect);

            m_aspect = sz.width / float(sz.height);
            m_fov = fovy;
        }

        draw_thread(const draw_thread&) = delete;

        void main();

        void push_frame(
                const cv::Mat& m,
                const cv::Mat& rvec, const cv::Mat& tvec);

        void push_fail(const cv::Mat& m);

        void run();

        bool running()
        {
            return m_running;
        }

        void stop()
        {
            m_run = false;
            m_cv.notify_one();
        }

        ~draw_thread()
        {
            m_thread.join();
        }

    private:

        float get_fov() const
        {
            return m_fov;
        }

        float get_aspect_ratio() const
        {
            return m_aspect;
        }

        struct frame_data
        {
            cv::Mat frame;
            bool found;
            cv::Mat rvec, tvec;
        };

        std::deque<frame_data> m_q;

        std::mutex m_lock;
        std::condition_variable m_cv;

        float m_fov;
        float m_aspect;

        cv::Size m_sz;
        cv::Mat m_intrin, m_coeff;
        std::atomic_bool m_run = true;

        std::thread m_thread;
        std::atomic_bool m_running = false;
    };
}
