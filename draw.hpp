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
        draw_thread(const cv::Mat& intrin, const cv::Mat& coeff)
            : m_intrin(intrin), m_coeff(coeff) {}

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

        float get_fov() const;
        float get_aspect_ratio() const;

        struct frame_data
        {
            cv::Mat frame;
            bool found;
            cv::Mat rvec, tvec;
        };

        std::deque<frame_data> m_q;

        std::mutex m_lock;
        std::condition_variable m_cv;

        cv::Mat m_intrin, m_coeff;
        std::atomic_bool m_run = true;

        std::thread m_thread;
        std::atomic_bool m_running = false;
    };
}
