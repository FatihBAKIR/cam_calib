//
// Created by fatih on 2/4/19.
//

#include "draw.hpp"
#include "calibrate.hpp"
#include <glm/glm.hpp>
#include <rtk/rtk_init.hpp>
#include <rtk/graphics/units.hpp>
#include <rtk/window.hpp>
#include <imgui_glfw.hpp>
#include <rtk/geometry/mesh.hpp>
#include <rtk/mesh_ops.hpp>
#include <rtk/gl/mesh.hpp>
#include <rtk/physics/transform.hpp>
#include <renderable.hpp>
#include <shader_man.hpp>
#include <material.hpp>
#include <lighting.hpp>
#include <scene.hpp>
#include <rtk/camera.hpp>

namespace {
    static std::shared_ptr<rtk::gl::program> get_phong_shader()
    {
        return app::load_shader("../shaders/phong.vert", "../shaders/phong.frag");
    }

    struct phong_material : app::material
    {
        rtk::gl::program& go() override
        {
            shader->set_variable("material.ambient", ambient);
            shader->set_variable("material.diffuse", diffuse);
            shader->set_variable("material.specular", specular);
            shader->set_variable("material.phong_exponent", phong_exponent);

            return *shader;
        }

        std::unique_ptr<material> clone() const override {
            auto res = std::make_unique<phong_material>();
            *res = *this;
            return res;
        }

        glm::vec3 diffuse;
        glm::vec3 specular;
        glm::vec3 ambient;
        float phong_exponent;
        std::shared_ptr<rtk::gl::program> shader;
    };

    auto get_phong_mat()
    {
        auto mat = std::make_shared<phong_material>();
        mat->shader = get_phong_shader();
        mat->ambient = {1, 1, 1};
        mat->diffuse = {1, 1, 1};
        mat->specular = {1, 1, 1};
        mat->phong_exponent = 16.f;
        return mat;
    }
}

namespace ar {
    void draw_thread::main() {

        using namespace rtk::literals;
        using namespace std::chrono_literals;

        rtk::rtk_init init;

        rtk::window w({960_px, 540_px});
        init_imgui(w);

        std::vector<rtk::geometry::mesh> meshes;
        meshes.emplace_back(rtk::geometry::primitive::cube());
        std::vector<rtk::gl::mesh> gl_meshes;
        gl_meshes.reserve(meshes.size());

        for (auto& m : meshes)
        {
            gl_meshes.emplace_back(create(m));

            auto normals = rtk::geometry::generate_normals(m);
            gl_meshes.back().add_vertex_data<glm::vec3>(1, normals);
        }

        rtk::camera cam(w);

        app::renderable teapot{};
        teapot.name = "teapot";
        teapot.mat = get_phong_mat();
        teapot.mesh = &gl_meshes[0];

        app::spot_light pl;
        pl.color = glm::vec3{ 25, 0, 0 };
        pl.transform->set_position({ -5, 5, 0 });

        app::scene ctx;
        ctx.objects.push_back(teapot);

        ctx.lights.push_back(pl);
        ctx.ambient = app::ambient_light{ glm::vec3{ .1, .1, .1 } };
        for (auto& l : ctx.lights) l.transform->look_at(teapot.transform->get_pos());

        cam.set_fov(get_fov());
        cam.set_aspect_ratio(get_aspect_ratio());

        m_running = true;
        while (m_run && !w.should_close())
        {
            ImGui_ImplGlfwGL3_NewFrame();

            w.begin_draw();
            w.set_viewport();

            render(cam, ctx);

            ImGui::Begin("Material");
            ImGui::ColorPicker3("Diffuse",
                                dynamic_cast<phong_material*>(teapot.mat.get())->diffuse.data.data);
            ImGui::ColorPicker3("Specular",
                                dynamic_cast<phong_material*>(teapot.mat.get())->specular.data.data);
            ImGui::End();

            ImGui::Render();
            ImGui_ImplGlfwGL3_RenderDrawData(ImGui::GetDrawData());

            w.end_draw();
        }
        m_running = false;
    }

    void draw_thread::run() {
        m_thread = std::thread([this]
        {
            main();
        });
        return;

        m_thread = std::thread([this]{
            cv::namedWindow("win", 1);

            const std::vector<cv::Point3f> world_points{{0, 0, 0}};

            m_running = true;

            while (m_run)
            {
                std::unique_lock lk{m_lock};
                while (m_q.empty())
                {
                    m_cv.wait(lk);
                    if (!m_run) break;
                }

                auto top = std::move(m_q.front()); m_q.pop_front();
                lk.unlock();

                if (top.found)
                {
                    std::vector<cv::Point2f> im_points;
                    cv::projectPoints(world_points, top.rvec, top.tvec,
                                      m_intrin, m_coeff, im_points);

                    std::cout << top.rvec << '\n';

                    cv::drawChessboardCorners(top.frame, {1, 1}, im_points, true);
                }

                cv::imshow("win", top.frame);
                if(cv::waitKey(30) >= 0) break;
            }

            m_running = false;
        });
    }

    void draw_thread::push_frame(const cv::Mat &m, const cv::Mat &rvec, const cv::Mat &tvec) {
        std::unique_lock lk{m_lock};
        m_q.emplace_back(frame_data{m, true, rvec, tvec});
        m_cv.notify_one();
    }

    void draw_thread::push_fail(const cv::Mat &m) {
        std::unique_lock lk{m_lock};
        m_q.emplace_back(frame_data{m, false});
        m_cv.notify_one();
    }
}