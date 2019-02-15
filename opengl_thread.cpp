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

#include <iostream>
#include <chrono>

#include <rtk/imgui.h>
#include <rtk/geometry/mesh.hpp>
#include <rtk/rtk_init.hpp>
#include <rtk/asset/mesh_import.hpp>
#include <rtk/mesh_ops.hpp>
#include <rtk/texture/tex2d.hpp>

#include "material.hpp"
#include "lighting.hpp"
#include "scene.hpp"
#include "shader_man.hpp"
#include "cam_controller.hpp"
#include "imgui_glfw.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <deque>

#include <glm/gtx/quaternion.hpp>

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

        shader->set_variable("material.textured", bool(tex));
        shader->set_variable("material.specd", bool(spec));
        shader->set_variable("material.normaled", bool(normal_map));

        if (tex)
        {
            shader->set_variable("tex", 5, *tex);
            shader->set_variable("uv_scale", tex_scale);
        }

        if (spec)
        {
            shader->set_variable("spec", 6, *spec);
            shader->set_variable("uv_scale", tex_scale);
        }

        if (normal_map)
        {
            shader->set_variable("normal_map", 7, *normal_map);
            shader->set_variable("uv_scale", tex_scale);
        }

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

    std::shared_ptr<rtk::gl::texture2d> normal_map;
    std::shared_ptr<rtk::gl::texture2d> tex;
    std::shared_ptr<rtk::gl::texture2d> spec;
    float tex_scale = 1;

    std::shared_ptr<rtk::gl::program> shader;
};

std::shared_ptr<rtk::gl::texture2d> load_tex(const std::string& path, bool mipmap = true)
{
    auto tex = rtk::graphics::load_texture(path);
    return std::make_shared<rtk::gl::texture2d>(tex, mipmap);
}

std::shared_ptr<rtk::gl::cubemap> load_cubemap(std::initializer_list<std::string> paths)
{
    std::vector<rtk::graphics::texture2d> texs;
    for (auto& s : paths)
    {
        texs.emplace_back(rtk::graphics::load_texture(s));
    }
    std::array<rtk::graphics::unsafe_texture*, 6> faces;
    std::transform(texs.begin(), texs.begin() + 6, faces.begin(), [](auto& x) {
        return &x;
    });
    return std::make_shared<rtk::gl::cubemap>(faces);
}

auto load_mat(const YAML::Node& mat_def)
{
    auto prog = app::load_shader(
            "../shaders/" + mat_def["vert"].as<std::string>(),
            "../shaders/" + mat_def["frag"].as<std::string>());

    auto get_vec3 = [](const YAML::Node& n) -> glm::vec3 {
        return { n[0].as<float>(), n[1].as<float>(), n[2].as<float>() };
    };

    std::shared_ptr<app::material> res;
    if (auto pname = mat_def["name"].as<std::string>(); pname == "Phong")
    {
        auto mat = std::make_shared<phong_material>();

        mat->shader = prog;
        mat->ambient = get_vec3(mat_def["ambient"]);
        mat->specular = get_vec3(mat_def["specular"]);
        mat->diffuse = get_vec3(mat_def["diffuse"]);
        if (mat_def["phong"])
        {
            mat->phong_exponent = mat_def["phong"].as<float>();
        }
        else
        {
            mat->phong_exponent = 1;
        }

        if (mat_def["tex_scale"])
        {
            mat->tex_scale = mat_def["tex_scale"].as<float>();
        }

        if (mat_def["diffuse_map"])
        {
            mat->tex = load_tex("../assets/" + mat_def["diffuse_map"].as<std::string>());
        }

        if (mat_def["normal_map"])
        {
            mat->normal_map = load_tex("../assets/" + mat_def["normal_map"].as<std::string>());
        }

        if (mat_def["specular_map"])
        {
            mat->spec = load_tex("../assets/" + mat_def["specular_map"].as<std::string>());
        }

        res = mat;
    }

    return res;
}

auto load_mats(const std::string& path)
{
    std::ifstream m(path);
    YAML::Node root = YAML::Load(m);

    std::unordered_map<std::string, std::shared_ptr<app::material>> res;
    for (auto mat : root)
    {
        auto n = mat.first.as<std::string>();
        res.emplace(n, load_mat(mat.second));
    }

    return res;
}

auto get_ground(const rtk::gl::mesh& mesh)
{
    app::renderable ground{};
    ground.name = "ground 2";

    ground.mat = load_mats("../assets/materials/hardwood.yaml")["hardwood"];
    ground.mesh = &mesh;
    ground.cast_shadow = false;

    ground.transform->translate(rtk::vectors::down * 0.5f);
    //ground.transform->rotate({-90, 0, 0});
    ground.transform->set_scale({5, 5, 1});
    return ground;
}

auto get_cube(const rtk::gl::mesh& mesh)
{
    app::renderable ground{};
    ground.name = "ground 2";

    ground.mat = load_mats("../assets/materials/teapot.yaml")["teapot"];
    ground.mesh = &mesh;
    ground.cast_shadow = false;
    ground.transform->rotate({-90, 0, 0});

    return ground;
}

namespace app
{
    std::shared_ptr<rtk::gl::texture2d>
    render_to_tex(const rtk::camera& cam, const app::scene& ctx);

    std::shared_ptr<rtk::gl::texture2d>
    render_to_tex(const glm::mat4& vp, const glm::vec3& pos, rtk::resolution sz, const app::scene& ctx);

    std::shared_ptr<rtk::gl::texture2d>
    detect_edges(const std::shared_ptr<rtk::gl::texture2d>& depth);

    std::shared_ptr<rtk::gl::texture2d>
    draw_tex(const std::shared_ptr<rtk::gl::texture2d>& scene);

    std::shared_ptr<rtk::gl::texture2d>
    object_pass(const rtk::camera& cam, const scene& ctx);

    std::shared_ptr<rtk::gl::texture2d>
    draw_tex(const std::shared_ptr<rtk::gl::texture2d>& scene);

    void
    overlay(const std::shared_ptr<rtk::gl::texture2d>& a, const std::shared_ptr<rtk::gl::texture2d>& b);
}

std::shared_ptr<rtk::gl::texture2d> mat_to_tex(const cv::Mat& m)
{
    cv::Mat flipped;
    cv::flip(m, flipped, 0);

    Expects(m.type() == CV_8UC3);
    rtk::graphics::unsafe_texture ut;
    ut.m_data = flipped.data;
    ut.m_height = m.rows;
    ut.m_width = m.cols;
    ut.m_fmt = rtk::graphics::pixel_format::bgr_byte;
    return std::make_shared<rtk::gl::texture2d>(ut);
}

glm::mat4 mat_to_mat4(const cv::Mat& rvec)
{
    using namespace glm;

    mat4 x;
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            x[i][j] = rvec.at<double>(i, j);
        }
    }

    return x;
}

glm::quat rot_to_glm(const cv::Mat& rvec)
{
    using namespace glm;

    auto x = mat_to_mat4(rvec);

    return conjugate(toQuat(x));
}

namespace ar {
    void draw_thread::main() {
        using namespace rtk::literals;
        using namespace std::chrono_literals;
        using namespace app;

        rtk::rtk_init init;

        rtk::window w({rtk::pixels(m_sz.width), rtk::pixels(m_sz.height)});
        init_imgui(w);

        auto mats = load_mats("/home/fatih/rtk/assets/materials/muro.yaml");

        std::vector<rtk::geometry::mesh> meshes;
        meshes = rtk::assets::load_meshes("/home/fatih/rtk/assets/muro/muro.obj");
        meshes.push_back(rtk::geometry::primitive::cube());
        meshes.push_back(rtk::geometry::primitive::quad());

        std::vector<rtk::gl::mesh> gl_meshes;
        gl_meshes.reserve(meshes.size());

        for (auto& m : meshes)
        {
            gl_meshes.emplace_back(create(m));

            if (!m.has_normals())
            {
                auto normals = rtk::geometry::generate_normals(m);
                gl_meshes.back().add_vertex_data<glm::vec3>(1, normals);
            }

            if (!m.has_uvs())
                continue;

            auto [bitans, tans] = rtk::geometry::generate_btn(m);
            gl_meshes.back().add_vertex_data<glm::vec3>(3, bitans);
            gl_meshes.back().add_vertex_data<glm::vec3>(4, tans);
        }

        std::vector<renderable> r;

        auto& mesh = meshes[1];
        auto max = std::max({mesh.get_bbox().extent.x, mesh.get_bbox().extent.y, mesh.get_bbox().extent.z});

        for (int i = 0; i < gl_meshes.size() - 2; ++i)
        {
            auto& headm = meshes[i];
            renderable head{};
            head.name = "mesh_" + std::to_string(i);
            head.mat = mats[headm.get_mat()];
            head.mesh = &gl_meshes[i];

            head.transform->set_scale(glm::vec3(6.f, 6.f, 6.f) / glm::vec3(max, max, max));
            head.transform->set_position(-mesh.get_bbox().position / glm::vec3(max, max, max));
            head.transform->rotate({ -90, 0, 0 });
            r.emplace_back(std::move(head));
        }

        auto ground = get_ground(gl_meshes.back());

        area_light pl;
        pl.color = glm::vec3{ 25, 25, 25 };
        pl.transform->set_position({ 0, 5, -3 });

        auto c = std::make_unique<rtk::camera>(w);
        c->set_fov(glm::radians(get_fov()));
        c->set_aspect_ratio(get_aspect_ratio());
        c->get_transform()->set_position({0,0,-5});

        scene ctx;
        ctx.objects = r;
        //ctx.objects.push_back(ground);

        ctx.lights.push_back(pl);

        ctx.ambient = ambient_light{ glm::vec3{ .2, .2, .2 } };

        auto proj = glm::perspective<float>(glm::radians(get_fov()), get_aspect_ratio(), 0.1, 100);

        m_running = true;
        while (!w.should_close())
        {
            std::unique_lock lk{m_lock};
            while (m_q.empty())
            {
                m_cv.wait(lk);
                if (!m_run) break;
            }

            auto top = std::move(m_q.front()); m_q.pop_front();
            lk.unlock();

            glm::vec3 pos;
            glm::mat4 vp;

            if (top.found)
            {
                cv::Mat r;
                cv::Rodrigues(top.rvec, r);
                cv::Mat viewMatrix(4, 4, CV_64F);

                for(unsigned int row=0; row<3; ++row)
                {
                    for(unsigned int col=0; col<3; ++col)
                    {
                        viewMatrix.at<double>(row, col) = r.at<double>(row, col);
                    }
                    viewMatrix.at<double>(row, 3) = top.tvec.at<double>(row, 0);
                }
                viewMatrix.at<double>(3, 3) = 1.0f;

                cv::Mat cvToGl = cv::Mat::zeros(4, 4, CV_64F);
                cvToGl.at<double>(0, 0) = 1.0f;
                cvToGl.at<double>(1, 1) = -1.0f;
                cvToGl.at<double>(2, 2) = -1.0f;
                cvToGl.at<double>(3, 3) = 1.0f;
                viewMatrix = cvToGl * viewMatrix;

                cv::Mat glViewMatrix = cv::Mat::zeros(4, 4, CV_64F);
                cv::transpose(viewMatrix , glViewMatrix);

                glm::mat4 m = mat_to_mat4(glViewMatrix);
                pos = m * glm::vec4(0, 0, 0, 1);
                vp = proj * m;

                static auto world_points = ar::get_3d_points(7, 5);
                std::vector<cv::Point2f> im_points;
                cv::projectPoints(world_points, top.rvec, top.tvec,
                                  m_intrin, m_coeff, im_points);
                cv::drawChessboardCorners(top.frame, {5, 7}, im_points, true);
            }
            auto t = mat_to_tex(top.frame);

            c->sync();
            auto out = render_to_tex(vp, pos, c->get_resolution(), ctx);

            w.begin_draw();
            w.set_viewport();

            overlay(out, t);

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