#include <iostream>
#include <chrono>
#include <thread>

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjvPerturb pert;
mjrContext con;   // 新增GPU渲染上下文
GLFWwindow* window = nullptr;

void glfw_resize(GLFWwindow* win, int w, int h)
{
    (void)win; (void)w; (void)h;
}

int main(int argc, const char** argv)
{
    const char* xml_path = "black_description.xml";
    char errmsg[1024];

    mjModel* model = mj_loadXML(xml_path, nullptr, errmsg, sizeof(errmsg));
    if (!model)
    {
        std::cerr << "load xml failed: " << errmsg << std::endl;
        return -1;
    }
    mjData* data = mj_makeData(model);

    if (!glfwInit())
    {
        std::cerr << "glfw init fail\n";
        return -1;
    }
    window = glfwCreateWindow(1200, 800, "mujoco sim", nullptr, nullptr);
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, glfw_resize);

    // 初始化mujoco渲染资源
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultPerturb(&pert);
    mjv_makeScene(model, &scn, 2000);

    mjr_defaultContext(&con);
    mjr_makeContext(model, &con, mjFONTSCALE_100); // 创建GPU上下文[[(MuJoCo)]](https://mujoco.readthedocs.io/en/stable/_sources/programming/visualization.rst.txt?f_link_type=f_linkinlinenote&flow_extra=eyJpbmxpbmVfZGlzcGxheV9wb3NpdGlvbiI6MCwiZG9jX3Bvc2l0aW9uIjowLCJkb2NfaWQiOiIxZjkxNDk2Yzk0MDRhMThjLWQwYWZjOGUzMDA3ZjY1Y2MifQ%3D%3D "[(MuJoCo)]")

    cam.azimuth = 90;
    cam.elevation = -20;
    cam.distance = 2.5;

    while (!glfwWindowShouldClose(window))
    {
        auto step_start = std::chrono::steady_clock::now();

        for(int i=0; i<12; i++)
        {
            data->ctrl[i] = 0.0;
        }

        mj_step(model, data);

        glfwMakeContextCurrent(window);
        mjv_updateScene(model, data, &opt, &pert, &cam, mjCAT_ALL, &scn);

        mjrRect rect{0,0,0,0};
        glfwGetFramebufferSize(window, &rect.width, &rect.height);
        mjr_render(rect, &scn, &con);   // 补上第三个参数 con

        glfwSwapBuffers(window);
        glfwPollEvents();

        double dt = model->opt.timestep;
        double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - step_start).count();
        double time_left = dt - elapsed;
        if(time_left > 0.0)
        {
            std::this_thread::sleep_for(std::chrono::duration<double>(time_left));
        }
    }

    mjr_freeContext(&con);
    mjv_freeScene(&scn);
    glfwDestroyWindow(window);
    glfwTerminate();

    mj_deleteData(data);
    mj_deleteModel(model);
    return 0;
}
