// VMEngine++.cpp : Этот файл содержит функцию "main". Здесь начинается и заканчивается выполнение программы.
//

#define GLEW_STATIC

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <gl/glew.h>
#include <GLFW/glfw3.h>

#include <iostream>
#include <cmath>
#include <vector>;

#include "Input.h";


#include <chrono>
#include <thread>
#include "VertexAttribute.h"
#include "ChunkController.h"
#include "AssetsController.h"
#include "VertexBufferObject.h";
#include "Time.h"
#include "GameObject.h"
#include "Camera.h"
#include "GhostCameraController.h"
#include "Debug.h"
#include "Render.h"

#include <chrono>
#include <typeinfo>

void GLAPIENTRY MessageCallback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar* message, const void* userParam) {
    std::cout << "[OpenGL Error](" << type << ") " << message << std::endl;
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
    glUniform2f(AssetsController::Shaders["raymarch"]->GetParam("u_resolution"), width, height);

    if (Camera::MainCamera != nullptr) {
        Camera::MainCamera->Width = width;
        Camera::MainCamera->Height = height;
    }
}

Render RenderConfig = Render();

int main()
{
    GLFWwindow* window;

    /* Initialize the library */
    if (!glfwInit())
        return -1;

    int monitorsCount = 0;
    GLFWmonitor **monitors = glfwGetMonitors(&monitorsCount);

    for (int i = 0; i < monitorsCount; i++) {
        std::cout << glfwGetMonitorName(monitors[i]) << "\n";
    }

    const GLFWvidmode* mode = glfwGetVideoMode(monitors[1]);

    glfwWindowHint(GLFW_RED_BITS, mode->redBits);
    glfwWindowHint(GLFW_GREEN_BITS, mode->greenBits);
    glfwWindowHint(GLFW_BLUE_BITS, mode->blueBits);
    glfwWindowHint(GLFW_REFRESH_RATE, mode->refreshRate);
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE);

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    /* Create a windowed mode window and its OpenGL context */
    window = glfwCreateWindow(1600, 900, "Hello World", NULL, NULL);

    if (!window)
    {
        glfwTerminate();
        return -1;
    }

    /* Make the window's context current */
    glfwMakeContextCurrent(window);
    glfwSwapInterval(RenderConfig.swapInterval ? 1 : 0);

    /* OpenGL info */
    std::cout << glGetString(GL_VERSION) << "\n";
    std::cout << glGetString(GL_VENDOR) << "\n";
    std::cout << glGetString(GL_RENDERER) << "\n";

    Input::InputStartup(window);

    GLenum err = glewInit();
    if (GLEW_OK != err)
    {
        /* Problem: glewInit failed, something is seriously wrong. */
        std::cerr << "Error: " << glewGetErrorString(err) << std::endl;
    }
    std::cerr << "Status: Using GLEW " << glewGetString(GLEW_VERSION) << std::endl;

    Time::Load();

    AssetsController::Load();
    AssetsController::Meshes["plane"] = AssetsController::CreatePlane();

    AssetsController::VBOs["plane"] = new VertexBufferObject(&(Vertex::Info), AssetsController::Meshes["plane"], GL_QUADS);
    AssetsController::VBOs["plane"]->Bind();


    glPointSize(10.0f);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glEnable(GL_TEXTURE_2D);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_DEBUG_OUTPUT);

    glDebugMessageCallback(MessageCallback, 0);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    AssetsController::Shaders["raymarch"]->Use();

    ChunkController::GenerateArea(Vector3::zero);

    GameObject ghost = GameObject();
    ghost.components.push_back(new Camera(&ghost, true));
    ghost.components.push_back(new GhostCameraController(&ghost));


    //ghost.AddComponent(&Camera(&ghost, true));
    //GhostCameraController g = ghost.GetComponent<GhostCameraController>();
        
    ghost.transform->position = Vector3(0, 0, -20);

    ghost.Start();

    glUniform2f(AssetsController::Shaders["raymarch"]->GetParam("u_resolution"), 1600, 900);


    // Uncommenting this call will result in wireframe polygons.
    //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);


    float _dtTick = 0;



    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsLight();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");


    if (Camera::MainCamera != nullptr) {
        glViewport(0, 0, Camera::MainCamera->Width, Camera::MainCamera->Height);
    }

    /* Loop until the user closes the window */
    while (!glfwWindowShouldClose(window))
    {

        /* Poll for and process events */
        glfwPollEvents();


        Time::Tick();


        /* Tick here */
        ChunkController::Tick();
        ghost.Update();
        RenderConfig.Tick();

        if (Camera::MainCamera != nullptr) {
            AssetsController::Shaders["raymarch"]->Use();
            glUniformMatrix4fv(AssetsController::Shaders["raymarch"]->GetParam("view"), 2, GL_FALSE, &Camera::MainCamera->View.Row0.x);
            glUniformMatrix4fv(AssetsController::Shaders["raymarch"]->GetParam("projection"), 2, GL_FALSE, &Camera::MainCamera->Projection.Row0.x);
            glUniform2f(AssetsController::Shaders["raymarch"]->GetParam("u_resolution"), Camera::MainCamera->Width, Camera::MainCamera->Height);
        }

        glUniform1f(AssetsController::Shaders["raymarch"]->GetParam("u_time"), Time::Alive());
        /* Render here */
        //glClearColor(1, 0, 0, 1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);



        //// quad drawcall
        AssetsController::Shaders["raymarch"]->Use();
        AssetsController::VBOs["plane"]->Draw();




        if (&ghost != nullptr) {
            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();

            ImGui::Begin("Properties");

            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
            ImGui::Text("Chunk mem: %s", Debug::SizeSuffix(ChunkController::Chunks[0]->DataLength * sizeof(float)).c_str());
            ImGui::Text("alive: %f", Time::Alive());
            ImGui::Text("delta time: %f", Time::DeltaTime());


            if (ImGui::CollapsingHeader("Render")) {
                ImGui::Checkbox("VSync", &RenderConfig.swapInterval);
                ImGui::SliderInt("Iterations visualize mode", &RenderConfig.VISUALIZE_ITERATIONS_MODE, 0, RenderConfig.MAX_VISUALIZE_ITERATIONS_MODE);
                ImGui::SliderInt("Backtraking mode", &RenderConfig.DEV_BACKTRACKING_MODE, 0, RenderConfig.MAX_DEV_BACKTRACKING_MODE);
            }

            if (ImGui::CollapsingHeader("Camera"))
            {
                float* p[] = { &ghost.transform->position.x, &ghost.transform->position.y, &ghost.transform->position.z };
                ImGui::InputFloat3("Camera pos", *p);
                //ImGui::SliderFloat("Camera speed", );

            }

            if (ImGui::CollapsingHeader("Import"))
            {
                int importCount = AssetsController::ImportModels.size();
                ImGui::BeginGroup();
                for (int i = 0; i < importCount; i++) {
                    ImGui::Selectable(AssetsController::ImportModels[i].c_str());
                    if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0))
                    {
                        Octree* oct = nullptr;
                        oct = AssetsController::LoadQB("models\\" + AssetsController::ImportModels[i]);
                        delete ChunkController::Chunks[0]->ChunkOctree;
                        ChunkController::Chunks[0]->ChunkOctree = oct;
                        //ChunkController::Chunks[0]->ChunkOctree = &oct;
                        ChunkController::Chunks[0]->ChunkOctree->AverageColor();
                        ChunkController::Chunks[0]->ChunkOctree->AverageFillState();
                        ChunkController::Chunks[0]->flag_dataUpdateNeeded = true;
                    }
                }
                ImGui::EndGroup();
            }
            ImGui::End();

            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        }



        /* Swap front and back buffers */
        glfwSwapBuffers(window);

        _dtTick += Time::DeltaTime();
        if (_dtTick > 0.5f) {
            _dtTick = 0;
            glfwSetWindowTitle(window, std::to_string(1.0f / Time::DeltaTime()).c_str());
        }



        if (Input::IsKeyPressed(GLFW_KEY_F1)) {
            Input::SetCursorMode(Input::GetCursorMode() == GLFW_CURSOR_NORMAL ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
        }
        if (Input::IsKeyPressed(GLFW_KEY_F2)) {
            AssetsController::ReloadShaders();
        }
        if (Input::IsKeyPressed(GLFW_KEY_F4)) {
            std::cout << Time::DeltaTime() << "\n";
        }
        //if (Input::IsKeyPressed(GLFW_KEY_R)) {
        //    Octree* oct = ChunkController::Chunks[0]->ChunkOctree;

        //    //oct->DivideRecour();

        //    for (int i = 0; i < 300000; i++) {
        //        oct->DivideNode();
        //        oct->RemoveLeafs();
        //    }

        //    //ChunkController::Chunks[0]->flag_dataUpdateNeeded = true;
        //}
        //if (Input::IsKeyPressed(GLFW_KEY_T)) {
        //    std::cout << "deleted: " << ChunkController::Chunks[0]->ChunkOctree->OptimizeToBottom() << "\n";
        //    ChunkController::Chunks[0]->flag_dataUpdateNeeded = true;
        //}
        glfwSwapInterval(RenderConfig.swapInterval ? 1 : 0);
        Input::key_tick();
    }
    

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwTerminate();
    return 0;
}
