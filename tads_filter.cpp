
#include <stdio.h>
#include <chrono>
#include <thread>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <random>
#include <iomanip>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>

#define WIDTH  800
#define HEIGHT 800

// NOTE: Uncomment the following line for GL error handling
//#define GL_DEBUG

#ifdef GL_DEBUG
#define GLCALL(function) \
   { \
      GLenum error = GL_INVALID_ENUM; \
      while (error != GL_NO_ERROR) \
      { \
         error = glGetError(); \
      } \
      function; \
      error = glGetError(); \
      if (error != GL_NO_ERROR) \
      { \
         fprintf(stderr, "OpenGL Error: GL_ENUM(%d) at %s:%d\n", error, __FILE__, __LINE__); \
      } \
   }
#else
#define GLCALL(function) function;
#endif

#define SAMPLES 6000

int main(int argc, char* argv[])
{
   GLFWwindow* window = nullptr;

   // initialize glfw
   if (!glfwInit())
      return 0;

   glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
   glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
   glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

   // Create window
   window = glfwCreateWindow(WIDTH, HEIGHT, "TADS Filter", NULL, NULL);

   if (!window)
   {
      glfwTerminate();
      return 0;
   }

   // make the window's context current
   glfwMakeContextCurrent(window);

   // use glad to load OpenGL function pointers
   if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
   {
      printf("Error: Failed to initialize GLAD.\n");
      glfwTerminate();
      return 0;
   }

   glfwSetWindowSize(window, WIDTH, HEIGHT);

   // Setup Dear ImGui
   IMGUI_CHECKVERSION();
   ImGui::CreateContext();
   ImPlot::CreateContext();
   ImGui::StyleColorsDark();

   // Setup Platform/Render backends
   ImGui_ImplGlfw_InitForOpenGL(window, true);
   ImGui_ImplOpenGL3_Init("#version 330");

   // Make the window visible
   glfwShowWindow(window);

   // Initialize opengl
   GLCALL(glClearColor(0.5, 0.5, 0.5, 1.0));

   // enable blending
   GLCALL(glEnable(GL_BLEND));
   GLCALL(glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));

   // set frame rate to 60 Hz
   using framerate = std::chrono::duration<double, std::ratio<1, 60>>;
   auto frame_time = std::chrono::high_resolution_clock::now() + framerate{1};

   std::vector<std::string> titles;
   std::vector<std::vector<float>> data;

   std::ifstream tads_csv("tads_filter.csv");
   if (tads_csv.is_open())
   {
      std::string line;
      bool        first_line = true;
      int         line_count = 0;

      while (!tads_csv.eof())
      {
         std::getline(tads_csv, line);

         if (line.empty())
            break;

         line_count++;

         std::string item;
         std::stringstream ss(line);
         int idx = 0;
         while (std::getline(ss, item, ','))
         {
            if (first_line)
            {
               std::vector<float> d;

               titles.push_back(item);
               data.push_back(d);
            }
            else
            {
               data[idx].push_back(std::strtof(item.c_str(), nullptr));
               idx++;
            }
         }

         first_line = false;
      }
   }

   std::vector<float> tads_yaw_rate_bias;
   float yaw_rate_bias = 0.0;
   for (int i = 0; i < data[1].size(); i++)
   {
      yaw_rate_bias = yaw_rate_bias + 0.000625 * (data[1][i] - data[3][i]);
      tads_yaw_rate_bias.push_back(yaw_rate_bias);
   }

   std::vector<std::string> tse_titles;
   std::vector<std::vector<float>> tse_data;

   std::ifstream tse_csv("tse_1.csv");
   if (tse_csv.is_open())
   {
      std::string line;
      bool        first_line = true;
      int         line_count = 0;

      while (!tse_csv.eof())
      {
         std::getline(tse_csv, line);

         if (line.empty())
            break;

         line_count++;

         std::string item;
         std::stringstream ss(line);
         int idx = 0;
         while (std::getline(ss, item, ','))
         {
            if (first_line)
            {
               std::vector<float> d;

               tse_titles.push_back(item);
               tse_data.push_back(d);
            }
            else
            {
               tse_data[idx].push_back(std::strtof(item.c_str(), nullptr));
               idx++;
            }
         }

         first_line = false;
      }
   }

   while (window)
   {
      // Poll events
      glfwPollEvents();

      if (glfwWindowShouldClose(window))
      {
         glfwTerminate();
         window = nullptr;
         break;
      }

      GLCALL(glClear(GL_COLOR_BUFFER_BIT));

      // Start the Dear ImGui frame
      ImGui_ImplOpenGL3_NewFrame();
      ImGui_ImplGlfw_NewFrame();
      ImGui::NewFrame();

      ImGui::Begin("TADS Filter");

      if (ImPlot::BeginPlot("TADS CSV File"))
      {
         ImPlot::SetupAxes("x - Iteration", "y - Data");

         for (int i = 1; i < titles.size(); i++)
         {
            ImPlot::PlotLine(titles[i].c_str(), data[0].data(), data[i].data(), data[i].size());
         }
         ImPlot::EndPlot();
      }

      if (ImPlot::BeginPlot("TSE CSV File"))
      {
         ImPlot::SetupAxes("x - Iteration", "y - Data");

         for (int i = 1; i < tse_titles.size(); i++)
         {
            ImPlot::PlotLine(tse_titles[i].c_str(), tse_data[0].data(), tse_data[i].data(), tse_data[i].size());
         }
         ImPlot::EndPlot();
      }

      if (ImPlot::BeginPlot("TAD Yaw Rate Bias"))
      {
         ImPlot::SetupAxes("x - Iteration", "y - Data");
         ImPlot::PlotLine("tads_yaw_rate_bias", data[0].data(), tads_yaw_rate_bias.data(), data[0].size());
         ImPlot::EndPlot();
      }

      ImGui::End();

      // Render ImGui
      ImGui::Render();
      ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

      glfwSwapBuffers(window);

      // wait until next frame
      std::this_thread::sleep_until(frame_time);
      frame_time += framerate{1};
   }

   // Cleanup
   ImGui_ImplOpenGL3_Shutdown();
   ImGui_ImplGlfw_Shutdown();
   ImPlot::DestroyContext();
   ImGui::DestroyContext();

   glfwDestroyWindow(window);
   glfwTerminate();

   return 0;
}
