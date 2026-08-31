
#include <stdio.h>
#include <chrono>
#include <thread>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
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

float time_constant = 0.5;
float lag_filter_c1 = exp(-0.016666 / time_constant);
float lag_filter_c2 = 1.0 - lag_filter_c1;

float heading = 0.0f;
float heading_vel = 0.01f;
float prev_heading = 0.0f;
float heading_lag = 0.0f;
float prev_heading_lag = 0.0f;
float heading_kalman = 0.0f;
float prev_heading_kalman = 0.0f;

float xhat = 0.0;
float xhatminus = 0.0;
float P = 1.0;
float Pminus = 0.0;
float K = 0.0;
float Q = 1e-5;
float R = 0.01;

float heading_plot[SAMPLES];
float heading_delta_plot[SAMPLES];
float heading_lag_plot[SAMPLES];
float heading_lag_delta_plot[SAMPLES];
float heading_kalman_plot[SAMPLES];
float heading_kalman_delta_plot[SAMPLES];
float plot_time[SAMPLES];
float time_elapsed = 0.0f;

bool enabled = true;

int offset = 0;

struct Orientation
{
   double yaw;
   double pitch;
   double roll;

   Orientation()
   {
      yaw = 0.0;
      pitch = 0.0;
      roll = 0.0;
   }

   Orientation(double yaw_val, double pitch_val, double roll_val)
      : yaw(yaw_val), pitch(pitch_val), roll(roll_val) {}

   Orientation(const Orientation& other) : yaw(other.yaw), pitch(other.pitch), roll(other.roll) {}

   Orientation& operator=(const Orientation& other)
   {
      yaw = other.yaw;
      pitch = other.pitch;
      roll = other.roll;
      return *this;
   }
};

static double wrap180(double a)
{
   while(a >  180.0) a -= 360.0;
   while(a < -180.0) a += 360.0;
}

//This assumes the calibration has occured
//This will use yaw towards the middle and pitch towards the side
static Orientation calculateHMDAngles(const Orientation& igRel)
{
   const double yaw_deg = -igRel.roll;
   const double y       = yaw_deg * (3.14159265358979323846 / 180.0);
   const double cy      = std::cos(y);
   const double sy_neg  = -std::sin(y);
   const double pitch_deg = -(cy * igRel.yaw + sy_neg * igRel.pitch);

   return Orientation{ wrap180(yaw_deg), wrap180(pitch_deg), 0.0 };
}

void az_el_to_ijk(double az, double el, double& i, double& j, double& k)
{
   i = cos(az) * cos(el);
   j = sin(az) * cos(el);
   k = -sin(el);
}

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
   window = glfwCreateWindow(WIDTH, HEIGHT, "Kalman Filter", NULL, NULL);

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

   for (size_t i = 0; i < SAMPLES; i++)
   {
      heading_plot[i] = 0.0f;
      heading_delta_plot[i] = 0.0f;
      plot_time[i] = 0.0f;
   }

   std::random_device rd{};
   std::mt19937 engine{rd()};
   std::normal_distribution<float> dist(heading_vel, 0.001f);

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

      ImGui::Begin("Heading Filter");

      ImGui::Checkbox("Enabled", &enabled);

      // Use random generator for velocity
      heading_vel = dist(engine);

      if (enabled)
      {
         time_elapsed += 0.0166666;
         heading = heading + heading_vel;
         if (heading > 360.0f)
            heading -= 360.0f;

         // Lag filter
         heading_lag = lag_filter_c1 * heading_lag + lag_filter_c2 * heading;

         // Kalman filter
         // 1. Time update
         xhatminus = xhat;
         Pminus = P + Q;

         // 2. Measurement update
         K = Pminus / (Pminus + R);
         xhat = xhatminus + K * (heading_vel - xhatminus);
         P = (1.0f - K) * Pminus;

         heading_kalman += xhat;
      }

      heading_plot[offset] = heading;
      heading_delta_plot[offset] = heading - prev_heading;
      heading_lag_plot[offset] = heading_lag;
      heading_lag_delta_plot[offset] = heading_lag - prev_heading_lag;
      heading_kalman_plot[offset] = heading_kalman;
      heading_kalman_delta_plot[offset] = heading_kalman - prev_heading_kalman;
      plot_time[offset] = time_elapsed;

      ImGui::Text("Heading: %f (delta %f) - offset %d", heading_plot[offset], heading_delta_plot[offset], offset);

      if (enabled)
         offset = (offset + 1) % SAMPLES;

      prev_heading = heading;
      prev_heading_lag = heading_lag;
      prev_heading_kalman = heading_kalman;

      if (ImPlot::BeginPlot("Heading"))
      {
         ImPlot::SetupAxes("x - Iteration", "y - Degrees");
         ImPlot::SetupAxisLimits(ImAxis_X1, 0.0, 60.0, ImGuiCond_Once);
         ImPlot::SetupAxisLimits(ImAxis_Y1, 0.0, 20.0, ImGuiCond_Once);
         ImPlot::PlotLine("Heading", plot_time, heading_plot, SAMPLES, ImPlotLineFlags_None, offset);
         ImPlot::PlotLine("Delta", plot_time, heading_delta_plot, SAMPLES, ImPlotLineFlags_None, offset);
         ImPlot::PlotLine("Heading Lag", plot_time, heading_lag_plot, SAMPLES, ImPlotLineFlags_None, offset);
         ImPlot::PlotLine("Delta Lag", plot_time, heading_lag_delta_plot, SAMPLES, ImPlotLineFlags_None, offset);
         ImPlot::PlotLine("Heading Kalman", plot_time, heading_kalman_plot, SAMPLES, ImPlotLineFlags_None, offset);
         ImPlot::PlotLine("Delta Kalman", plot_time, heading_kalman_delta_plot, SAMPLES, ImPlotLineFlags_None, offset);
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
