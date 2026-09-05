
#include <stdio.h>
#include <chrono>
#include <thread>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <random>
#include <iomanip>
#include <filesystem>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>

#define WIDTH  800
#define HEIGHT 1200

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
   return a;
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

class Angle
{
public:

   Angle(const char* Title, bool Enable, bool LoadedFromFile = false, double Vel = 0.0)
      : mTitle(Title), mEnabled(Enable), mLoadedFromFile(LoadedFromFile), mAngleVel(Vel) {}

   const char* mTitle;

   double mAngle = 0.0;
   double mPrevAngle = 0.0;
   double mAngleVel = 0.0;

   double mTimeConstant = 0.5;
   double mLagFilterC1 = exp(-0.016666 / mTimeConstant);
   double mLagFilterC2 = 1.0 - mLagFilterC1;
   double mAngleLag = 0.0;
   double mPrevAngleLag = 0.0;

   double mXHat = 0.0;
   double mXHatMinus = 0.0;
   double mP = 1.0;
   double mPMinus = 0.0;
   double mK = 0.0;
   double mQ = 1e-5;
   double mR = 0.01;
   double mAngleKalman = 0.0;
   double mPrevAngleKalman = 0.0;

   float mAnglePlot[SAMPLES] = {};
   float mAngleDeltaPlot[SAMPLES] = {};
   float mAngleLagPlot[SAMPLES] = {};
   float mAngleLagDeltaPlot[SAMPLES] = {};
   float mAngleKalmanPlot[SAMPLES] = {};
   float mAngleKalmanDeltaPlot[SAMPLES] = {};
   float mPlotTime[SAMPLES] = {};
   int   mRequestId[SAMPLES] = {};
   float mTimeElapsed = 0.0;

   int mOffset = 0;

   bool mEnabled;
   bool mLoadedFromFile;
   bool mAngleOverride = false;

   void DrawAngle();

};

void Angle::DrawAngle()
{
   // Use random generator for velocity
   // heading_vel = dist(engine);

   // Update filters
   if (mEnabled)
   {
      mTimeElapsed += 0.0166666;
      mAngle = wrap180(mAngle + mAngleVel);

      // Lag filter
      mAngleLag = mLagFilterC1 * mAngleLag + mLagFilterC2 * mAngle;

      // Kalman filter
      // 1. Time update
      mXHatMinus = mXHat;
      mPMinus = mP + mQ;

      // 2. Measurement update
      mK = mPMinus / (mPMinus + mR);
      mXHat = mXHatMinus + mK * (mAngleVel - mXHatMinus);
      mP = (1.0f - mK) * mPMinus;

      mAngleKalman += mXHat;
   }

   if (ImGui::CollapsingHeader(mTitle, ImGuiTreeNodeFlags_DefaultOpen))
   {
      std::string angle_title = mTitle + std::string("##angle");
      std::string angle_checkbox_title = mTitle + std::string(" Override##angle_checkbox");
      float angle = mAngle;

      ImGui::Text("%s: %f (delta %f) - offset %d", mTitle, mAnglePlot[mOffset], mAngleDeltaPlot[mOffset], mOffset);

      if (!mLoadedFromFile)
      {
         ImGui::Checkbox(angle_checkbox_title.c_str(), &mAngleOverride);
         if (mAngleOverride)
         {
            ImGui::SliderFloat(angle_title.c_str(), &angle, -180.0f, 180.0f);
            mAngle = angle;
         }
      }

      std::string plot_title = mTitle + std::string("##Plot");
      if (ImPlot::BeginPlot(plot_title.c_str()))
      {
         ImPlot::SetupAxes("x - Iteration", "y - Degrees");
         ImPlot::SetupAxisLimits(ImAxis_X1, 0.0, 60.0, ImGuiCond_Once);
         ImPlot::SetupAxisLimits(ImAxis_Y1, 0.0, 20.0, ImGuiCond_Once);
         ImPlot::PlotLine("Angle", mPlotTime, mAnglePlot, SAMPLES, ImPlotLineFlags_None, mOffset);
         ImPlot::PlotLine("Delta", mPlotTime, mAngleDeltaPlot, SAMPLES, ImPlotLineFlags_None, mOffset);
         ImPlot::PlotLine("Angle Lag", mPlotTime, mAngleLagPlot, SAMPLES, ImPlotLineFlags_None, mOffset);
         ImPlot::PlotLine("Delta Lag", mPlotTime, mAngleLagDeltaPlot, SAMPLES, ImPlotLineFlags_None, mOffset);
         ImPlot::PlotLine("Angle Kalman", mPlotTime, mAngleKalmanPlot, SAMPLES, ImPlotLineFlags_None, mOffset);
         ImPlot::PlotLine("Delta Kalman", mPlotTime, mAngleKalmanDeltaPlot, SAMPLES, ImPlotLineFlags_None, mOffset);
         ImPlot::EndPlot();
      }
   }

   if (mEnabled)
   {
      mAnglePlot[mOffset] = mAngle;
      mAngleDeltaPlot[mOffset] = mAngle - mPrevAngle;
      mAngleLagPlot[mOffset] = mAngleLag;
      mAngleLagDeltaPlot[mOffset] = mAngleLag - mPrevAngleLag;
      mAngleKalmanPlot[mOffset] = mAngleKalman;
      mAngleKalmanDeltaPlot[mOffset] = mAngleKalman - mPrevAngleKalman;
      mPlotTime[mOffset] = mTimeElapsed;

      mOffset = (mOffset + 1) % SAMPLES;

      mPrevAngle = mAngle;
      mPrevAngleLag = mAngleLag;
      mPrevAngleKalman = mAngleKalman;
   }
}

bool LoadFile(int argc, char* argv[], Angle& yaw, Angle& pitch, Angle& roll)
{
   bool result = false;

   if (argc > 1)
   {
      std::filesystem::path file_path = argv[1];

      if (file_path.extension() == ".csv")
      {
         std::ifstream file(file_path.c_str());

         if (file.is_open())
         {
            char   line[256];
            int    linecount = 0;
            double time_start;

            result = true;

            // Skip first line
            file.getline(line, sizeof(line));

            while (!file.eof())
            {
               file.getline(line, sizeof(line));
               linecount++;

               std::string item;
               std::stringstream ss(line);
               int    idx = 0;
               bool   send = false;
               double time;
               int    request_id;
               float  roll_in;
               float  pitch_in;
               float  yaw_in;

               while (std::getline(ss, item, ','))
               {
                  switch (idx)
                  {
                     case 2:
                        send = (item == "true");
                        break;
                     case 3:
                        time = std::strtod(item.c_str(), nullptr);
                        if (linecount == 1)
                           time_start = time;
                        break;
                     case 5:
                        request_id = std::strtol(item.c_str(), nullptr, 10);
                        break;
                     case 9:
                        roll_in = std::strtof(item.c_str(), nullptr);
                        break;
                     case 10:
                        pitch_in = std::strtof(item.c_str(), nullptr);
                        break;
                     case 11:
                        yaw_in = std::strtof(item.c_str(), nullptr);
                        break;
                     default:
                        break;
                  }

                  idx++;
               }

               if (!send && yaw.mOffset < SAMPLES)
               {
                  yaw.mPlotTime[yaw.mOffset] = (float)(time - time_start);
                  yaw.mAnglePlot[yaw.mOffset] = yaw_in;
                  yaw.mRequestId[yaw.mOffset] = request_id;
                  yaw.mOffset++;

                  pitch.mPlotTime[pitch.mOffset] = (float)(time - time_start);
                  pitch.mAnglePlot[pitch.mOffset] = pitch_in;
                  pitch.mRequestId[pitch.mOffset] = request_id;
                  pitch.mOffset++;

                  roll.mPlotTime[roll.mOffset] = (float)(time - time_start);
                  roll.mAnglePlot[roll.mOffset] = roll_in;
                  roll.mRequestId[roll.mOffset] = request_id;
                  roll.mOffset++;
               }
            }

            printf("Read %d lines from %s\n", linecount, file_path.string().c_str());
         }
      }
      else if (file_path.extension() == ".bin")
      {
         result = true;
         printf("Error - binary file reading not implemented yet!\n");
      }
   }

   return result;
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
   bool enabled = true;

   Angle yaw("Yaw", enabled);
   Angle pitch("Pitch", enabled);
   Angle roll("Roll", enabled);

   bool loaded_from_file = LoadFile(argc, argv, yaw, pitch, roll);

   std::random_device rd{};
   std::mt19937 engine{rd()};
   //std::normal_distribution<float> dist(heading_vel, 0.001f);

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

      ImGui::Begin("HMD Filter");

      if (loaded_from_file)
      {
         yaw.mEnabled = false;
         pitch.mEnabled = false;
         roll.mEnabled = false;
      }
      else if (ImGui::Checkbox("Enabled", &enabled))
      {
         yaw.mEnabled = enabled;
         pitch.mEnabled = enabled;
         roll.mEnabled = enabled;
      }

      if (ImGui::CollapsingHeader("Headset inputs from Vital"))
      {
         yaw.DrawAngle();
         pitch.DrawAngle();
         roll.DrawAngle();
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
