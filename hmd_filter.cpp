
#include <stdio.h>
#include <chrono>
#include <thread>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <random>
#include <iomanip>
#include <algorithm>
#include <filesystem>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>

#define WIDTH  1600
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

float calc_az[SAMPLES];
float calc_el[SAMPLES];
float calc_az_new[SAMPLES];
float calc_el_new[SAMPLES];
float calc_i[SAMPLES];
float calc_j[SAMPLES];
float calc_k[SAMPLES];
float file_az[SAMPLES];
float file_el[SAMPLES];
float file_az_delta[SAMPLES];
float file_el_delta[SAMPLES];
float file_time[SAMPLES];
float file_time_delta[SAMPLES];
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

struct PlaybackState
{
   int  sample = 0;
   bool playing = false;
   bool loop = true;

   double accumulator = 0.0;
   std::chrono::steady_clock::time_point prevTime = std::chrono::steady_clock::now();
};

struct ViewportCamera
{
   float yaw      = 35.0f;
   float pitch    = 20.0f;
   float distance = 5.0f;
};

struct BinaryData
{
   uint64_t frame;
   double   time;
   int32_t  words[14];
};

union LfUnion
{
   long  l;
   float f;
};

float doBSCALE(int width, int scale)
{
   LfUnion lf;

   lf.l = (127 + (width) - (scale)) << 23;

   return lf.f;
}

double HALF_CIRCLE_DEGREES = 180.0;
double revsToDegs32(int32_t value)
{
   return static_cast<double>(value * HALF_CIRCLE_DEGREES / doBSCALE(31, 0));
}

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

static Orientation calculateHMDAngles_new(const Orientation& igRel)
{
   // Head yaw appears to correspond to negative IG-relative roll.
   const double yaw_deg = -igRel.roll;
   const double y = glm::radians(yaw_deg);

   const double c = std::cos(y);
   const double s = std::sin(y);

   // Rotate the remaining two axes into the head-relative frame.
   const double pitch_deg = -(c * igRel.yaw - s * igRel.pitch);

   // Orthogonal component of the same rotation.
   // Sign may need to be reversed after visually checking the headset.
   const double roll_deg = s * igRel.yaw + c * igRel.pitch;

   return Orientation
   {
      wrap180(yaw_deg),
      wrap180(pitch_deg),
      wrap180(roll_deg)
   };
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
   float mPlotTimeDelta[SAMPLES] = {};
   float mRequestId[SAMPLES] = {};
   float mRequestIdDelta[SAMPLES] = {};
   float mTimeElapsed = 0.0;

   int mOffset = 0;

   bool mEnabled;
   bool mLoadedFromFile;
   bool mAngleOverride = false;

   void DrawAngle(const PlaybackState& Playback);

};

void Angle::DrawAngle(const PlaybackState& Playback)
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
         ImPlot::PlotLine("Angle", mPlotTime, mAnglePlot, SAMPLES, ImPlotLineFlags_None, mOffset);
         ImPlot::PlotLine("Delta", mPlotTime, mAngleDeltaPlot, SAMPLES, ImPlotLineFlags_None, mOffset);
         ImPlot::PlotLine("Angle Lag", mPlotTime, mAngleLagPlot, SAMPLES, ImPlotLineFlags_None, mOffset);
         ImPlot::PlotLine("Delta Lag", mPlotTime, mAngleLagDeltaPlot, SAMPLES, ImPlotLineFlags_None, mOffset);
         ImPlot::PlotLine("Angle Kalman", mPlotTime, mAngleKalmanPlot, SAMPLES, ImPlotLineFlags_None, mOffset);
         ImPlot::PlotLine("Delta Kalman", mPlotTime, mAngleKalmanDeltaPlot, SAMPLES, ImPlotLineFlags_None, mOffset);
         ImPlot::PlotLine("Request Id", mPlotTime, mRequestIdDelta, SAMPLES, ImPlotLineFlags_None, mOffset);
         ImPlot::PlotLine("Time Delta", mPlotTime, mPlotTimeDelta, SAMPLES, ImPlotLineFlags_None, mOffset);

         if (Playback.playing)
         {
            float playback_time[1];
            float playback_angle[1];
            playback_time[0] = mPlotTime[Playback.sample];
            playback_angle[0] = mAnglePlot[Playback.sample];
            ImPlot::PlotScatter("Playback", playback_time, playback_angle, 1);
         }
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
               int    prev_request_id;
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
                        if (linecount > 1)
                           prev_request_id = request_id;
                        request_id = std::strtol(item.c_str(), nullptr, 10);
                        if (linecount == 1)
                           prev_request_id = request_id;
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
                  yaw.mRequestIdDelta[yaw.mOffset] = request_id - prev_request_id;

                  if (yaw.mOffset == 0)
                     yaw.mPlotTimeDelta[yaw.mOffset] = 0.0f;
                  else
                     yaw.mPlotTimeDelta[yaw.mOffset] = yaw.mPlotTime[yaw.mOffset] - yaw.mPlotTime[yaw.mOffset - 1];
                  yaw.mOffset++;

                  pitch.mPlotTime[pitch.mOffset] = (float)(time - time_start);
                  pitch.mAnglePlot[pitch.mOffset] = pitch_in;
                  pitch.mRequestId[pitch.mOffset] = request_id;
                  pitch.mRequestIdDelta[pitch.mOffset] = request_id - prev_request_id;

                  if (pitch.mOffset == 0)
                     pitch.mPlotTimeDelta[pitch.mOffset] = 0.0f;
                  else
                     pitch.mPlotTimeDelta[pitch.mOffset] = pitch.mPlotTime[pitch.mOffset] - pitch.mPlotTime[pitch.mOffset - 1];
                  pitch.mOffset++;

                  roll.mPlotTime[roll.mOffset] = (float)(time - time_start);
                  roll.mAnglePlot[roll.mOffset] = roll_in;
                  roll.mRequestId[roll.mOffset] = request_id;
                  roll.mRequestIdDelta[roll.mOffset] = request_id - prev_request_id;

                  if (roll.mOffset == 0)
                     roll.mPlotTimeDelta[roll.mOffset] = 0.0f;
                  else
                     roll.mPlotTimeDelta[roll.mOffset] = roll.mPlotTime[roll.mOffset] - roll.mPlotTime[roll.mOffset - 1];
                  roll.mOffset++;
               }
               else if (send && offset < SAMPLES)
               {
                  file_az[offset] = yaw_in;
                  file_el[offset] = pitch_in;
                  file_time[offset] = (float)(time - time_start);

                  if (offset == 0)
                  {
                     file_az_delta[offset] = 0.0f;
                     file_el_delta[offset] = 0.0f;
                     file_time_delta[offset] = 0.0f;
                  }
                  else
                  {
                     file_az_delta[offset] = file_az[offset] - file_az[offset - 1];
                     file_el_delta[offset] = file_el[offset] - file_el[offset - 1];
                     file_time_delta[offset] = file_time[offset] - file_time[offset - 1];
                  }
                  offset++;
               }
            }

            printf("Read %d lines from %s\n", linecount, file_path.string().c_str());
         }
      }
      else if (file_path.extension() == ".bin")
      {
         result = true;
         std::ifstream file(file_path.c_str(), std::ios::binary);

         if (file.is_open())
         {
            int recordcount = 0;

            result = true;

            BinaryData record;
            file.read((char*)&record, sizeof(record));
            double time_start = record.time;
            while (!file.eof())
            {
               recordcount++;

               double time = record.time;
               float  roll_in = revsToDegs32(record.words[5]);
               float  pitch_in = revsToDegs32(record.words[6]);
               float  yaw_in = revsToDegs32(record.words[7]);

               if (yaw.mOffset < SAMPLES)
               {
                  yaw.mPlotTime[yaw.mOffset] = (float)(time - time_start);
                  yaw.mAnglePlot[yaw.mOffset] = yaw_in;
                  yaw.mOffset++;

                  pitch.mPlotTime[pitch.mOffset] = (float)(time - time_start);
                  pitch.mAnglePlot[pitch.mOffset] = pitch_in;
                  pitch.mOffset++;

                  roll.mPlotTime[roll.mOffset] = (float)(time - time_start);
                  roll.mAnglePlot[roll.mOffset] = roll_in;
                  roll.mOffset++;
               }

               file.read((char*)&record, sizeof(record));
            }

            printf("Read %d records from %s\n", recordcount, file_path.string().c_str());
         }
      }
   }

   for (int i = 0; i < yaw.mOffset; i++)
   {
      Orientation ig_angles = Orientation(yaw.mAnglePlot[i], pitch.mAnglePlot[i], roll.mAnglePlot[i]);
      Orientation avcsim_angles = calculateHMDAngles(ig_angles);
      calc_az[i] = avcsim_angles.yaw;
      calc_el[i] = avcsim_angles.pitch;
      avcsim_angles = calculateHMDAngles_new(ig_angles);
      calc_az_new[i] = avcsim_angles.yaw;
      calc_el_new[i] = avcsim_angles.pitch;

      double ii, j, k;
      double az_radians = glm::radians(avcsim_angles.yaw);
      double el_radians = -glm::radians(avcsim_angles.pitch);
      az_el_to_ijk(az_radians, el_radians, ii, j, k);
      calc_i[i] = -j;
      calc_j[i] = k;
      calc_k[i] = -ii;
   }

   return result;
}

static ImVec2 ProjectPoint(
   const glm::vec3& p,
   const glm::mat4& mvp,
   const ImVec2& origin,
   const ImVec2& size)
{
   glm::vec4 clip = mvp * glm::vec4(p, 1.0f);

   if (std::abs(clip.w) < 0.00001f)
      return origin;

   glm::vec3 ndc = glm::vec3(clip) / clip.w;

   return ImVec2(
      origin.x + (ndc.x * 0.5f + 0.5f) * size.x,
      origin.y + (1.0f - (ndc.y * 0.5f + 0.5f)) * size.y);
}


static void DrawLine3D(
   ImDrawList* dl,
   const glm::vec3& a,
   const glm::vec3& b,
   const glm::mat4& mvp,
   const ImVec2& origin,
   const ImVec2& size,
   ImU32 color,
   float thickness = 2.0f)
{
   ImVec2 p0 = ProjectPoint(a, mvp, origin, size);
   ImVec2 p1 = ProjectPoint(b, mvp, origin, size);

   dl->AddLine(p0, p1, color, thickness);
}

static void DrawHMDViewport(
   const char* title,
   const Orientation& hmd,
   ViewportCamera& camera,
   const ImVec2& requestedSize = ImVec2(600, 450), bool convert = false)
{
   ImVec2 origin = ImGui::GetCursorScreenPos();

   ImVec2 size = requestedSize;

   float availWidth = ImGui::GetContentRegionAvail().x;
   if (availWidth > 100.0f)
      size.x = availWidth;

   //----------------------------------------------------------
   // Invisible viewport input surface
   //----------------------------------------------------------

   std::string button_title = "##HMD3DViewport" + std::string(title);
   ImGui::InvisibleButton(
      button_title.c_str(),
      size,
      ImGuiButtonFlags_MouseButtonLeft);

   bool hovered = ImGui::IsItemHovered();

   if (hovered)
   {
      ImGuiIO& io = ImGui::GetIO();

      //
      // Left mouse drag = orbit camera
      //
      if (ImGui::IsMouseDragging(ImGuiMouseButton_Left))
      {
         camera.yaw   -= io.MouseDelta.x * 0.4f;
         camera.pitch += io.MouseDelta.y * 0.4f;

         camera.pitch = glm::clamp(camera.pitch, -89.0f, 89.0f);
      }

      //
      // Mouse wheel = zoom
      //
      if (io.MouseWheel != 0.0f)
      {
         camera.distance -= io.MouseWheel * 0.4f;

         camera.distance = glm::clamp(camera.distance, 2.0f, 15.0f);
      }
   }

   ImDrawList* dl = ImGui::GetWindowDrawList();

   ImVec2 bottomRight(
      origin.x + size.x,
      origin.y + size.y);

   dl->PushClipRect(
      origin,
      bottomRight,
      true);

   dl->AddRectFilled(
      origin,
      bottomRight,
      ImGui::GetColorU32(ImGuiCol_FrameBg));

   dl->AddRect(
      origin,
      bottomRight,
      ImGui::GetColorU32(ImGuiCol_Border));

   //----------------------------------------------------------
   // Orbit camera
   //----------------------------------------------------------

   float cameraYaw = glm::radians(camera.yaw);

   float cameraPitch = glm::radians(camera.pitch);

   glm::vec3 cameraPos;

   cameraPos.x =
      camera.distance *
      cos(cameraPitch) *
      sin(cameraYaw);

   cameraPos.y =
      camera.distance *
      sin(cameraPitch);

   cameraPos.z =
      camera.distance *
      cos(cameraPitch) *
      cos(cameraYaw);

   glm::mat4 view = glm::lookAt(
      cameraPos,
      glm::vec3(0.0f),
      glm::vec3(0, 1, 0));

   float aspect = size.x / size.y;

   glm::mat4 projection =
      glm::perspective(
         glm::radians(40.0f),
         aspect,
         0.1f,
         100.0f);

   glm::mat4 worldMVP = projection * view;

   //----------------------------------------------------------
   // World coordinate axes
   //----------------------------------------------------------

   const float axisLength = 2.0f;

   ImU32 xColor = IM_COL32(255, 80, 80, 255);
   ImU32 yColor = IM_COL32(80, 255, 80, 255);
   ImU32 zColor = IM_COL32(80, 140, 255, 255);

   DrawLine3D(
      dl,
      glm::vec3(0),
      glm::vec3(axisLength, 0, 0),
      worldMVP,
      origin,
      size,
      xColor,
      3.0f);

   DrawLine3D(
      dl,
      glm::vec3(0),
      glm::vec3(0, axisLength, 0),
      worldMVP,
      origin,
      size,
      yColor,
      3.0f);

   DrawLine3D(
      dl,
      glm::vec3(0),
      glm::vec3(0, 0, axisLength),
      worldMVP,
      origin,
      size,
      zColor,
      3.0f);

   //
   // Negative axes -- dimmer
   //
   DrawLine3D(
      dl,
      glm::vec3(0),
      glm::vec3(-axisLength, 0, 0),
      worldMVP,
      origin,
      size,
      IM_COL32(120, 50, 50, 255));

   DrawLine3D(
      dl,
      glm::vec3(0),
      glm::vec3(0, -axisLength, 0),
      worldMVP,
      origin,
      size,
      IM_COL32(50, 120, 50, 255));

   DrawLine3D(
      dl,
      glm::vec3(0),
      glm::vec3(0, 0, -axisLength),
      worldMVP,
      origin,
      size,
      IM_COL32(50, 70, 120, 255));

   //----------------------------------------------------------
   // Axis labels
   //----------------------------------------------------------

   auto drawAxisLabel =
      [&](const char* text,
         glm::vec3 position,
         ImU32 color)
   {
      ImVec2 p =
         ProjectPoint(
               position,
               worldMVP,
               origin,
               size);

      dl->AddText(
         ImVec2(p.x + 4, p.y + 4),
         color,
         text);
   };

   drawAxisLabel(
      "+X",
      glm::vec3(axisLength, 0, 0),
      xColor);

   drawAxisLabel(
      "+Y",
      glm::vec3(0, axisLength, 0),
      yColor);

   drawAxisLabel(
      "+Z",
      glm::vec3(0, 0, axisLength),
      zColor);

   //----------------------------------------------------------
   // Headset model matrix
   //----------------------------------------------------------

   glm::mat4 model(1.0f);

   //
   // Coordinate system:
   //
   // +X = right
   // +Y = up
   // -Z = forward
   //

   Orientation hmd_update = hmd;
   if (convert)
   {
      hmd_update = calculateHMDAngles_new(hmd_update);
      hmd_update.yaw = -hmd_update.yaw;
   }

   model = glm::rotate(
      model,
      glm::radians((float)hmd_update.yaw),
      glm::vec3(0, 1, 0));

   model = glm::rotate(
      model,
      glm::radians((float)hmd_update.pitch),
      glm::vec3(1, 0, 0));

   model = glm::rotate(
      model,
      glm::radians((float)hmd_update.roll),
      glm::vec3(0, 0, 1));

   //
   // One MVP including the headset orientation.
   //
   glm::mat4 hmdMVP = projection * view * model;

   //----------------------------------------------------------
   // Headset geometry
   //----------------------------------------------------------

   const float x = 1.0f;
   const float y = 0.45f;
   const float z = 0.40f;

   glm::vec3 vertices[8] =
   {
      {-x, -y, -z},
      { x, -y, -z},
      { x,  y, -z},
      {-x,  y, -z},

      {-x, -y,  z},
      { x, -y,  z},
      { x,  y,  z},
      {-x,  y,  z}
   };

   static const int edges[][2] =
   {
      {0,1}, {1,2}, {2,3}, {3,0},
      {4,5}, {5,6}, {6,7}, {7,4},
      {0,4}, {1,5}, {2,6}, {3,7}
   };

   ImU32 hmdColor = ImGui::GetColorU32(ImGuiCol_Text);
   ImU32 hmdYellowColor = IM_COL32(255, 255, 0, 255);

   int count = 0;
   for (const auto& edge : edges)
   {
      ImU32 color = hmdColor;
      if (count < 4) color = hmdYellowColor;
      DrawLine3D(
         dl,
         vertices[edge[0]],
         vertices[edge[1]],
         hmdMVP,
         origin,
         size,
         color,
         2.0f);
      count++;
   }

   // Draw light yellow semi-transparent quad that represents the front of hmd
   ImVec2 quad2D[4];

   for (int i = 0; i < 4; ++i)
   {
      quad2D[i] = ProjectPoint(
         vertices[edges[i][0]],
         hmdMVP,
         origin,
         size);
   }

   // Light yellow, semi-transparent.
   ImU32 quadFill = IM_COL32(255, 245, 150, 70);

   dl->AddQuadFilled(
      quad2D[0],
      quad2D[1],
      quad2D[2],
      quad2D[3],
      quadFill);

   //----------------------------------------------------------
   // Headset-local axes
   //
   // These rotate WITH the headset.
   //----------------------------------------------------------

   const float localAxis = 1.35f;

   DrawLine3D(
      dl,
      glm::vec3(0),
      glm::vec3(localAxis, 0, 0),
      hmdMVP,
      origin,
      size,
      xColor,
      2.0f);

   DrawLine3D(
      dl,
      glm::vec3(0),
      glm::vec3(0, localAxis, 0),
      hmdMVP,
      origin,
      size,
      yColor,
      2.0f);

   DrawLine3D(
      dl,
      glm::vec3(0),
      glm::vec3(0, 0, localAxis),
      hmdMVP,
      origin,
      size,
      zColor,
      2.0f);

   //----------------------------------------------------------
   // Headset forward vector
   //
   // Forward = local -Z
   //----------------------------------------------------------

   glm::vec3 forward(0, 0, -2.0f);

   ImU32 forwardColor = IM_COL32(255, 220, 50, 255);

   if (!convert)
   {
      DrawLine3D(
         dl,
         glm::vec3(0),
         forward,
         hmdMVP,
         origin,
         size,
         forwardColor,
         4.0f);
   }

   ImVec2 forwardEnd =
      ProjectPoint(
         forward,
         hmdMVP,
         origin,
         size);

   dl->AddCircleFilled(
      forwardEnd,
      5.0f,
      forwardColor);

   if (convert)
   {
      double i, j, k;

      double az_radians = glm::radians(hmd_update.yaw);
      double el_radians = -glm::radians(hmd_update.pitch);
      az_el_to_ijk(az_radians, el_radians, i, j, k);
      glm::vec3 direction_vector(-j, k, -i);

      ImU32 direction_color = IM_COL32(255, 0, 0, 255);

      DrawLine3D(
         dl,
         glm::vec3(0),
         direction_vector * 2.0f,
         worldMVP,
         origin,
         size,
         direction_color,
         4.0f);

      //----------------------------------------------------------
      // Headset model matrix
      //----------------------------------------------------------

      glm::mat4 model(1.0f);

      //
      // Coordinate system:
      //
      // +X = right
      // +Y = up
      // -Z = forward
      //

      Orientation hmd_update_2 = hmd;
      if (convert)
      {
         hmd_update_2 = calculateHMDAngles(hmd_update_2);
         hmd_update_2.yaw = -hmd_update_2.yaw;
      }

      model = glm::rotate(
         model,
         glm::radians((float)hmd_update_2.yaw),
         glm::vec3(0, 1, 0));

      model = glm::rotate(
         model,
         glm::radians((float)hmd_update_2.pitch),
         glm::vec3(1, 0, 0));

      model = glm::rotate(
         model,
         glm::radians((float)hmd_update_2.roll),
         glm::vec3(0, 0, 1));

      //
      // One MVP including the headset orientation.
      //
      glm::mat4 hmdMVP = projection * view * model;

      //----------------------------------------------------------
      // Headset geometry
      //----------------------------------------------------------

      const float x = 1.0f;
      const float y = 0.45f;
      const float z = 0.40f;

      glm::vec3 vertices[8] =
      {
         {-x, -y, -z},
         { x, -y, -z},
         { x,  y, -z},
         {-x,  y, -z},

         {-x, -y,  z},
         { x, -y,  z},
         { x,  y,  z},
         {-x,  y,  z}
      };

      static const int edges[][2] =
      {
         {0,1}, {1,2}, {2,3}, {3,0},
         {4,5}, {5,6}, {6,7}, {7,4},
         {0,4}, {1,5}, {2,6}, {3,7}
      };

      ImU32 hmdColor = ImGui::GetColorU32(ImGuiCol_Text);
      ImU32 hmdYellowColor = IM_COL32(255, 255, 0, 255);

      int count = 0;
      for (const auto& edge : edges)
      {
         ImU32 color = hmdColor;
         if (count < 4) color = hmdYellowColor;
         DrawLine3D(
            dl,
            vertices[edge[0]],
            vertices[edge[1]],
            hmdMVP,
            origin,
            size,
            color,
            2.0f);
         count++;
      }

      // Draw light yellow semi-transparent quad that represents the front of hmd
      ImVec2 quad2D[4];

      for (int i = 0; i < 4; ++i)
      {
         quad2D[i] = ProjectPoint(
            vertices[edges[i][0]],
            hmdMVP,
            origin,
            size);
      }

      // Light yellow, semi-transparent.
      ImU32 quadFill = IM_COL32(255, 245, 150, 70);

      dl->AddQuadFilled(
         quad2D[0],
         quad2D[1],
         quad2D[2],
         quad2D[3],
         quadFill);

      //----------------------------------------------------------
      // Headset-local axes
      //
      // These rotate WITH the headset.
      //----------------------------------------------------------

      const float localAxis = 1.35f;

      DrawLine3D(
         dl,
         glm::vec3(0),
         glm::vec3(localAxis, 0, 0),
         hmdMVP,
         origin,
         size,
         xColor,
         2.0f);

      DrawLine3D(
         dl,
         glm::vec3(0),
         glm::vec3(0, localAxis, 0),
         hmdMVP,
         origin,
         size,
         yColor,
         2.0f);

      DrawLine3D(
         dl,
         glm::vec3(0),
         glm::vec3(0, 0, localAxis),
         hmdMVP,
         origin,
         size,
         zColor,
         2.0f);
   }

   //----------------------------------------------------------
   // Text overlay
   //----------------------------------------------------------

   char text[256];

   snprintf(
      text,
      sizeof(text),
      "Yaw: %7.2f  Pitch: %7.2f  Roll: %7.2f",
      hmd_update.yaw,
      hmd_update.pitch,
      hmd_update.roll);

   dl->AddText(
      ImVec2(
         origin.x + 10,
         origin.y + 10),
      ImGui::GetColorU32(ImGuiCol_Text),
      title);

   dl->AddText(
      ImVec2(
         origin.x + 10,
         origin.y + 30),
      ImGui::GetColorU32(ImGuiCol_Text),
      text);

   dl->AddText(
      ImVec2(
         origin.x + 10,
         origin.y + 50),
      ImGui::GetColorU32(ImGuiCol_TextDisabled),
      "Left drag: orbit camera   Mouse wheel: zoom");

   dl->PopClipRect();
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

   PlaybackState playback;
   ViewportCamera viewportCamera;

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

      yaw.DrawAngle(playback);
      pitch.DrawAngle(playback);
      roll.DrawAngle(playback);

      if (offset > 0)
      {
         if (ImPlot::BeginPlot("AVCSIM Angles##Plot"))
         {
            ImPlot::SetupAxes("x - Iteration", "y - Degrees");
            ImPlot::PlotLine("Az", file_time, file_az, SAMPLES, ImPlotLineFlags_None, offset);
            ImPlot::PlotLine("El", file_time, file_el, SAMPLES, ImPlotLineFlags_None, offset);
            ImPlot::PlotLine("Az Delta", file_time, file_az_delta, SAMPLES, ImPlotLineFlags_None, offset);
            ImPlot::PlotLine("El Delta", file_time, file_el_delta, SAMPLES, ImPlotLineFlags_None, offset);
            ImPlot::PlotLine("Time Delta", file_time, file_time_delta, SAMPLES, ImPlotLineFlags_None, offset);

            ImPlot::EndPlot();
         }
      }

      if (ImPlot::BeginPlot("AVCSIM Angles Calculated##Plot"))
      {
         ImPlot::SetupAxes("x - Iteration", "y - Degrees");
         ImPlot::PlotLine("Calculated Az", yaw.mPlotTime, calc_az, SAMPLES, ImPlotLineFlags_None, yaw.mOffset);
         ImPlot::PlotLine("Calculated El", yaw.mPlotTime, calc_el, SAMPLES, ImPlotLineFlags_None, yaw.mOffset);
         ImPlot::PlotLine("Calculated Az New", yaw.mPlotTime, calc_az, SAMPLES, ImPlotLineFlags_None, yaw.mOffset);
         ImPlot::PlotLine("Calculated El New", yaw.mPlotTime, calc_el, SAMPLES, ImPlotLineFlags_None, yaw.mOffset);

         if (playback.playing)
         {
            float playback_time[1];
            float playback_angle[1];
            playback_time[0] = yaw.mPlotTime[playback.sample];
            playback_angle[0] = calc_az[playback.sample];
            ImPlot::PlotScatter("Playback", playback_time, playback_angle, 1);
         }
         ImPlot::EndPlot();
      }

      if (ImPlot::BeginPlot("AVCSIM Direction Vector Calculated##Plot"))
      {
         ImPlot::SetupAxes("x - Iteration", "y - Degrees");
         ImPlot::PlotLine("Calculated i", yaw.mPlotTime, calc_i, SAMPLES, ImPlotLineFlags_None, yaw.mOffset);
         ImPlot::PlotLine("Calculated j", yaw.mPlotTime, calc_j, SAMPLES, ImPlotLineFlags_None, yaw.mOffset);
         ImPlot::PlotLine("Calculated k", yaw.mPlotTime, calc_k, SAMPLES, ImPlotLineFlags_None, yaw.mOffset);

         if (playback.playing)
         {
            float playback_time[1];
            float playback_angle[1];
            playback_time[0] = yaw.mPlotTime[playback.sample];
            playback_angle[0] = calc_i[playback.sample];
            ImPlot::PlotScatter("Playback", playback_time, playback_angle, 1);
         }
         ImPlot::EndPlot();
      }

      ImGui::End();

      ImGui::Begin("HMD Viewports");

      int sampleCount = std::min({
         yaw.mOffset,
         pitch.mOffset,
         roll.mOffset
      });

      //------------------------------------------------------
      // Playback timing
      //------------------------------------------------------

      auto now = std::chrono::steady_clock::now();

      double dt = std::chrono::duration<double>(now - playback.prevTime).count();

      playback.prevTime = now;

      if (playback.playing)
      {
         playback.accumulator += dt;

         constexpr double samplePeriod = 1.0 / 60.0;

         while (playback.accumulator >= samplePeriod)
         {
            playback.accumulator -= samplePeriod;

            playback.sample++;

            if (playback.sample >= sampleCount)
            {
               if (playback.loop)
               {
                  playback.sample = 0;
               }
               else
               {
                  playback.sample = sampleCount - 1;
                  playback.playing = false;
                  break;
               }
            }
         }
      }

      //------------------------------------------------------
      // Playback controls
      //------------------------------------------------------

      ImGui::SeparatorText("Recorded HMD Orientation");

      if (!playback.playing)
      {
         if (ImGui::Button("Play"))
         {
            playback.playing = true;
            playback.prevTime = std::chrono::steady_clock::now();
         }
      }
      else
      {
         if (ImGui::Button("Pause"))
         {
            playback.playing = false;
         }
      }

      ImGui::SameLine();

      if (ImGui::Button("Restart"))
      {
         playback.sample = 0;
         playback.accumulator = 0.0;
      }

      ImGui::SameLine();

      ImGui::Checkbox("Loop", &playback.loop);

      //------------------------------------------------------
      // Sample slider
      //------------------------------------------------------

      int previousSample = playback.sample;

      ImGui::SliderInt("Sample", &playback.sample, 0, sampleCount - 1);

      //
      // If manually scrubbing, reset playback timing so it
      // doesn't immediately advance after releasing slider.
      //
      if (previousSample != playback.sample)
      {
         playback.accumulator = 0.0;
         playback.prevTime = std::chrono::steady_clock::now();
      }

      //------------------------------------------------------
      // Current raw sample
      //------------------------------------------------------

      Orientation igRelative
      {
         yaw.mAnglePlot[playback.sample],
         pitch.mAnglePlot[playback.sample],
         roll.mAnglePlot[playback.sample]
      };

      Orientation head = calculateHMDAngles_new(igRelative);
      head.yaw = -head.yaw;

      //------------------------------------------------------
      // Information
      //------------------------------------------------------

      ImGui::Text("Sample: %d / %d", playback.sample, sampleCount - 1);

      ImGui::SameLine();

      ImGui::Text("Time: %.3f sec", yaw.mPlotTime[playback.sample]);

      ImGui::Text(
         "Raw   Yaw: %8.3f   Pitch: %8.3f   Roll: %8.3f",
         igRelative.yaw,
         igRelative.pitch,
         igRelative.roll);

      ImGui::Text(
         "Head  Yaw: %8.3f   Pitch: %8.3f   Roll: %8.3f",
         head.yaw,
         head.pitch,
         head.roll);

      DrawHMDViewport(
         "Raw",
         igRelative,
         viewportCamera,
         ImVec2(600, 450));

      DrawHMDViewport(
         "Converted",
         igRelative,
         viewportCamera,
         ImVec2(600, 450), true);

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
