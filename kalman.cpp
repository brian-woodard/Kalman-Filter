
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

// initial parameters
int n_iter = 50;
float x = -0.37727; // truth value
float Q = 1e-5; // process variance
float R = 0.01; // estimate of measurement variance

// allocate space for arrays
float* z = nullptr;         // observations (normal about x, sigma=0.1) white noise
float* xhat = nullptr;      // a posteri estimate of x
float* P = nullptr;         // a posteri error estimate
float* xhatminus = nullptr; // a priori estimate of x
float* Pminus = nullptr;    // a priori error estimate
float* K = nullptr;         // gain or blending factor
float* x_axis = nullptr; 
float* x_truth = nullptr;

void create_arrays(int iterations)
{
   if (z)
      delete [] z;
   if (xhat)
      delete [] xhat;
   if (P)
      delete [] P;
   if (xhatminus)
      delete [] xhatminus;
   if (Pminus)
      delete [] Pminus;
   if (K)
      delete [] K;
   if (x_axis)
      delete [] x_axis;
   if (x_truth)
      delete [] x_truth;

   n_iter = iterations;

   z = new float[iterations];
   xhat = new float[iterations];
   P = new float[iterations];
   xhatminus = new float[iterations];
   Pminus = new float[iterations];
   K = new float[iterations];
   x_axis = new float[iterations];
   x_truth = new float[iterations];

   ////////////////////////////////////////////////////////////////////////////
   // generate random observations
   ////////////////////////////////////////////////////////////////////////////
   // 1. Initialize a random number engine
   // std::random_device provides a non-deterministic seed (true randomness if available)
   std::random_device rd{};
   // std::mt19937 is a high-quality pseudo-random number generator (Mersenne Twister)
   std::mt19937 engine{rd()};

   // 2. Define the normal distribution
   // Template the distribution for float and provide mean and stddev
   std::normal_distribution<float> dist{x, 0.1f};

   // 3. Generate and print random numbers
   for (int n = 0; n < n_iter; ++n)
   {
      // Call the distribution object with the engine to get a random number
      z[n] = dist(engine);
   }
}

void execute_filter(bool GenerateMeasurements)
{
   ////////////////////////////////////////////////////////////////////////////
   // generate random observations
   ////////////////////////////////////////////////////////////////////////////
   if (GenerateMeasurements)
   {
      // 1. Initialize a random number engine
      // std::random_device provides a non-deterministic seed (true randomness if available)
      std::random_device rd{};
      // std::mt19937 is a high-quality pseudo-random number generator (Mersenne Twister)
      std::mt19937 engine{rd()};

      // 2. Define the normal distribution
      // Template the distribution for float and provide mean and stddev
      std::normal_distribution<float> dist{x, 0.1f};

      // 3. Generate and print random numbers
      for (int n = 0; n < n_iter; ++n)
      {
         // Call the distribution object with the engine to get a random number
         z[n] = dist(engine);
      }
   }

   ////////////////////////////////////////////////////////////////////////////
   // initial guesses, these could be tunable as well, pass them in?
   ////////////////////////////////////////////////////////////////////////////
   xhat[0] = 0.0;
   P[0] = 1.0;
   Pminus[0] = P[0];
   x_truth[0] = x;
   x_axis[0] = 0.0;

   for (int k = 1; k < n_iter; ++k)
   {
      // time update
      xhatminus[k] = xhat[k-1];
      Pminus[k] = P[k-1] + Q;

      // measurement update
      K[k] = Pminus[k] / (Pminus[k] + R);
      xhat[k] = xhatminus[k] + K[k] * (z[k] - xhatminus[k]);
      P[k] = (1.0f - K[k]) * Pminus[k];
      x_axis[k] = k;
      x_truth[k] = x;
   }
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

   // run filter once for initial graphs
   create_arrays(n_iter);
   execute_filter(true);

   bool generate_measurements = false;
   int min = 50;
   int max = 900;
   int number_of_iterations = 50;

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

      ImGui::Begin("Kalman Filter");

      ImGui::Checkbox("Generate Measurements", &generate_measurements);

      if (ImGui::DragInt("Number of Iterations", &number_of_iterations, 1.0f, min, max))
      {
         // recreate arrays
         create_arrays(number_of_iterations);
         execute_filter(generate_measurements);
      }

      ImGui::InputFloat("Process Variance (Q)", &Q, 0.0f, 0.0f, "%e");
      ImGui::InputFloat("Measurement Variance (R)", &R, 0.0f, 0.0f, "%e");

      if (ImGui::Button("Execute"))
      {
         execute_filter(generate_measurements);
      }

      if (ImPlot::BeginPlot("Filter"))
      {
         ImPlot::SetupAxes("x - Iteration", "y - Voltage");
         ImPlot::PlotLine("Truth Value", x_axis, x_truth, n_iter);
         ImPlot::SetNextMarkerStyle(ImPlotMarker_Plus);
         ImPlot::PlotScatter("Noisy Measurements", x_axis, z, n_iter);
         ImPlot::PlotLine("A Posteri Estimate", x_axis, xhat, n_iter);
         ImPlot::EndPlot();
      }

      if (ImPlot::BeginPlot("Error Estimate"))
      {
         // skip first two iterations, not valid at step 0
         ImPlot::SetupAxes("x - Iteration", "y - Voltage^2");
         ImPlot::PlotLine("A Priori Error Estimate", &x_axis[2], &Pminus[2], n_iter-2);
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
