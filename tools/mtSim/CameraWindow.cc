/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include "CameraWindow.hh"
#include <ctime>
using namespace std;
#if __APPLE__
  #include <OpenGL/gl.h>
  #include <GLUT/glut.h>
#else
  #include <GL/glew.h>
  #include <GL/gl.h>
  #include <GL/glut.h>
#endif

#include <gazebo/common/Image.hh>
#include <gazebo/common/Console.hh>

#include "ignition/rendering/Camera.hh"
#include "ignition/rendering/Image.hh"
#include "ignition/rendering/Scene.hh"
#include "ignition/rendering.hh"

#if not (__APPLE__ || _WIN32)
  #include <GL/glx.h>
#endif

#define KEY_ESC 27
#define KEY_TAB  9
// ONLY USING OGRE RENDERING ENGINE
//////////////////////////////////////////////////
unsigned int imgw = 0;
unsigned int imgh = 0;

std::vector<gz::CameraPtr> g_cameras;
gz::CameraPtr g_camera;
gz::CameraPtr g_currCamera;
unsigned int g_cameraIndex = 0;
gz::ImagePtr g_image;
bool g_initContext = false;

// FPS counter
double g_fps = 0.0;
const int g_fpsSize = 10;
double g_fpsQueue[g_fpsSize];
int g_fpsIndex = 0;
int g_fpsCount = 0;
clock_t g_prevTime;
bool g_showFPS = true;

// Camera Parameters
// Rotation
double g_cameraYawRotation = 0.0;
double g_cameraPitchRotation = 0.0;
// Position
double g_cameraXPosition = 0.0;
double g_cameraYPosition = 0.0;
double g_cameraZPosition = 1.0;

#if not (__APPLE__ || _WIN32)
  GLXContext g_context;
  Display *g_display;
  GLXDrawable g_drawable;
  GLXContext g_glutContext;
  Display *g_glutDisplay;
  GLXDrawable g_glutDrawable;
#endif

double g_offset = 0.0;

//////////////////////////////////////////////////
void GlutRun(std::vector<gz::CameraPtr> _cameras)
{
#if not (__APPLE__ || _WIN32)
  g_context = glXGetCurrentContext();
  g_display = glXGetCurrentDisplay();
  g_drawable = glXGetCurrentDrawable();
#endif

  g_cameras = _cameras;
  GlutInitCamera(_cameras[0]);
  GlutInitContext();
  //GlutPrintUsage();

#if not (__APPLE__ || _WIN32)
  g_glutDisplay = glXGetCurrentDisplay();
  g_glutDrawable = glXGetCurrentDrawable();
  g_glutContext = glXGetCurrentContext();
#endif

  glutMainLoop();
}

//////////////////////////////////////////////////
void GlutDisplay()
{
#if not (__APPLE__ || _WIN32)
  if (g_display)
  {
    glXMakeCurrent(g_display, g_drawable, g_context);
  }
#endif

  g_cameras[g_cameraIndex]->Capture(*g_image);

#if not (__APPLE__ || _WIN32)
  glXMakeCurrent(g_glutDisplay, g_glutDrawable, g_glutContext);
#endif

  unsigned char *data = g_image->GetData<unsigned char>();

  glClearColor(0.5, 0.5, 0.5, 1);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPixelZoom(1, -1);
  glRasterPos2f(-1, 1);
  glDrawPixels(imgw, imgh, GL_RGB, GL_UNSIGNED_BYTE, data);

  GlutPrintEngine();
  GlutPrintFPS();
  glutSwapBuffers();
}

//////////////////////////////////////////////////
void GlutIdle()
{
#if not (__APPLE__ || _WIN32)
  if (g_display)
  {
    glXMakeCurrent(g_display, g_drawable, g_context);
  }
#endif

  ignition::rendering::SceneManager* manager =
      ignition::rendering::SceneManager::Instance();

  manager->UpdateScenes();

#if not (__APPLE__ || _WIN32)
  glXMakeCurrent(g_glutDisplay, g_glutDrawable, g_glutContext);
#endif

  glutPostRedisplay();
}

//////////////////////////////////////////////////
void GlutKeyboard(unsigned char _key, int, int)
{
  if (_key == KEY_ESC || _key == 'q' || _key == 'Q')
  {
    exit(0);
  }
  else if (_key == KEY_TAB)
  {
    g_cameraIndex = (g_cameraIndex + 1) % g_cameras.size();
  }
  else if (_key == 'a')
  {
    g_cameraYawRotation -= 0.0174533;
  }
  else if (_key == 'd')
  {
    g_cameraYawRotation += 0.0174533;
  }
  else if (_key == 'w')
  {
    g_cameraPitchRotation -= 0.0174533;
  }
  else if (_key == 's')
  {
    g_cameraPitchRotation += 0.0174533;
  }
  else if (_key == 'i')
  {
    g_cameraXPosition += 0.1;
  }
  else if (_key == 'k')
  {
    g_cameraXPosition -= 0.1;
  }
  else if (_key == 'j')
  {
    g_cameraYPosition += 0.1;
  }
  else if (_key == 'l')
  {
    g_cameraYPosition -= 0.1;
  }
  
  g_cameras[0]->SetLocalRotation(0.0, g_cameraPitchRotation, g_cameraYawRotation);
  g_cameras[0]->SetLocalPosition(g_cameraXPosition, g_cameraYPosition, g_cameraZPosition);
}

//////////////////////////////////////////////////
void GlutReshape(int, int)
{
}

//////////////////////////////////////////////////
void GlutInitCamera(gz::CameraPtr _camera)
{
  g_camera = _camera;
  imgw = g_camera->GetImageWidth();
  imgh = g_camera->GetImageHeight();
  gz::Image image = g_camera->CreateImage();
  g_image = std::make_shared<gz::Image>(image);
  g_camera->Capture(*g_image);
}

//////////////////////////////////////////////////
void GlutInitContext()
{
  int argc = 0;
  char **argv = 0;
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE);
  glutInitWindowPosition(0, 0);
  glutInitWindowSize(imgw, imgh);
  glutCreateWindow("Gazebo MinhoTeam Viewer");
  glutDisplayFunc(GlutDisplay);
  glutIdleFunc(GlutIdle);
  glutKeyboardFunc(GlutKeyboard);
  glutReshapeFunc(GlutReshape);
}

void GlutPrintUsage()
{
  std::cout << "===============================" << std::endl;
  std::cout << "  A - Decrease Camera Yaw      " << std::endl;
  std::cout << "  D - Increase Camera Yaw      " << std::endl;
  std::cout << "  W - Decrease Camera Pitch    " << std::endl;
  std::cout << "  S - Increase Camera Pitch    " << std::endl;
  std::cout << "  ESC - Exit                   " << std::endl;
  std::cout << "===============================" << std::endl;
}

//////////////////////////////////////////////////
void GlutPrintEngine()
{
  int y = imgh - 20;
  std::string text = "Engine: OGRE";
  GlutPrintText(text, 10, y);
}

//////////////////////////////////////////////////
void GlutPrintFPS()
{
  double total = 0;
  GlutUpdateFPS();

  for (int i = 0; i < g_fpsCount; ++i)
  {
    total += g_fpsQueue[i];
  }

  int y = imgh - 40;
  double fps = total / g_fpsCount;
  std::string fpsText = "FPS: " + std::to_string(fps);
  GlutPrintText(fpsText, 10, y);
}

//////////////////////////////////////////////////
void GlutUpdateFPS()
{
  clock_t currTime = clock();
  double elapsedTime = double(currTime - g_prevTime) / CLOCKS_PER_SEC;
  g_fpsQueue[g_fpsIndex] = 1 / elapsedTime;
  g_fpsCount = (g_fpsCount >= g_fpsSize) ? g_fpsSize : g_fpsCount + 1;
  g_fpsIndex = (g_fpsIndex + 1) % g_fpsSize;
  g_prevTime = currTime;
}

//////////////////////////////////////////////////
void GlutPrintText(const std::string &_text, int x, int y)
{
  GlutPrintTextBack(_text, x, y);
  GlutPrintTextFore(_text, x, y);
}

void GlutPrintTextBack(const std::string &_text, int x, int y)
{
  glColor3f(0, 0, 0);

  for (int i = -2; i < 3; ++i)
  {
    for (int j = -2; j < 3; ++j)
    {
      GlutPrintTextImpl(_text, x + i, y + j);
    }
  }
}

void GlutPrintTextFore(const std::string &_text, int x, int y)
{
  glColor3f(0, 0.8, 0);
  GlutPrintTextImpl(_text, x, y);
}

void GlutPrintTextImpl(const std::string &_text, int x, int y)
{
  glutWindowPos2i(x, y);
  const char *ctext = _text.c_str();
  while (*ctext) {glutBitmapCharacter(GLUT_BITMAP_9_BY_15, *ctext++);}
}

void glWindowPos4f(float x,float y,float z,float w)
{
   //  Integer versions of x and y
   int ix = (int)x;
   int iy = (int)y;
   //  Save transform attributes (Matrix Mode and Viewport)
   glPushAttrib(GL_TRANSFORM_BIT|GL_VIEWPORT_BIT);
   //  Save projection matrix and set identity
   glMatrixMode(GL_PROJECTION);
   glPushMatrix();
   glLoadIdentity();
   //  Save model view matrix and set to indentity
   glMatrixMode(GL_MODELVIEW);
   glPushMatrix();
   glLoadIdentity();
   //  Set viewport to 2x2 pixels around (x,y)
   glViewport(ix-1,iy-1,2,2);
   //  Finally set the raster position
   glRasterPos4f(x-ix,y-iy,z,w);
   //  Reset model view matrix
   glPopMatrix();
   //  Reset projection matrix
   glMatrixMode(GL_PROJECTION);
   glPopMatrix();
   //  Pop transform attributes (Matrix Mode and Viewport)
   glPopAttrib();
}

void glutWindowPos2i(int x,int y) { glWindowPos4f(x,y,0,1); }
