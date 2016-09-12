#ifndef RENDERINGENGINE_H
#define RENDERINGENGINE_H

#include "ignition/rendering/RenderTypes.hh"
#include <QObject>
#include <stdio.h>
#include <ctime>
#include <functional>

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

using namespace std;
namespace gz = ignition::rendering;
#define KEY_ESC 27
#define KEY_TAB  9
template <typename T>
struct Callback;
template <typename Ret, typename... Params>
struct Callback<Ret(Params...)> {
   template <typename... Args>
   static Ret callback(Args... args) {
      func(args...);
   }
   static std::function<Ret(Params...)> func;
};

template <typename Ret, typename... Params>
std::function<Ret(Params...)> Callback<Ret(Params...)>::func;

//Glut callback typedefs
typedef void (*_glutdisplay)(void);
typedef void (*_glutidle)(void);
typedef void (*_glutkeyboard)(unsigned char _key, int _x, int _y);
typedef void (*_glutreshape)(int _width, int _height);

class RenderingEngine : public QObject
{
    Q_OBJECT
public:
    explicit RenderingEngine(QObject *parent = 0);
    // Data Output Utilities
    void GlutPrintEngine();
    void GlutPrintFPS();
    void _glutRun(std::vector<ignition::rendering::CameraPtr> _camera);
private:
    // Configuration
    void GlutUpdateFPS();
    void GlutInitCamera(gz::CameraPtr _camera);
    void GlutInitContext();
    void bindGlut();
    void initializeVariables();
    // Glut Callbacks
    void GlutDisplay(void);
    void GlutIdle();
    void GlutKeyboard(unsigned char _key, int _x, int _y);
    void GlutReshape(int _width, int _height);
    // Running Functions
    void GlutRun(std::vector<ignition::rendering::CameraPtr> _camera);
    // Text Utilities
    void GlutPrintText(const std::string &_text, int x, int y);
    void GlutPrintTextBack(const std::string &_text, int x, int y);
    void GlutPrintTextFore(const std::string &_text, int x, int y);
    void GlutPrintTextImpl(const std::string &_text, int x, int y);
    void glWindowPos4f(float x,float y,float z,float w);
    void glutWindowPos2i(int x,int y);

    // VARIABLES
    // *******************************************************************************************
    // *******************************************************************************************
    // Camera Variables
    unsigned int imgh,imgw;
    std::vector<gz::CameraPtr> g_cameras;
    gz::CameraPtr g_camera;
    gz::CameraPtr g_currCamera;
    unsigned int g_cameraIndex;
    gz::ImagePtr g_image;
    bool g_initContext;
    // OpenGL Variables
    /*GLXContext g_context;
    Display *g_display;
    GLXDrawable g_drawable;
    GLXContext g_glutContext;
    Display *g_glutDisplay;
    GLXDrawable g_glutDrawable;*/
    // FPS counter
    double g_fps;
    double g_fpsQueue[10];
    int g_fpsIndex = 0;
    int g_fpsCount = 0;
    clock_t g_prevTime;
    // Camera Parameters
    // Rotation
    double g_cameraYawRotation;
    double g_cameraPitchRotation;
    // Position
    double g_cameraXPosition;
    double g_cameraYPosition;
    double g_cameraZPosition;
    // *******************************************************************************************
    // *******************************************************************************************
    // Glut Callbacks
    _glutdisplay _GLDisplay;
    _glutkeyboard _GLKeyboard;
    _glutreshape _GLReshape;
    _glutidle _GLIdle;
};

#endif // RENDERINGENGINE_H
