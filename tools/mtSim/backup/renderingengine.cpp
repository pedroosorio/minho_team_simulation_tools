#include "renderingengine.h"

// Constructor
// *******************************************************************************************
// *******************************************************************************************
RenderingEngine::RenderingEngine(QObject *parent) : QObject(parent)
{
    initializeVariables();
    bindGlut();
}

// *******************************************************************************************
// *******************************************************************************************

// Data Output Utilities
// *******************************************************************************************
// *******************************************************************************************
void RenderingEngine::GlutPrintEngine()
{
    int y = imgh - 20;
    std::string text = "Engine: OGRE";
    GlutPrintText(text, 10, y);
}

void RenderingEngine::GlutPrintFPS()
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

void RenderingEngine::_glutRun(std::vector<ignition::rendering::CameraPtr> _camera)
{
    GlutRun(_camera);
}

void RenderingEngine::GlutUpdateFPS()
{
    clock_t currTime = clock();
    double elapsedTime = double(currTime - g_prevTime) / CLOCKS_PER_SEC;
    g_fpsQueue[g_fpsIndex] = 1 / elapsedTime;
    g_fpsCount = (g_fpsCount >= 10) ? 10 : g_fpsCount + 1;
    g_fpsIndex = (g_fpsIndex + 1) % 10;
    g_prevTime = currTime;
}
// *******************************************************************************************
// *******************************************************************************************

// Configuration
void RenderingEngine::GlutInitCamera(ignition::rendering::CameraPtr _camera)
{
    g_camera = _camera;
    imgw = g_camera->GetImageWidth();
    imgh = g_camera->GetImageHeight();
    gz::Image image = g_camera->CreateImage();
    g_image = std::make_shared<gz::Image>(image);
    g_camera->Capture(*g_image);
}

void RenderingEngine::GlutInitContext()
{
    int argc = 0;
    char **argv = 0;
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE);
    glutInitWindowPosition(0, 0);
    glutInitWindowSize(imgw, imgh);
    glutCreateWindow("Gazebo MinhoTeam Viewer");
    glutDisplayFunc(_GLDisplay);
    glutIdleFunc(_GLIdle);
    glutKeyboardFunc(_GLKeyboard);
    glutReshapeFunc(_GLReshape);
}

void RenderingEngine::bindGlut()
{
    //Bind glut callbacks
    Callback<void(void)>::func = std::bind(&RenderingEngine::GlutDisplay, this);
    _GLDisplay = static_cast<_glutdisplay>(Callback<void(void)>::callback);

    Callback<void(void)>::func = std::bind(&RenderingEngine::GlutIdle, this);
    _GLIdle = static_cast<_glutidle>(Callback<void(void)>::callback);

    Callback<void(unsigned char, int, int)>::func = std::bind(&RenderingEngine::GlutKeyboard, this,
                std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    _GLKeyboard = static_cast<_glutkeyboard>(Callback<void(unsigned char, int, int)>::callback);

    Callback<void(int,int)>::func = std::bind(&RenderingEngine::GlutReshape, this,
                std::placeholders::_1,std::placeholders::_2);
    _GLReshape = static_cast<_glutreshape>(Callback<void(int,int)>::callback);
}

void RenderingEngine::initializeVariables()
{
    imgh = imgw = 0;
    g_cameraIndex = 0;
    g_initContext = false;
    g_fps = 0.0;
    g_fpsIndex = g_fpsCount = 0;
    g_cameraYawRotation = g_cameraPitchRotation = 0.0;
    g_cameraXPosition = g_cameraYPosition = 0.0;
    g_cameraZPosition = 1.0;
}

void RenderingEngine::GlutRun(std::vector<ignition::rendering::CameraPtr> _cameras)
{
    g_context = glXGetCurrentContext();
    g_display = glXGetCurrentDisplay();
    g_drawable = glXGetCurrentDrawable();
    g_cameras = _cameras;
    GlutInitCamera(_cameras[0]);
    GlutInitContext();
    g_glutDisplay = glXGetCurrentDisplay();
    g_glutDrawable = glXGetCurrentDrawable();
    g_glutContext = glXGetCurrentContext();
    glutMainLoop();
}

void RenderingEngine::GlutDisplay(void)
{
    if (g_display) glXMakeCurrent(g_display, g_drawable, g_context);

    g_cameras[g_cameraIndex]->Capture(*g_image);
    glXMakeCurrent(g_glutDisplay, g_glutDrawable, g_glutContext);

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

void RenderingEngine::GlutIdle()
{
  if (g_display) glXMakeCurrent(g_display, g_drawable, g_context);

  ignition::rendering::SceneManager* manager =
      ignition::rendering::SceneManager::Instance();
  manager->UpdateScenes();
  glXMakeCurrent(g_glutDisplay, g_glutDrawable, g_glutContext);
  glutPostRedisplay();
}

void RenderingEngine::GlutKeyboard(unsigned char _key, int _x, int _y)
{
    Q_UNUSED(_x); Q_UNUSED(_y);
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

void RenderingEngine::GlutReshape(int _width, int _height)
{
    Q_UNUSED(_width); Q_UNUSED(_height);
}
// *******************************************************************************************
// *******************************************************************************************

// Text Utilities
// *******************************************************************************************
// *******************************************************************************************
void RenderingEngine::GlutPrintText(const string &_text, int x, int y)
{
    GlutPrintTextBack(_text, x, y);
    GlutPrintTextFore(_text, x, y);
}

void RenderingEngine::GlutPrintTextBack(const string &_text, int x, int y)
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

void RenderingEngine::GlutPrintTextFore(const string &_text, int x, int y)
{
    glColor3f(0, 0.6, 0);
    GlutPrintTextImpl(_text, x, y);
}

void RenderingEngine::GlutPrintTextImpl(const string &_text, int x, int y)
{
    glutWindowPos2i(x, y);
    const char *ctext = _text.c_str();
    while (*ctext) {glutBitmapCharacter(GLUT_BITMAP_9_BY_15, *ctext++);}
}

void RenderingEngine::glWindowPos4f(float x, float y, float z, float w)
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

void RenderingEngine::glutWindowPos2i(int x, int y)
{
     glWindowPos4f(x,y,0,1);
}
// *******************************************************************************************
// *******************************************************************************************

