#include "renderingcamera.h"

///
/// \brief RenderingCamera implements the connection to Gazebo's server and
/// renders the simulation view using the world camera. It uses ignition's
/// ign-rendering to render the images. TODO:Improve FPS.
/// \param screen Pointer to the widget where the scene
/// \param parent
///
RenderingCamera::RenderingCamera(QGraphicsView *screen, QObject *parent) : QObject(parent)
{
    initalizeVariables();
    ConnectToGazebo();
    _g_camera_ = CreateCamera(ENGINE);
    _render_screen_ = screen;
    if(_render_screen_==NULL) {exit(1);}
    _scene_ = new QGraphicsScene();
    _render_screen_->setScene(_scene_);

    updateCameraView();
    gz::Image image = _g_camera_->CreateImage();
    _g_image_ = std::make_shared<gz::Image>(image);
    _g_camera_->Capture(*_g_image_);
    timer_.start();
    isOn_ = true;
}

///
/// \brief RenderingCamera::~RenderingCamera destructor of the class. Finishes the transpor layer
/// connection.
///
RenderingCamera::~RenderingCamera()
{
    gazebo::transport::fini();
}

///
/// \brief startRendering starts the resources to render. For now, starts
/// the timer that triggers an event to timely render a view of the simulation.
///
void RenderingCamera::startRendering()
{
    _rendering_timer_->start(30);
}

///
/// \brief initalizeVariables initalizes member variables of the class.
///
void RenderingCamera::initalizeVariables()
{
    view_.g_cameraYawRotation = view_.g_cameraPitchRotation = 0.0;
    view_.g_cameraXPosition = view_.g_cameraYPosition = 0.0;
    view_.g_cameraZPosition = 1.0;
    g_fps_ = 0.0;
    _rendering_timer_ = new QTimer();
    connect(_rendering_timer_,SIGNAL(timeout()),this,SLOT(renderCamera()));
}

///
/// \brief setCameraPosition sets the camera's position to the defined values.
/// \param x X world position of the camera in meters.
/// \param y Y world position of the camera in meters.
/// \param z Z world position of the camera in meters.
///
void RenderingCamera::setCameraPosition(double x, double y, double z)
{
    view_.g_cameraXPosition = x;
    view_.g_cameraYPosition = y;
    view_.g_cameraZPosition = z;
    updateCameraView();
}

///
/// \brief setCameraRotation sets the camera's orientation to the defined values.
/// \param yaw Yaw (Z axis) angle in radians.
/// \param pitch Yaw (X axis) angle in radians.
///
void RenderingCamera::setCameraRotation(double yaw, double pitch)
{
    view_.g_cameraYawRotation = yaw;
    view_.g_cameraPitchRotation = pitch;
    updateCameraView();
}

///
/// \brief changeCameraYaw adds delta radians to the current yaw value of the camera.
/// \param delta value to be added in radians.
///
void RenderingCamera::changeCameraYaw(double delta)
{
    view_.g_cameraYawRotation += delta;
    updateCameraView();
}

///
/// \brief changeCameraPitch adds delta radians to the current pitch value of the camera.
/// \param delta value to be added in radians.
///
void RenderingCamera::changeCameraPitch(double delta)
{
    view_.g_cameraPitchRotation += delta;
    updateCameraView();
}

///
/// \brief changeCameraX adds delta meters to the current x position of the camera.
/// \param delta value to be added in meters.
///
void RenderingCamera::changeCameraX(double delta)
{
    view_.g_cameraXPosition += delta;
    updateCameraView();
}


///
/// \brief changeCameraY adds delta meters to the current y position of the camera.
/// \param delta value to be added in meters.
///
void RenderingCamera::changeCameraY(double delta)
{
    view_.g_cameraYPosition += delta;
    updateCameraView();
}

///
/// \brief changeCameraZ adds delta meters to the current z position of the camera.
/// \param delta value to be added in meters.
///
void RenderingCamera::changeCameraZ(double delta)
{
    view_.g_cameraZPosition += delta;
    updateCameraView();
}

///
/// \brief getCameraYaw returns camera's yaw in radians.
/// \return Camera's yaw in radians.
///
double RenderingCamera::getCameraYaw()
{
    return view_.g_cameraYawRotation;
}

///
/// \brief setAspectRatio sets aspect ratio to be applied to the camera.
/// \param _aspect ratio value.
///
void RenderingCamera::setAspectRatio(double _aspect)
{
    _g_camera_->SetAspectRatio(_aspect);
}

///
/// \brief getAspectRatio returns current aspect ratio of the camera.
/// \return aspect ratio value.
///
float RenderingCamera::getAspectRatio()
{
    return _g_camera_->GetAspectRatio();
}

///
/// \brief getFOV returns horizontal field of view of the camera, in radians.
/// \return horizontal field of view of the camera in radians.
///
float RenderingCamera::getFOV()
{
    return _g_camera_->GetHFOV().Radian();
}

///
/// \brief getHeight returns the height of the camera's image in pixels.
/// \return height of the image in pixels.
///
unsigned int RenderingCamera::getHeight()
{
    return _g_camera_->GetImageHeight();
}

///
/// \brief getHeight returns the width of the camera's image in pixels.
/// \return width of the image in pixels.
///
unsigned int RenderingCamera::getWidth()
{
    return _g_camera_->GetImageWidth();
}

///
/// \brief getCameraView returns the current configuration of the camera.
/// \return Current configuration of the camera.
///
_cameraview RenderingCamera::getCameraView()
{
    return view_;
}

///
/// \brief setCameraView sets a new configuration for the camera's pose.
/// \param _view struct that holds the new configuration's information.
///
void RenderingCamera::setCameraView(_cameraview &_view)
{
    view_ = _view;
    updateCameraView();
}

///
/// \brief updateCameraView updates the camera's current configuration.
///
void RenderingCamera::updateCameraView()
{
    _g_camera_->SetLocalPosition(view_.g_cameraXPosition, view_.g_cameraYPosition, view_.g_cameraZPosition);
    _g_camera_->SetLocalRotation(0.0, view_.g_cameraPitchRotation, view_.g_cameraYawRotation);
}

///
/// \brief getScene returns a pointer to the current rendering scene.
/// \return pointer to graphics' rendeing scene.
///
QGraphicsScene *RenderingCamera::getScene()
{
    return _scene_;
}

///
/// \brief getFPS returns the current measures FPS value.
/// \return measures FPS value.
///
int RenderingCamera::getFPS()
{
    return (int)g_fps_;

}

///
/// \brief rosServiceCall runs a service call using rosservice call.
/// \param service is the service id.
/// \param data is the data to apply to the determined service
///
void RenderingCamera::rosServiceCall(_rosservice service, QString data)
{
    QString command = QString("rosservice call ")+service.pre+data+service.post+QString(" > /dev/null");
    system(command.toStdString().c_str());
}

///
/// \brief ConnectToGazebo connects to Gazebo transport layer and instantiates the
/// scene manager in order to have control of the scene's rendering.
///
void RenderingCamera::ConnectToGazebo()
{
    gazebo::common::Console::SetQuiet(false);
    getGazeboMasterProperties();
    if(!gazebo::transport::init(gzmaster_ip_.toStdString(),gzmaster_port_,2)) throwConnectionError();
    gazebo::transport::run();
    _manager_ = SceneManager::Instance();
    _manager_->Load();
    _manager_->Init();
}

///
/// \brief CreateScene creates a scene of the simulation based on a rendering engine.
/// \param _engine is the engine to be used in the rendering procedure. Only allowing OGRE.
/// \return returns a scene pointer to the created scene.
///
ScenePtr RenderingCamera::CreateScene(const std::string &_engine)
{
    RenderEngine *engine = rendering::get_engine(_engine);
    ScenePtr scene = engine->CreateScene("scene");
    SceneManager::Instance()->AddScene(scene);
    return scene;
}

///
/// \brief CreateCamera creates a camera to capture the scene/view of the simulation.
/// \param _engine is the engine to be used in the rendering procedure. Only allowing OGRE.
/// \return returns a pointer to the created camera.
///
CameraPtr RenderingCamera::CreateCamera(const std::string &_engine)
{
    ScenePtr scene = CreateScene(_engine);
    _root_ = scene->GetRootVisual();
    CameraPtr camera = scene->CreateCamera("camera");
    // Parse info from camera_conf.xml
    camera->SetLocalPosition(view_.g_cameraXPosition, view_.g_cameraYPosition, view_.g_cameraZPosition);
    camera->SetLocalRotation(0.0, view_.g_cameraPitchRotation, view_.g_cameraYawRotation);
    camera->SetImageWidth(280);
    camera->SetImageHeight(280);
    camera->SetAntiAliasing(5);
    camera->SetAspectRatio(1.0);
    camera->SetHFOV((float)(M_PI/2.0));
    _root_->AddChild(camera);
    return camera;
}

///
/// \brief getGazeboMasterProperties retrieves informatio (ip and port) from Gazebo's master.
///
void RenderingCamera::getGazeboMasterProperties()
{
    // Defaults, get from env variables later
    std::string _str = "";
    gazebo::transport::get_master_uri(_str,gzmaster_port_);
    gzmaster_ip_.fromStdString(_str);
}

///
/// \brief throwConnectionError throws an error message and quits the program when a critical
/// problem is encountered.
///
void RenderingCamera::throwConnectionError()
{
    QMessageBox asd;
    asd.setWindowTitle("Failed to connect to Gazebo Master !");
    QString text = QString("Please open a terminal and write ")+
    QString("$rosrun gazebo_ros gzserver before running MinhoTeam Simulation Tools.");
    asd.setInformativeText(text);
    asd.setStandardButtons(QMessageBox::Ok);
    asd.setIconPixmap(QPixmap("://resources/images/critical.png"));
    asd.exec();
    exit(0);
}


///
/// \brief renderCamera rendering function (slot) to render the most recent updated scene view.
///
void RenderingCamera::renderCamera()
{
    _manager_->UpdateScenes();
    _g_camera_->Capture(*_g_image_); // Search for improvements for render procedure to boost fps
    float h_scale_factor = 1.0, w_scale_factor = 1.0;
    if(_g_image_){
        g_fps_ = 1000.0/(double)timer_.elapsed();
        timer_.start();
        _scene_->clear();
        recent_ = _render_screen_->size();
        _scene_->addPixmap(QPixmap::fromImage(QImage(_g_image_->GetData<unsigned char>()
                                                     ,_g_image_->GetWidth()
                                                     ,_g_image_->GetHeight(),
        _g_image_->GetWidth()*_g_image_->GetDepth()*sizeof(unsigned char),QImage::Format_RGB888)).scaled(
                               QSize(recent_.width(),recent_.height()),
                               Qt::IgnoreAspectRatio,
                               Qt::SmoothTransformation));
    }
}

///
/// \brief stopRendering stops rendering functions to be called.
///
void RenderingCamera::stopRendering()
{
    _rendering_timer_->stop();
}

///
/// \brief startRenderin starts the procedure for the rendering functions to be called.
///
void RenderingCamera::startRender()
{
    _rendering_timer_->start(30);
}

///
/// \brief isRendering verifies if the rendering procedure is on or off.
/// \return returns true if rendering is on, false it its off.
///
bool RenderingCamera::isRendering()
{
    return _rendering_timer_->isActive();
}


