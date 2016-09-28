#ifndef RENDERINGCAMERA_H
#define RENDERINGCAMERA_H

///
/// \brief This code is part of the MinhoTeam Simulation Tools, developed by Pedro
/// Osório Silva (pedroosorio.eeic@gmail.com) and it's free to distribute and edit.
///
///

#include <QObject>
#include <QDebug>
#include <QTimer>
#include <QTime>
#include <QMessageBox>
#include <QGraphicsView>
#include <QGraphicsScene>

#include <stdlib.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/transport/TransportIface.hh>
#include <gazebo/common/Image.hh>

#include "ignition/rendering/Camera.hh"
#include "ignition/rendering/Image.hh"
#include "ignition/rendering/Scene.hh"
#include "ignition/rendering.hh"
#include <boost/thread.hpp>

using namespace ignition;
using namespace rendering;
using namespace gazebo::event;
using namespace std;
namespace gz = ignition::rendering;

#define ENGINE "ogre"

///
/// \brief The _cameraview struct holds the data for the camera's pose. Includes
/// [x,y,z] position and both yaw and pitch orientation values.
///
struct _cameraview
{
    QString id;
    // Rotation
    double g_cameraYawRotation;
    double g_cameraPitchRotation;
    // Position
    double g_cameraXPosition;
    double g_cameraYPosition;
    double g_cameraZPosition;
};

///
/// \brief The _rosservice struct holds data for a rosservice. DEPRECATED
///
struct _rosservice
{
    QString name;
    QString pre;
    QString post;
};

class RenderingCamera : public QObject
{
    Q_OBJECT
public:
    // Common
    ///
    /// \brief RenderingCamera implements the connection to Gazebo's server and
    /// renders the simulation view using the world camera. It uses ignition's
    /// ign-rendering to render the images. TODO:Improve FPS.
    /// \param screen Pointer to the widget where the scene
    /// \param parent
    ///
    explicit RenderingCamera(QGraphicsView *screen, QObject *parent = 0);

    ///
    /// \brief RenderingCamera::~RenderingCamera destructor of the class. Finishes the transpor layer
    /// connection.
    ///
    ~RenderingCamera();

    ///
    /// \brief startRendering starts the resources to render. For now, starts
    /// the timer that triggers an event to timely render a view of the simulation.
    ///
    void startRendering();

    ///
    /// \brief initalizeVariables initalizes member variables of the class.
    ///
    void initalizeVariables();
public slots:
    // Camera Pose Set/Get
    ///
    /// \brief setCameraPosition sets the camera's position to the defined values.
    /// \param x X world position of the camera in meters.
    /// \param y Y world position of the camera in meters.
    /// \param z Z world position of the camera in meters.
    ///
    void setCameraPosition(double x,double y,double z);

    ///
    /// \brief setCameraRotation sets the camera's orientation to the defined values.
    /// \param yaw Yaw (Z axis) angle in radians.
    /// \param pitch Yaw (X axis) angle in radians.
    ///
    void setCameraRotation(double yaw,double pitch);

    ///
    /// \brief changeCameraYaw adds delta radians to the current yaw value of the camera.
    /// \param delta value to be added in radians.
    ///
    void changeCameraYaw(double delta);

    ///
    /// \brief changeCameraPitch adds delta radians to the current pitch value of the camera.
    /// \param delta value to be added in radians.
    ///
    void changeCameraPitch(double delta);

    ///
    /// \brief changeCameraX adds delta meters to the current x position of the camera.
    /// \param delta value to be added in meters.
    ///
    void changeCameraX(double delta);

    ///
    /// \brief changeCameraY adds delta meters to the current y position of the camera.
    /// \param delta value to be added in meters.
    ///
    void changeCameraY(double delta);

    ///
    /// \brief changeCameraZ adds delta meters to the current z position of the camera.
    /// \param delta value to be added in meters.
    ///
    void changeCameraZ(double delta);

    ///
    /// \brief getCameraYaw returns camera's yaw in radians.
    /// \return Camera's yaw in radians.
    ///
    double getCameraYaw();

    ///
    /// \brief setAspectRatio sets aspect ratio to be applied to the camera.
    /// \param _aspect ratio value.
    ///
    void setAspectRatio(double _aspect);

    ///
    /// \brief getAspectRatio returns current aspect ratio of the camera.
    /// \return aspect ratio value.
    ///
    float getAspectRatio();

    ///
    /// \brief getFOV returns horizontal field of view of the camera, in radians.
    /// \return horizontal field of view of the camera in radians.
    ///
    float getFOV();

    ///
    /// \brief getHeight returns the height of the camera's image in pixels.
    /// \return height of the image in pixels.
    ///
    unsigned int getHeight();

    ///
    /// \brief getHeight returns the width of the camera's image in pixels.
    /// \return width of the image in pixels.
    ///
    unsigned int getWidth();

    ///
    /// \brief getCameraView returns the current configuration of the camera.
    /// \return Current configuration of the camera.
    ///
    _cameraview getCameraView();

    ///
    /// \brief setCameraView sets a new configuration for the camera's pose.
    /// \param _view struct that holds the new configuration's information.
    ///
    void setCameraView(_cameraview &_view);

    ///
    /// \brief updateCameraView updates the camera's current configuration.
    ///
    void updateCameraView();

    ///
    /// \brief getScene returns a pointer to the current rendering scene.
    /// \return pointer to graphics' rendeing scene.
    ///
    QGraphicsScene *getScene();

    ///
    /// \brief getFPS returns the current measures FPS value.
    /// \return measures FPS value.
    ///
    int getFPS();
    // Service calls through command line - DEPRECATED
    ///
    /// \brief rosServiceCall runs a service call using rosservice call.
    /// \param service is the service id.
    /// \param data is the data to apply to the determined service
    ///
    void rosServiceCall(_rosservice service, QString data = "");
    ///
    /// \brief stopRendering stops rendering functions to be called.
    ///
    void stopRendering();

    ///
    /// \brief startRenderin starts the procedure for the rendering functions to be called.
    ///
    void startRender();

    ///
    /// \brief isRendering verifies if the rendering procedure is on or off.
    /// \return returns true if rendering is on, false it its off.
    ///
    bool isRendering();
private slots:
    // Configuration of Gazebo Image Transport using IGNITION RENDERING
    ///
    /// \brief ConnectToGazebo connects to Gazebo transport layer and instantiates the
    /// scene manager in order to have control of the scene's rendering.
    ///
    void ConnectToGazebo();

    ///
    /// \brief CreateScene creates a scene of the simulation based on a rendering engine.
    /// \param _engine is the engine to be used in the rendering procedure. Only allowing OGRE.
    /// \return returns a scene pointer to the created scene.
    ///
    ScenePtr CreateScene(const std::string &_engine);

    ///
    /// \brief CreateCamera creates a camera to capture the scene/view of the simulation.
    /// \param _engine is the engine to be used in the rendering procedure. Only allowing OGRE.
    /// \return returns a pointer to the created camera.
    ///
    CameraPtr CreateCamera(const std::string &_engine);

    ///
    /// \brief getGazeboMasterProperties retrieves informatio (ip and port) from Gazebo's master.
    ///
    void getGazeboMasterProperties();

    // Error handling
    ///
    /// \brief throwConnectionError throws an error message and quits the program when a critical
    /// problem is encountered.
    ///
    void throwConnectionError();

    // Rendering Functions
    ///
    /// \brief renderCamera rendering function (slot) to render the most recent updated scene view.
    ///
    void renderCamera();
private:
    // Variables
    // *******************************************************************************************
    // *******************************************************************************************

    // Gazebo Master/gzserver variables
    ///
    /// \brief gzmaster_ip_ holds the IP address for the Gazebo master, generally "", "localhost"
    /// or 127.0.0.1 .
    ///
    QString gzmaster_ip_;
    ///
    /// \brief gzmaster_port_ holds the communication port to the master, to use in the Gazebo's
    /// transport layer.
    ///
    unsigned int gzmaster_port_;
    // Camera/Rendering Variables
    ///
    /// \brief _g_camera_ Pointer to the camera object from ignition rendering. This object deals
    /// with the rendering of the simulation, returning a 2D view of the scene.
    ///
    gz::CameraPtr _g_camera_;
    ///
    /// \brief _g_image_ Pointer to the image holder, whichs holds the 2D view of the scene rendered
    /// by _g_camera_.
    ///
    gz::ImagePtr _g_image_;
    ///
    /// \brief _rendering_timer_ times the rendering of the scene.
    /// TODO : Replace the rendering scheme with threaded procedures to improve FPS.
    ///
    QTimer *_rendering_timer_;
    ///
    /// \brief _render_screen_ pointer to the rendereing widget of the viewport.
    ///
    QGraphicsView *_render_screen_;
    ///
    /// \brief _scene_ pointer to the scene that renders the 2D view of the simulation, holded in
    /// _render_screen_ .
    ///
    QGraphicsScene *_scene_;
    ///
    /// \brief _manager_ manages the scene used by the rendering engine, updating the scene to
    /// be rendered.
    ///
    SceneManager* _manager_;
    ///
    /// \brief _root_ pointer to a visual node in the scene.
    ///
    VisualPtr _root_;
    // Camera Parameters
    ///
    /// \brief view_ hold the current state (position and orientation) of the camera.
    ///
    _cameraview view_;
    ///
    /// \brief last_ keeps track of the latest size of the viewport, to automatically resize the scene.
    /// \brief recent_
    ///
    QSize last_,recent_;
    // Perfromance measure
    ///
    /// \brief g_fps_ counter for rendering FPS values.
    ///
    double g_fps_;
    ///
    /// \brief timer_ timer to count the time between render updates, generating the FPS value.
    ///
    QTime timer_;
    ///
    /// \brief isOn represents the state of the simulation's rendering.
    ///
    bool isOn_;
};

#endif // RENDERINGCAMERA_H
