#ifndef GZWIDGET_H
#define GZWIDGET_H

#include "includes.h"
#include <QMenu>
#include <QAction>

using namespace gazebo;
using namespace gazebo::rendering;
using namespace ignition::math;

class GzWidget : public QWidget
{
    Q_OBJECT
public:
    /// \brief class constructor. Applies minor changes to widget
    explicit GzWidget(QWidget *parent = 0);
    /// \brief class destructor. Cleanly calls close, terminating
    /// everything up
    ~GzWidget();
signals:
    /// \brief signal that informs that rendering system is ready
    void renderingReady();
    /// \brief signal that informs that a new frame was rendered
    void newFrameRendered();
    /// \brief signal that informs that indicators need update
    void newIndicatorUpdate();
protected:
    /// \brief overload of paintEngine() to avoid Qt painting over
    /// widget. Ogre deals with all the widget painting
    virtual QPaintEngine *paintEngine() const;
public slots:
    /// \brief function to set grid to true or false
    /// \param state - if true, grid appears. If false, grid
    /// is not displayed
    inline void setGrid(bool state) {if(scene_ready && scene && camera->Initialized()) {scene->ShowOrigin(false); scene->SetGrid(state);}
                                     showGrid = state;}
    /// \brief function to get camera's average fps
    /// return - average rendering fps of camera
    inline float getAverageFPS() {if(scene_ready && scene && camera->Initialized()) return camera->AvgFPS();
                                 else return 0.0;}
    /// \brief function to set true or false the control mode.
    /// If true (all controls enabled) it is supposed to be running inside
    /// a normal simulation, if false, in replay mode. In replay mode, special
    /// features and object-moving is diabled
    inline void setAllControlsMode(bool state) {allowAllControls = state;}
    /// \brief init function of rendering system. Makes the connection to the proper
    /// gazebo master, setting everything up. Also, instanciates a RenderEngine
    /// and creates a scene.
    void init(QString worldname);


    void resetWorld();
private slots:
    /// \brief overload of show event. If the rendering system is
    /// not up yet, creates an Ogre handle and assigns a window
    /// to the render enginge, assigning a camera to that window
    void showEvent(QShowEvent *event);
    /// \brief overload of resize event. Resizes widget, camera
    /// and Ogre window
    void resizeEvent(QResizeEvent *event);
    /// \brief function that sets up the rendering system. After
    /// connecting to gazebo master and creating a scene (init())
    /// it creates a camera and sets up its initial position
    void setupScene();
    /// \brief returns an Ogre handle to use in the Ogre window
    /// for rendering
    /// \return - ogre handle string
    std::string getOgreHandle() const;
    /// \brief overload of paintEvent, called by Ogre. Draws what the camera
    /// is viewing.
    void paintEvent(QPaintEvent *event);
    /// \brief overload of closeEvent, shutting down rendering, transport and
    /// gazebo, cleaning its way out.
    void closeEvent(QCloseEvent *event);
    /// \brief slot for mouse movement, used to move objects, pan camera, orbit camera
    void mouseMoveEvent(QMouseEvent *event);
    /// \brief slot for mouse wheel event. It aims to implement
    /// zoom efect in scene's camera.
    void wheelEvent(QWheelEvent *event); // Zoom
    /// \brief function to detect if a casted ray intersects with the ground floor at a certain level
    /// \param point - pointer to the resulting point
    /// \param ray - casted ray from viewport to scene
    /// \param ground_level - z height level in meters
    /// \return - true if intersects with ground, false, if doesn't
    bool intersectsWithGroundLevel(ignition::math::Vector3d *point, ignition::math::Vector3d ray, float ground_level);
    /// \brief slot for mouse buttons release. Detects which movement
    /// stops
    void mouseReleaseEvent(QMouseEvent *event);
    /// \brief slot for mouse buttons press.
    void mousePressEvent(QMouseEvent *event);
    /// \brief function to detect if selectable model is in mouse scope
    bool isSelectableModel();
    /// \brief highlights a model
    void highlightModel(rendering::VisualPtr visual);
    /// \brief highlights a model and moves the camera to follow it
    void highlightFollowedModel(rendering::VisualPtr visual);
    /// \brief updates indicator positions
    void updateIndicators();
    /// \brief uses gazebo's transport layer in order to rotate a model in the world
    void rotateModelInWorld(std::string model_name, float amount);
    /// \brief uses gazebo's transport layer in order to set model's pose
    void setModelPoseInWorld(std::string model_name, ignition::math::Vector3d pos, float rot);
    /// \brief uses gazebo's transport layer in order to move a model in the world in xy
    void panModelInWorld(std::string model_name, ignition::math::Vector3d motion_vector);
    /// \brief uses gazebo's transport layer in order to move a model in the world in z
    void heightModelInWorld(std::string model_name, float amount);
    /// \brief computes point in world's floor given a mouse viewport position
    bool computeMouseWorldPositionGround(ignition::math::Vector3d *point, QPoint mp);
    /// \brief opens a context menu to perform actions in the
    void launchModelContextMenu(QPoint mp, rendering::VisualPtr model);
    /// \brief follow model with the user camera
    void followModel();
    /// \brief move camera to first person view of a model
    void moveToFirstPerson();
private:
    /// \brief window identifier
    int window_id;
    /// \brief holds value of readiness of the scene
    bool scene_ready;
    /// \brief holds value of type of gzwidget
    /// if it is true, it will allow all controls to be executed (simulation mode)
    /// if it is false, it will restrict the controls (replay mode)
    bool allowAllControls;
    /// \brief pointer to the user camera
    rendering::UserCameraPtr camera;
    /// \brief pointer to rendered scene
    rendering::ScenePtr scene;
    /// \brief gazebo transport node
    transport::NodePtr node;
    /// \brief last and current points for mouse move event
    QPoint last_mp, current_mp;
    /// \brief visual to indicate mouse position in world
    rendering::VisualPtr indicator;
    /// \brief inicial indicator scale
    ignition::math::Vector3d indicator_scale;
    /// \brief variables that hold state for pan, zoom and orbit for
    /// camera control
    bool panning, orbiting, heighting;
    /// \brief variable to store point to orbit around
    ignition::math::Vector3d orbit_point;
    //model selection and highlighting
    /// \brief visual pointers for indicators and indicated models in both
    /// selection and highlighting
    rendering::VisualPtr modelHighlighter, modelFollower, highlightedModel, selectedModel;
    rendering::VisualPtr tempModel, followedModel;
    /// \brief publisher to change model's pose int the world
    transport::PublisherPtr modelPub;
    /// \brief variable that holds if a model is selected or not
    /// and if a model is being followed or not
    bool isModelSelected, isModelFollowed, isFirstPerson;
    /// \brief delta in camera position and model position in follow mode
    ignition::math::Vector3d deltaFollow;
    /// \brief camera's pose before entering first person mode
    ignition::math::Pose3d beforeFPS;
    /// \brief height at wich the camera is following the model,
    /// maximum height for the camera in fps mode
    float followHeight, fpsMaxHeight;
    /// \brief bias provided for yaw, pitch and height for the first person mode
    ignition::math::Vector3d biasFPS;
    bool showGrid;


};

#endif // GZWIDGET_H
