#include "gzwidget.h"
#define mag_indicator 15.0
#define mag_zoom 1.0
#define mag_pan 0.025
#define mag_orbit 0.01
#define max_selection_size 3.0
#define highSel_freq 30
#define max_fps_camdelta 80

GzWidget::GzWidget(QWidget *parent) : QWidget(parent)
{
    window_id = -1;
    scene_ready = false;
    panning = orbiting = heighting = isModelSelected = isModelFollowed = isFirstPerson = false;
    this->setAttribute(Qt::WA_OpaquePaintEvent, true);
    this->setAttribute(Qt::WA_PaintOnScreen, true);
    this->setMouseTracking(true);
    showGrid = false;
}

void GzWidget::init(QString worldname)
{
    // Connect transport layer to running gazebo master
    gazebo::common::Console::SetQuiet(false);
    std::string _str = "";
    unsigned int gzmaster_port = 0;
    std::string gzmaster_ip = "";

    if(worldname.compare("mtbasestation")==0) {
        gzmaster_ip = "localhost";
        gzmaster_port = 11346;
    }else {
        gazebo::transport::get_master_uri(_str,gzmaster_port);
        gzmaster_ip = _str;
    }


    // While connection is not made, keep trying
    int tries = 0;
    while(!gazebo::transport::init(gzmaster_ip,gzmaster_port,5) && tries<3){
        perror("Failed to init gazebo transport layer.\n");
        tries++;
    }

    std::cout << "GAZEBO_MASTER_URI:" << gzmaster_ip << " " << gzmaster_port << std::endl;

    gazebo::transport::run();
    // Init new transport layer node
    node = transport::NodePtr(new transport::Node());
    node->Init();
    if(worldname.compare("mtbasestation")==0) modelPub = node->Advertise<msgs::Model>("~/bsplugin/modify");
    else modelPub = node->Advertise<msgs::Model>("~/replayplugin/modify");
    // Init render engine
    rendering::RenderEngine::Instance()->Load();
    rendering::RenderEngine::Instance()->Init();

    // Create the scene. This must be done in the constructor so that
    // we can then create a user camera.
    scene = rendering::RenderEngine::Instance()->CreateScene(worldname.toStdString().c_str(),false);
    if(!scene) perror("Failed to create scene.\n");
    else {
        setupScene();
        scene_ready = true;
        this->showEvent(NULL);
    }
}

void GzWidget::resetWorld()
{
    msgs::Model msg;
    msg.set_name("reset");
    msg.set_id(3); // if id = 3, reset world
    modelPub->Publish(msg);
}

GzWidget::~GzWidget()
{
    this->close();
}

QPaintEngine *GzWidget::paintEngine() const
{
    return nullptr;
}

void GzWidget::setupScene()
{
    // The user camera name.
    std::string cameraBaseName = "msimcam";
    std::string cameraName = cameraBaseName;
    // Connect to gazebo master
    transport::ConnectionPtr connection = transport::connectToMaster();
    if(connection){
        std::string topicData;
        msgs::Packet packet;
        msgs::Request request;
        msgs::GzString_V topics;
        // Request base topics
        request.set_id(0);
        request.set_request("get_topics");
        connection->EnqueueMsg(msgs::Package("request", request), true);
        connection->Read(topicData);

        packet.ParseFromString(topicData);
        topics.ParseFromString(packet.serialized_data());

        std::string searchable;
        for (int i = 0; i < topics.data_size(); ++i)
        searchable += topics.data(i);
        // Settle camera name in order to be unique
        int i = 0;
        while (searchable.find(cameraName) != std::string::npos)
            cameraName = cameraBaseName + boost::lexical_cast<std::string>(++i);

    } else perror("Unable to connect to a running Gazebo master.\n");

    // If there is already a user camera, use it, otherwise create a new one
    if(scene->UserCameraCount() == 0) {
        camera = scene->CreateUserCamera(cameraName,false);
    } else camera = scene->GetUserCamera(0);

    // Set initial position for the camera
    ignition::math::Vector3d camPos(12, -10, 8);
    ignition::math::Vector3d lookAt(0, 0, 0);
    auto delta = lookAt - camPos;
    double yaw = atan2(delta.Y(), delta.X());
    double pitch = atan2(-delta.Z(),
    sqrt(delta.X()*delta.X() + delta.Y()*delta.Y()));
    scene->ShowOrigin(false);
    camera->SetDefaultPose(ignition::math::Pose3d(camPos,
            ignition::math::Quaterniond(0, pitch, yaw)));

    // Create Ogre visual: indicator for mouse usage
    indicator.reset(new Visual("msim_indicator", camera->GetScene()));
    indicator->Load();
    indicator->AttachMesh("unit_sphere");
    indicator->SetScale(math::Vector3(0.1, 0.1, 0.001));
    indicator_scale = Vector3d(0.1, 0.1, 0.001);
    indicator->SetCastShadows(false);
    indicator->SetMaterial("Gazebo/YellowTransparent");
    indicator->SetVisible(false);
    indicator->SetVisibilityFlags(GZ_VISIBILITY_GUI);

    // Create Ogre visual: indicator for model highlighting
    modelHighlighter.reset(new Visual("msim_highlighter", camera->GetScene()));
    modelHighlighter->Load();
    modelHighlighter->AttachMesh("unit_sphere");
    modelHighlighter->SetScale(math::Vector3(1.0, 1.0, 0.001));
    modelHighlighter->SetCastShadows(false);
    modelHighlighter->SetMaterial("Gazebo/RedTransparent");
    modelHighlighter->SetVisible(false);
    modelHighlighter->SetVisibilityFlags(GZ_VISIBILITY_GUI);

    modelFollower.reset(new Visual("msim_follower", camera->GetScene()));
    modelFollower->Load();
    modelFollower->AttachMesh("unit_sphere");
    modelFollower->SetScale(math::Vector3(1.0, 1.0, 0.001));
    modelFollower->SetCastShadows(false);
    modelFollower->SetMaterial("Gazebo/YellowTransparent");
    modelFollower->SetVisible(false);
    modelFollower->SetVisibilityFlags(GZ_VISIBILITY_GUI);
}

void GzWidget::showEvent(QShowEvent *event)
{
    // These two functions are most applicable for Linux.

    if(window_id <=0 && scene_ready){
        // Get the window handle in a form that OGRE can use.
        std::string winHandle = getOgreHandle();
        // Create the OGRE render window
        window_id = rendering::RenderEngine::Instance()->GetWindowManager()->
        CreateWindow(winHandle, this->width(), this->height());
        // Attach the user camera to the window
        rendering::RenderEngine::Instance()->GetWindowManager()->SetCamera(
        window_id, camera);
        // Emit rendering ready signal
        emit renderingReady();
        connect(this,SIGNAL(newIndicatorUpdate()),this,SLOT(updateIndicators()));
    }

    // Let QT continue processing the show event.
    if(event) QWidget::showEvent(event);
    // Grab focus.
    this->setFocus();
}

void GzWidget::resizeEvent(QResizeEvent *event)
{
    // Resize the camera and Ogre window with the resizing of the widget
    if(window_id >= 0){
        rendering::RenderEngine::Instance()->GetWindowManager()->Resize(
        window_id, event->size().width(), event->size().height());
        if(camera) camera->Resize(event->size().width(), event->size().height());
    }
}

std::__cxx11::string GzWidget::getOgreHandle() const
{
    // Get Ogre handler
    std::string ogreHandle;
    ogreHandle = "0:0:"+std::to_string(static_cast<uint64_t>(this->winId()));
    return ogreHandle;
}

void GzWidget::paintEvent(QPaintEvent *event)
{
    static int i=0;
    // If camera is ready, render a frame
    if(camera && camera->Initialized()){
        event::Events::preRender();
        event::Events::render();
        event::Events::postRender();
        // emit new frame rendered signal
        emit newFrameRendered();
        if(i>highSel_freq){
            i=0;
            emit newIndicatorUpdate();
            if(scene_ready && scene && camera->Initialized()) {scene->ShowOrigin(false); scene->SetGrid(showGrid);}
            /*Pose3d camPose = camera->WorldPose();
            printf("%.2f %.2f %.2f | %.2f %.2f %.2f\n",camPose.Pos().X(),camPose.Pos().Y(), camPose.Pos().Z(),
                   camPose.Rot().Roll(), camPose.Rot().Pitch(), camPose.Rot().Yaw());*/
        }i++;
    } else event::Events::preRender();
    this->update();
    event->accept();
}

void GzWidget::closeEvent(QCloseEvent *event)
{
    // Close everything up
    if(node) node->Fini();
    node.reset();
    if (camera) camera->Fini();
    camera.reset();
    scene.reset();
    RenderEngine::Instance()->Fini();
    gazebo::transport::fini();
    event->accept();
}

void GzWidget::mouseMoveEvent(QMouseEvent *event)
{
    static int i=0;
    if (!scene || !camera) return;
    current_mp = event->pos();
    QPoint delta_mp = current_mp-last_mp;

    if(isFirstPerson){
        if(!followedModel || !camera) return;
        if((event->buttons()&Qt::LeftButton)){
            if(delta_mp.x()>max_fps_camdelta) delta_mp.setX(max_fps_camdelta);
            if(delta_mp.x()<-max_fps_camdelta) delta_mp.setX(-max_fps_camdelta);
            if(delta_mp.y()>max_fps_camdelta) delta_mp.setY(max_fps_camdelta);
            if(delta_mp.y()<-max_fps_camdelta) delta_mp.setY(-max_fps_camdelta);
            Vector2d delta_orbit = Vector2d(delta_mp.x()*mag_orbit*-0.01,
                                            delta_mp.y()*mag_orbit*0.01);
            biasFPS.X() += delta_orbit.X();
            if(biasFPS.X()>M_PI) biasFPS.X() = M_PI;
            while(biasFPS.X()<-M_PI) biasFPS.X() = -M_PI;
            biasFPS.Y() += delta_orbit.Y();
            if(biasFPS.Y()>(M_PI/4)) biasFPS.Y() = M_PI/4;
            if(biasFPS.Y()<-(M_PI/4)) biasFPS.Y() = -M_PI/4;
            Vector3d camPos = camera->WorldPosition();
            if(i>3) {
                Pose3d pose;
                pose.Set(camPos,Vector3d(0,biasFPS.Y(),biasFPS.X())+
                                      followedModel->GetWorldPose().rot.GetAsEuler().Ign());
                camera->SetWorldPose(pose);
                i=0;
            } else i++;
            return;
        }
    }else if(isModelSelected){
        if((event->buttons()&Qt::LeftButton)){ // move model along xy plane
            Vector3d pos_a, pos_b;
            if(computeMouseWorldPositionGround(&pos_a,last_mp)&&computeMouseWorldPositionGround(&pos_b,current_mp)){
                panModelInWorld(highlightedModel->GetName(),pos_b-pos_a);
            }
        } else if((event->buttons()&Qt::MiddleButton)){ // move model along z Axis
            heightModelInWorld(highlightedModel->GetName(),-delta_mp.y()*mag_pan);
        }
    } else {
        if((event->buttons()&Qt::LeftButton)&&panning){// Pan on xy plane, keeping z
            if(abs(delta_mp.x())>50 || abs(delta_mp.x())>50){
                last_mp = current_mp;
                return;
            } else {
                float camYaw = camera->WorldRotation().Yaw();
                Vector3d camPos = camera->WorldPosition();
                camPos.Y() += (delta_mp.x()*cos(camYaw)+delta_mp.y()*sin(camYaw))*mag_pan;
                camPos.X() += (-delta_mp.x()*sin(camYaw)+delta_mp.y()*cos(camYaw))*mag_pan;
                camera->SetWorldPosition(camPos);
            }
        }else if ((event->buttons()&Qt::MiddleButton)&&orbiting){// Orbit around a point
            if(abs(delta_mp.x())>50 || abs(delta_mp.x())>50){
                last_mp = current_mp;
                return;
            } else {
                Vector3d camPos = camera->WorldPosition();
                // if x and y are much different, keep only the bigger one
                if(abs(delta_mp.x())>(abs(delta_mp.y()))) delta_mp.setY(0);
                else if(abs(delta_mp.y())>(abs(delta_mp.x()))) delta_mp.setX(0);
                Vector2d delta_orbit = Vector2d(delta_mp.x()*mag_orbit*-0.4,
                                                delta_mp.y()*mag_orbit*0.4);

                Vector3d rotatedCamPos = orbit_point;
                rotatedCamPos.X() += cos(delta_orbit.X())*(camPos.X()-orbit_point.X())-
                                    sin(delta_orbit.X())*(camPos.Y()-orbit_point.Y());
                rotatedCamPos.Y() += sin(delta_orbit.X())*(camPos.X()-orbit_point.X())+
                                    cos(delta_orbit.X())*(camPos.Y()-orbit_point.Y());
                rotatedCamPos.Z() += sin(delta_orbit.Y())*(camPos.X()-orbit_point.X())+
                                    cos(delta_orbit.Y())*(camPos.Z()-orbit_point.Z());
                camera->SetWorldPosition(rotatedCamPos);
                camera->SetWorldRotation(Quaterniond(0, camera->WorldRotation().Pitch()+delta_orbit.Y()
                                                     , camera->WorldRotation().Yaw()+delta_orbit.X()));
            }
        } else if((event->buttons()&Qt::RightButton)&&heighting){//Simple Height variation
            if(abs(delta_mp.x())>50 || abs(delta_mp.x())>50){
                last_mp = current_mp;
                return;
            } else {
                camera->SetWorldPosition(camera->WorldPosition()+Vector3d(0.0,0.0,delta_mp.y()*mag_pan));
            }
        }else {if(indicator)indicator->SetVisible(false);}
    }


    last_mp = current_mp;
}

void GzWidget::wheelEvent(QWheelEvent *event)
{
    if (!scene || !camera) return;
    current_mp = event->pos();
    // -------------------------------------------------------------------
    if(isFirstPerson){ // height change in first person mode
        if(!followedModel || !camera) return;
        Vector3d modelPos = followedModel->GetWorldPose().pos.Ign();
        Vector3d camPos = camera->WorldPosition();
        if(event->delta()>0) biasFPS.Z()+=0.05*fpsMaxHeight;
        else biasFPS.Z()-=0.05*fpsMaxHeight;
        camPos.Z() = modelPos.Z()+biasFPS.Z();
        if(camPos.Z()<0.1) { camPos.Z() = 0.1; biasFPS.Z()+=0.05*fpsMaxHeight; }
        if(camPos.Z()>fpsMaxHeight) { camPos.Z() = fpsMaxHeight; biasFPS.Z()-=0.05*fpsMaxHeight; }
        camera->SetWorldPosition(camPos);
        return;
    }
    // MODEL UNDER THE MOUSE
    // -------------------------------------------------------------------
    // if a model is highlighter, rotate it
    if(modelHighlighter->GetVisible() && allowAllControls){
        if(highlightedModel){
            rotateModelInWorld(highlightedModel->GetName(),event->delta()*0.1*(M_PI/180.0));
            return;
        }
    }
    // -------------------------------------------------------------------
    // NO MODEL UNDER THE MOUSE
    // If intersects with ground plane, make it zoom !
    if(isModelFollowed){
        if(!followedModel || !camera) return;
        Vector3d camPos = camera->WorldPosition(), follow = followedModel->GetWorldPose().pos.Ign();
        double zoomFactor = 0.1;
        double d = camPos.Distance(follow);
        if(fabs(d)<1) return;
        if(fabs(d)<3.0) zoomFactor/=5.0;
        if(event->delta()>0) zoomFactor *= -1.0;
        deltaFollow += deltaFollow*zoomFactor;
        followHeight += followHeight*zoomFactor;
    }else {
        Vector3d ray, origin, point;
        origin = Vector3d(0.0,0.0,0.0);
        camera->CameraToViewportRay(current_mp.x(),current_mp.y(),origin,ray);
        double zoomFactor = mag_zoom;
        indicator->SetVisible(false);

        if(intersectsWithGroundLevel(&point,ray,0.0)){
            Vector3d camPos = camera->WorldPosition();
            double d = camPos.Distance(point);
            if(fabs(d)<3.0) zoomFactor/=5.0;
            if(event->delta()<0) zoomFactor *= -1.0;
            camPos.X() += zoomFactor*ray.X();
            camPos.Y() += zoomFactor*ray.Y();
            camPos.Z() += zoomFactor*ray.Z();
            camera->SetWorldPosition(camPos);
            if(d>0.0 && d<50.0) indicator->SetScale(indicator_scale*((mag_indicator+d)/mag_indicator));
            indicator->SetWorldPosition(point);
            indicator->SetVisible(true);
        } else return;
    }
}

bool GzWidget::intersectsWithGroundLevel(Vector3d *point, Vector3d ray, float ground_level)
{
    Vector3d c0 = camera->WorldPosition();
    Vector3d p0 = Vector3d(0.0,0.0,ground_level);
    Vector3d n = Vector3d(0.0,0.0,1.0);
    if(ray.Dot(n)==0){
        return false;
    } else {
        Vector3d temp = p0-c0;
        float d = temp.Dot(n)/ray.Dot(n);
        (*point)= d*ray+c0;
        return true;
    }
}

void GzWidget::mouseReleaseEvent(QMouseEvent *event)
{
    // End all panning/orbiting events
    // and hide the mouse indicator
    panning = orbiting = false;
    if(isModelSelected && allowAllControls) { if(modelHighlighter) modelHighlighter->SetMaterial("Gazebo/RedTransparent");
        isModelSelected = false;
        emit modelReleased(QString::fromStdString(highlightedModel->GetName()));
    }
    indicator->SetVisible(false);
}

void GzWidget::mousePressEvent(QMouseEvent *event)
{
    if (!scene || !camera) return;
    current_mp = event->pos();

   // -------------------------------------------------------------------
    if(isFirstPerson){
      if(event->buttons()&Qt::RightButton){//out of first person mode
         isFirstPerson = false;
         if(!followedModel || !camera) return;
         followedModel->SetTransparency(0.0);
         camera->SetWorldPose(beforeFPS);
      }
      return;
    }
    // MODEL UNDER THE MOUSE
    // -------------------------------------------------------------------
    if(modelHighlighter->GetVisible()&&(event->buttons()&Qt::LeftButton|event->buttons()&Qt::MiddleButton)&&allowAllControls){
        isModelSelected = true;
        modelHighlighter->SetMaterial("Gazebo/BlueTransparent");
        isModelFollowed = false;
        modelFollower->SetVisible(false);
        emit modelClicked(QString::fromStdString(highlightedModel->GetName()));
        return;
    }
    if(modelHighlighter->GetVisible()&&(event->buttons()&Qt::RightButton)){
        // Launch model's context menu
        launchModelContextMenu(current_mp,highlightedModel);
        return;
    }
    // -------------------------------------------------------------------
    // NO MODEL UNDER THE MOUSE
    // If intersects with ground plane, detect panning and orbiting events
    Vector3d point;
    if(computeMouseWorldPositionGround(&point,current_mp)){
        Vector3d camPos = camera->WorldPosition();
        double d = camPos.Distance(point);
        if(d>0.0 && d<50.0) indicator->SetScale(indicator_scale*((mag_indicator+d)/mag_indicator));
        indicator->SetWorldPosition(point);
        indicator->SetVisible(true);
        isModelFollowed = false;
        modelFollower->SetVisible(false);
        if(event->buttons()&Qt::LeftButton){
            panning = true;
        } else if(event->buttons()&Qt::RightButton){
            heighting = true;
        } else if(event->buttons()&Qt::MiddleButton){
            orbiting = true;
            orbit_point = point;
        }
    }
}

bool GzWidget::isSelectableModel()
{
    std::string trans = "translate";
    rendering::VisualPtr vis = camera->GetVisual(Vector2i(current_mp.x(),current_mp.y()),trans);
    if(vis && !vis->IsPlane() && !vis->IsStatic()){
        vis = vis->GetRootVisual();
        Box bbox = vis->GetBoundingBox().Ign();
        if(bbox.XLength()<max_selection_size && bbox.YLength()<max_selection_size && vis->GetTransparency()==0.0){
            tempModel = vis;
            return true;
        }
    }
    return false;
}

void GzWidget::highlightModel(VisualPtr visual)
{
    // If the visual is valid, resize and put the indicator below
    // the model
    if(!visual) return;
    Vector3d world_pos = visual->GetPosition().Ign();
    world_pos.Z() = 0;
    Box bbox = visual->GetBoundingBox().Ign();
    modelHighlighter->SetScale(Vector3d(bbox.XLength()*1.5,bbox.YLength()*1.5,0.01));
    modelHighlighter->SetPosition(world_pos);
    modelHighlighter->SetVisible(true);
    // If it is highlightable, change the cursor
    if(!isModelSelected) QApplication::setOverrideCursor(Qt::OpenHandCursor);
    else QApplication::setOverrideCursor(Qt::ClosedHandCursor);
}

void GzWidget::highlightFollowedModel(VisualPtr visual)
{
    // If the visual is valid, resize and put the indicator below
    // the model
    if(!visual) return;
    Vector3d world_pos = visual->GetPosition().Ign();
    world_pos.Z() = 0;
    Box bbox = visual->GetBoundingBox().Ign();
    modelFollower->SetScale(Vector3d(bbox.XLength()*1.5,bbox.YLength()*1.5,0.01));
    modelFollower->SetPosition(world_pos);
    modelFollower->SetVisible(true);
    //change camera position
    Vector3d camPos = camera->WorldPosition();
    Vector3d modelPos = followedModel->GetWorldPose().pos.Ign();
    camPos.X() = modelPos.X()+deltaFollow.X();
    camPos.Y() = modelPos.Y()+deltaFollow.Y();
    camPos.Z() = followHeight;
    camera->SetWorldPosition(camPos);
}

void GzWidget::updateIndicators()
{
    if(isFirstPerson){
        if(!followedModel || !camera) return;
        Pose3d camPose = camera->WorldPose();
        Vector3d modelPos = followedModel->GetWorldPose().pos.Ign();
        Pose3d pose; pose.Set(Vector3d(modelPos.X(),modelPos.Y(),modelPos.Z()+biasFPS.Z()),camPose.Rot().Euler());
        camera->SetWorldPose(pose);
        QApplication::setOverrideCursor(Qt::ArrowCursor);
        return;
    }
    if(isModelFollowed){
        highlightFollowedModel(followedModel);
    }

    if(isModelSelected) {
        highlightModel(highlightedModel);
    } else {
        if(isSelectableModel()) { highlightedModel = tempModel; highlightModel(highlightedModel); }
        else {
            QApplication::setOverrideCursor(Qt::ArrowCursor); modelHighlighter->SetVisible(false);
        }
    }
}

void GzWidget::rotateModelInWorld(std::__cxx11::string model_name, float amount)
{
    msgs::Model msg;
    msg.set_name(model_name);
    msg.set_id(0); // if id = 0, it is a relative transformation/rotation
    msgs::Set(msg.mutable_pose(), Pose3d(0, 0, amount, 0, 0, 0));
    modelPub->Publish(msg);
}

void GzWidget::setModelPoseInWorld(std::__cxx11::string model_name, Vector3d pos)
{
    msgs::Model msg;
    msg.set_name(model_name);
    msg.set_id(4); // if id = 0, it is a relative transformation/rotation
    msgs::Set(msg.mutable_pose(), Pose3d(pos.X(), pos.Y(), pos.Z(), 0, 0, 0));
    modelPub->Publish(msg);
}

void GzWidget::panModelInWorld(std::__cxx11::string model_name, Vector3d motion_vector)
{
    msgs::Model msg;
    msg.set_name(model_name);
    msg.set_id(1); // if id = 1, it is a relative transformation/rotation
    msgs::Set(msg.mutable_pose(), Pose3d(motion_vector.X(), motion_vector.Y(), 0, 0, 0, 0));
    modelPub->Publish(msg);
}

void GzWidget::heightModelInWorld(std::__cxx11::string model_name, float amount)
{
    msgs::Model msg;
    msg.set_name(model_name);
    msg.set_id(2); // if id = 2, it is a relative change in height
    msgs::Set(msg.mutable_pose(), Pose3d(0, 0, amount, 0, 0, 0));
    modelPub->Publish(msg);
}

bool GzWidget::computeMouseWorldPositionGround(Vector3d *point, QPoint mp)
{
    Vector3d ray, origin;
    origin = Vector3d(0.0,0.0,0.0);
    camera->CameraToViewportRay(mp.x(),mp.y(),origin,ray);
    return intersectsWithGroundLevel(point,ray,0.0);
}

void GzWidget::launchModelContextMenu(QPoint mp, VisualPtr model)
{
    if(!model) return;
    QMenu contextMenu(tr(model->GetName().c_str()), this);
    contextMenu.setPalette(this->palette());
    contextMenu.setFont(this->font());
    contextMenu.setTitle(QString::fromStdString(model->GetName()));
    followedModel = model;
    QAction follow("Follow Model", this);
    QAction first_person("First Person View",this);

    connect(&follow,SIGNAL(triggered(bool)),this,SLOT(followModel()));
    connect(&first_person,SIGNAL(triggered(bool)),this,SLOT(moveToFirstPerson()));

    QAction name(model->GetName().c_str(), this);
    name.setEnabled(false);
    contextMenu.addAction(&name);
    contextMenu.addAction(&follow);
    contextMenu.addAction(&first_person);
    QApplication::setOverrideCursor(Qt::ArrowCursor);
    contextMenu.exec(mapToGlobal(mp));
}

void GzWidget::followModel()
{
    isModelFollowed = true;
    modelFollower->SetMaterial("Gazebo/YellowTransparent");
    modelHighlighter->SetVisible(false);
    deltaFollow = camera->WorldPosition()-followedModel->GetWorldPose().pos.Ign();
   followHeight = camera->WorldPosition().Z();
}

void GzWidget::moveToFirstPerson()
{
   isModelFollowed = isModelSelected = false;
   modelFollower->SetVisible(false);
   modelHighlighter->SetVisible(false);
   isFirstPerson = true;

   beforeFPS = camera->WorldPose();
   Box bbox = followedModel->GetBoundingBox().Ign();
   fpsMaxHeight = bbox.ZLength(); // change accord to model
   followedModel->SetTransparency(1.0);
   Vector3d camPos = followedModel->GetPosition().Ign();
   camPos.Z() = fpsMaxHeight/2.0;
   Pose3d pose; pose.Set(camPos,Vector3d(0,0,0));
   camera->SetWorldPose(pose);
   biasFPS = Vector3d(0,0,camPos.Z());
}

void GzWidget::setCameraPose(Vector3d pos, Vector3d rot)
{
    math::Pose camPose;
    camPose.Set(pos,rot);
    camera->SetWorldPose(camPose);
}
