#include "renderview.h"

///
/// \brief RenderView::RenderView Constructor of RenderView widget
/// which subclasses QGraphicsScene to render 3D Scene in Qt.
/// \param parent widget.
///
RenderView::RenderView(QWidget *parent):
QGraphicsView(parent)
{
    this->setHorizontalScrollBarPolicy ( Qt::ScrollBarAlwaysOff );
    this->setVerticalScrollBarPolicy ( Qt::ScrollBarAlwaysOff );
    this->setMouseTracking(true);
    in_orbit_ = false;
}

///
/// \brief RenderView::mouseMoveEvent detects mouse move events to provide PAN,
/// ORBIT and HEIGHT-PAN, changing the scene's camera position and orientation.
/// \param event from mouse event (provides information about mouse state).
///
void RenderView::mouseMoveEvent(QMouseEvent *event)
{
    current_ = event->pos();
    delta_ = current_-last_;
    if((event->buttons()&Qt::MidButton)&&_ptr_){ //Orbit
        if(delta_.x()>50 || delta_.y()>50 || delta_.x()<-50 || delta_.y()<-50){
            last_ = current_;
        } else {
            _cameraview view = _ptr_->getCameraView(); // camera position
            glm::vec3 point;
            if(screenToWorldFloor(event->x(),event->y(),&point) && !in_orbit_){
                in_orbit_ = true;
                orbit_point_ = point;
            }

            if(in_orbit_){
                glm::vec3 camera = glm::vec3(view.g_cameraXPosition, view.g_cameraYPosition, view.g_cameraZPosition);
                float angle_yaw = -(float)delta_.x()*0.00872665;
                float angle_pitch = (float)delta_.y()*0.00872665;
                float rotatedX = cos(angle_yaw)*(camera.x-orbit_point_.x)-
                                sin(angle_yaw)*(camera.y-orbit_point_.y)+orbit_point_.x;
                float rotatedY = sin(angle_yaw)*(camera.x-orbit_point_.x)+
                                cos(angle_yaw)*(camera.y-orbit_point_.y)+orbit_point_.y;
                float rotatedZ = sin(angle_pitch)*(camera.x-orbit_point_.x)+
                                cos(angle_pitch)*(camera.z-orbit_point_.z)+orbit_point_.z;
                _ptr_->setCameraPosition(rotatedX,rotatedY,rotatedZ);
                _ptr_->changeCameraYaw(angle_yaw);
                _ptr_->changeCameraPitch(angle_pitch);

            }
        }
    }else if((event->buttons()&Qt::LeftButton)&&_ptr_){//Pan X-Y
        if(delta_.x()>50 || delta_.y()>50 || delta_.x()<-50 || delta_.y()<-50){
            last_ = current_;
        } else {
            double yaw = _ptr_->getCameraYaw();
            _ptr_->changeCameraY((delta_.x()*cos(yaw)+delta_.y()*sin(yaw))*0.025);
            _ptr_->changeCameraX(-(delta_.x()*sin(yaw)-delta_.y()*cos(yaw))*0.025);
        }
    }else if((event->buttons()&Qt::RightButton)&&_ptr_){//Simple Height change
        if(delta_.x()>50 || delta_.y()>50 || delta_.x()<-50 || delta_.y()<-50){
            last_ = current_;
        } else {
            _ptr_->changeCameraZ(delta_.y()*0.03);
        }
    }
    last_ = current_;
}

///
/// \brief RenderView::mousePressEvent detects mouse click events to display
/// world coordinates on viewport, in the matching position where the scene
/// has been clicked.
/// \param event from mouse event (provides information about mouse state).
///
void RenderView::mousePressEvent(QMouseEvent *event)
{
    glm::vec3 point;
    if(screenToWorldFloor(event->x(),event->y(),&point)){
        QToolTip::showText(this->mapToGlobal(QPoint(event->x(),event->y())),
                           QString(QString("[")+
                                   QString::number(point.x,'f',2)+QString(",")+
                                   QString::number(point.y,'f',2)+QString(",")+
                                   QString::number(point.z,'f',2)+
                                   QString("]")
                           ));
    }
    event->accept();
}

///
/// \brief RenderView::mouseReleaseEvent detects release of a button, that
/// influences the orbit function, implemented in mousePressEvent.
/// \param event from mouse event (provides information about mouse state).
///
void RenderView::mouseReleaseEvent(QMouseEvent *event)
{
    if((event->buttons()|Qt::MidButton)){
        in_orbit_ = false;
    }
}

///
/// \brief RenderView::wheelEvent detects scrool wheel movement (up and down)
/// to provide zoom-in and zoom-out abilites to control the view of the scene.
/// \param event from mouse event (provides information about mouse state).
///
void RenderView::wheelEvent(QWheelEvent *event) // Zoom
{
    glm::vec3 ray = rayCasting(event->x(),event->y());
    glm::vec3 point;
    double zoomFactor = 0.5;
    if(hasIntersectionWithZPlane(&point,ray,0.0)){
        _cameraview view = _ptr_->getCameraView();
        double d = glm::distance(point,glm::vec3(view.g_cameraXPosition,
                                                 view.g_cameraYPosition,
                                                 view.g_cameraZPosition));
        if(fabs(d)<3.0) zoomFactor/=3.0;
        if(event->delta()<0) zoomFactor *= -1.0;

        _ptr_->changeCameraX(zoomFactor*ray.x);
        _ptr_->changeCameraY(zoomFactor*ray.y);
        _ptr_->changeCameraZ(zoomFactor*ray.z);
    }
}

///
/// \brief RenderView::rayCasting casts a ray from viewport (screen) to world space (simulated
/// world) to allow the user to click/select positions and objects in the 3D scene from
/// it's 2D screen visualization.
/// \param x - X position of the mouse in the viewport where the ray should be casted.
/// \param y - Y pPosition of the mouse in the viewport where the ray should be casted.
/// \return - Returns a directional vector which contains the direction of the casted ray.
///
glm::fvec3 RenderView::rayCasting(int x,int y) // Cast ray from viewport to world space
{
    updateProjectionMatrix(0.1,50.0);
    updateViewMatrix();
    // Get view port coordinates
    glm::vec2 viewport = glm::vec2(x,y);
    // Get normalized device coordinates
    glm::vec3 normalized_coords;
    normalized_coords.x = (2.0f*viewport.x)/this->width()-1.0f;
    normalized_coords.y = 1.0f-(2.0f*viewport.y)/this->height();
    normalized_coords.z = 1.0f;
    // Get homogeneous clip coordinates
    glm::vec4 clip_coords;
    clip_coords = glm::vec4(normalized_coords.x,normalized_coords.y,-1.0,1.0);
    // Get eye coordinates
    glm::vec4 eye_coords = glm::inverse(projection_matrix_)*clip_coords;
    eye_coords = glm::vec4(eye_coords.x,eye_coords.y,-1.0,0.0);
    // Get world coordinates
    glm::vec4 ray_world = glm::inverse(view_matrix_)*eye_coords;
    glm::vec3 ray = glm::vec3(ray_world.x,ray_world.y,ray_world.z);
    ray = glm::normalize(ray);
    return ray;
}

///
/// \brief RenderView::hasIntersectionWithZPlane performs the intersection of the casted ray
/// with a plane paralel to the XY plane (floor).
/// \param _point - Point of intersection [x,y,z].
/// \param ray - Direction of the casted ray, from the viewport selection.
/// \param target_z - Target Z(height) of the plane to intersect with in meters.
/// \return - Returns true if an intersection point exists (_point) and false if there is no
/// solution.
///
bool RenderView::hasIntersectionWithZPlane(glm::vec3 *_point, glm::vec3 ray, float target_z) // Ray picking on plane Z=x
{
    _cameraview view = _ptr_->getCameraView();
    glm::vec3 c0 = glm::vec3(view.g_cameraXPosition,view.g_cameraYPosition,view.g_cameraZPosition);
    glm::vec3 p0 = glm::vec3(0.0,0.0,target_z);
    glm::vec3 n = glm::vec3(0.0,0.0,1.0);
    if(glm::dot(ray,n)==0){
        return false;
    } else {
        float d = (glm::dot((p0-c0),n))/(glm::dot(ray,n));
        glm::vec3 intersection_point = d*ray+c0;
        _point->x = intersection_point.x;
        _point->y = intersection_point.y;
        _point->z = intersection_point.z;
        return true;
    }
}

///
/// \brief RenderView::screenToWorldFloor subimplements RenderView::hasIntersectionWithZPlane
/// to make the detection in the ground plane (Z=0).
/// \param x - X position of the mouse in the viewport where the ray should be casted.
/// \param y - Y position of the mouse in the viewport where the ray should be casted.
/// \param _point - Intersection point between the casted ray and the floor plane (Z=0)
/// \return - Returns true if an intersection point exists (_point) and false if there is no
/// solution.
///
bool RenderView::screenToWorldFloor(int x, int y, glm::vec3 *_point) // Ray picking on Z = 0
{
    glm::vec3 ray = rayCasting(x,y);
    glm::vec3 point;
    if(hasIntersectionWithZPlane(&point,ray,0.0)){
        _point->x = point.x;
        _point->y = point.y;
        _point->z = point.z;
        return true;
    }else return false;
}

///
/// \brief RenderView::setRenderingCamera sets the rendering camera, from the rendering library,
/// used in RenderingCamera class.
/// \param _ptr - Pointer to the camera.
///
void RenderView::setRenderingCamera(RenderingCamera *_ptr)
{
    _ptr_ = _ptr;
}

///
/// \brief RenderView::updateProjectionMatrix updates camera's projection matrix based on it's field
/// of view, aspect ratio and (far and near) clipping planes.
/// \param nearDist - Near distance clipping plane (meters, keep high as possible) in meters.
/// \param farDist - Far distance clipping plane (meters, keep low as possible) in meters.
///
void RenderView::updateProjectionMatrix(float nearDist, float farDist) //RH - Correct
{
    memset(&projection_matrix_,0,sizeof(glm::mat4));
    // get perspective/projection matrix
    projection_matrix_ = glm::perspectiveRH(_ptr_->getFOV(),_ptr_->getAspectRatio(),nearDist,farDist);
}

///
/// \brief RenderView::updateViewMatrix - Updates camera's view matrix bases on it's position and
/// orientation relative to the world origin (0,0,0).
///
void RenderView::updateViewMatrix()
{
    memset(&view_matrix_,0,sizeof(glm::mat4));
    _cameraview view = _ptr_->getCameraView();
     // start as identity matrix and make a translation to camera referencial
    glm::mat4 translation = glm::translate(glm::mat4(),
                        glm::vec3(-view.g_cameraXPosition,
                                  -view.g_cameraYPosition,
                                  -view.g_cameraZPosition));
    // rotate matrix for yaw and pitch rotations
    glm::mat4 rotation_yaw = glm::rotate(glm::mat4(),(float)-view.g_cameraYawRotation+(float)M_PI/2.0f,glm::vec3(0.0,0.0,1.0));
    glm::mat4 rotation_pitch = glm::rotate(glm::mat4(),(float)+view.g_cameraPitchRotation-(float)M_PI/2.0f,glm::vec3(1.0,0.0,0.0));
    view_matrix_ = translation*rotation_pitch*rotation_yaw;

}
