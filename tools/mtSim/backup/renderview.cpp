#include "renderview.h"

RenderView::RenderView(QWidget *parent):
QGraphicsView(parent)
{
    this->setHorizontalScrollBarPolicy ( Qt::ScrollBarAlwaysOff );
    this->setVerticalScrollBarPolicy ( Qt::ScrollBarAlwaysOff );
}

void RenderView::mouseMoveEvent(QMouseEvent *event)
{
    current_ = event->pos();
    delta_ = current_-last_;
    if((event->buttons()&Qt::MidButton)&&_ptr_){ //Orbitate
        if(delta_.x()>50 || delta_.y()>50 || delta_.x()<-50 || delta_.y()<-50){
            last_ = current_;
        } else {
            _ptr_->changeCameraYaw((-(double)delta_.x())*0.00872665);
            _ptr_->changeCameraPitch(((double)delta_.y())*0.00872665);
        }
    }else if((event->buttons()&Qt::LeftButton)&&_ptr_){//Pan
        if(delta_.x()>50 || delta_.y()>50 || delta_.x()<-50 || delta_.y()<-50){
            last_ = current_;
        } else {
            double yaw = _ptr_->getCameraYaw();
            _ptr_->changeCameraY((delta_.x()*cos(yaw)+delta_.y()*sin(yaw))*0.05);
            _ptr_->changeCameraX(-(delta_.x()*sin(yaw)-delta_.y()*cos(yaw))*0.05);
        }
    }else if((event->buttons()&Qt::RightButton)&&_ptr_){//Simple Zoom
        if(delta_.x()>50 || delta_.y()>50 || delta_.x()<-50 || delta_.y()<-50){
            last_ = current_;
        } else {
            _ptr_->changeCameraZ(delta_.y()*0.05);
        }
    }
    last_ = current_;
}

void RenderView::wheelEvent(QWheelEvent *event)
{
    /*glm::vec3 ray = rayCasting(event);
    glm::vec3 point;
    if(hasIntersectionWithZPlane(&point,ray,0.0)){
        qDebug() << "Mouse Point = [" << event->x() << " , " << event->y() << " ]";
        qDebug() << "World Point = [" << point.x << " , " << point.y << " , " << point.z << " ]";
        qDebug() << "";
    }*/
    qDebug() << this->width() << _ptr_->getWidth() << "\n"
             << this->height() << _ptr_->getHeight() << "\n"
             << (double)this->width()/(double)this->height() << _ptr_->getAspectRatio();

    vec3 ray = get_ray_from_mouse(event);
    glm::vec3 _ray = glm::vec3(ray.v[0],ray.v[1],ray.v[2]);
    qDebug() << _ray.x << _ray.y << _ray.z;
    glm::vec3 point;
    if(hasIntersectionWithZPlane(&point,_ray,0.0)){
        qDebug() << "World Point = [" << point.x << " , " << point.y << " , " << point.z << " ]";
        qDebug() << "";
    }

}

glm::fvec3 RenderView::rayCasting(QWheelEvent *event)
{
    /*updateProjectionMatrix(0.1,50.0);
    updateViewMatrix();
    // Get view port coordinates
    glm::vec2 viewport = glm::vec2(event->x(), event->y());
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
    return ray;*/
}

bool RenderView::hasIntersectionWithZPlane(glm::vec3 *_point, glm::vec3 ray, float target_z)
{
    _cameraview view = _ptr_->getCameraView();
    glm::vec3 c0 = glm::vec3(view.g_cameraXPosition,view.g_cameraYPosition,view.g_cameraZPosition);
    glm::vec3 p0 = glm::vec3(0.0,0.0,0.0);
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

vec3 RenderView::get_ray_from_mouse (QWheelEvent *event) {
    updateProjectionMatrix(0.1,50.0);
    updateViewMatrix();
    // screen space (viewport coordinates)
    float x = (2.0f * event->x()) / this->width() - 1.0f;
    float y = 1.0f - (2.0f * event->y()) / this->height();
    float z = 1.0f;
    // normalised device space
    vec3 ray_nds = vec3 (x, y, z);
    // clip space
    vec4 ray_clip = vec4 (ray_nds.v[0], ray_nds.v[1], -1.0, 1.0);
    // eye space
    vec4 ray_eye = inverse (proj_mat) * ray_clip;
    ray_eye = vec4 (ray_eye.v[0], ray_eye.v[1], -1.0, 0.0);
    // world space
    vec3 ray_wor = vec3 (inverse (view_mat) * ray_eye);
    // don't forget to normalise the vector at some point
    ray_wor = normalise(ray_wor);
    return ray_wor;
}

void RenderView::setRenderingCamera(RenderingCamera *_ptr)
{
    _ptr_ = _ptr;
}

void RenderView::updateProjectionMatrix(float nearDist, float farDist) //RH - Correct
{
    /*memset(&projection_matrix_,0,sizeof(glm::mat4));
    //projection_matrix_ = glm::perspectiveRH(_ptr_->getFOV(),_ptr_->getAspectRatio(),nearDist,farDist);*/
    proj_mat = perspective(90.0, _ptr_->getAspectRatio(), nearDist, farDist);
}


void RenderView::updateViewMatrix()
{
    memset(&view_matrix_,0,sizeof(glm::mat4));
    _cameraview view = _ptr_->getCameraView();
    /*glm::vec3 cam_pos = glm::vec3(view.g_cameraXPosition,view.g_cameraYPosition,view.g_cameraZPosition);
    glm::vec3 targ_pos = glm::vec3(0.0,0.0,0.0);
    glm::vec3 up = glm::vec3(0.0,1.0,0.0);
    // inverse translation
    glm::mat4 p = glm::mat4();
    p = glm::translate(p, glm::vec3 (-cam_pos.x, -cam_pos.y, -cam_pos.z));
    // distance vector
    glm::vec3 d = targ_pos - cam_pos;
    // forward vector
    glm::vec3 f = glm::normalize(d);
    // right vector
    glm::vec3 r = glm::normalize(glm::cross(f,up));
    // real up vector
    glm::vec3 u = glm::normalize(glm::cross(r,f));
    glm::mat4 ori = glm::mat4();

    ori[0][0] = r.x;
    ori[1][0] = r.y;
    ori[2][0] = r.z;
    ori[0][1] = u.x;
    ori[1][1] = u.y;
    ori[2][1] = u.z;
    ori[0][2] = -f.x;
    ori[1][2] = -f.y;
    ori[2][2] = -f.z;

    view_matrix_ = ori*p;//p * ori;*/
    /*glm::mat4 yaw_rot = glm::mat4();
    glm::mat4 pitch_rot = glm::mat4();
    glm::mat4 roll_rot;




    glm::vec3 forward = glm::vec3(rotation,ORIENTATION_DEFAULT_FORWARD);
    glm::vec3 up = glm::rotate(rotation, ORIENTATION_DEFAULT_UP);
    view_matrix_ = glm::lookAt(
        glm::vec3(view.g_cameraXPosition,view.g_cameraYPosition,view.g_cameraZPosition),
        forward,
        up
    );*/
    view_mat = look_at(vec3(view.g_cameraXPosition,view.g_cameraYPosition,view.g_cameraZPosition),
                       vec3(0.0,0.0,0.0),
                       vec3(0.0,1.0,0.0));
}
