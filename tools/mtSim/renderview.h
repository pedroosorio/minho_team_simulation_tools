#ifndef RENDERVIEW_H
#define RENDERVIEW_H

///
/// \brief This code is part of the MinhoTeam Simulation Tools, developed by Pedro
/// Osório Silva (pedroosorio.eeic@gmail.com) and it's free to distribute and edit.
///
///

#include <QObject>
#include <QWidget>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QMouseEvent>
#include <QDebug>
#include <QToolTip>
#include "renderingcamera.h"

// 3D Point mapping
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/rotate_vector.hpp>

class RenderView : public QGraphicsView
{
    Q_OBJECT
public:
    ///
    /// \brief RenderView::RenderView Constructor of RenderView widget
    /// which subclasses QGraphicsScene to render 3D Scene in Qt.
    /// \param parent widget.
    ///
    explicit RenderView(QWidget *parent = 0);

    ///
    /// \brief RenderView::setRenderingCamera sets the rendering camera, from the
    /// rendering library used in RenderingCamera class.
    /// \param _ptr - Pointer to the camera.
    ///
    void setRenderingCamera(RenderingCamera *_ptr);
protected:
    ///
    /// \brief RenderView::mouseMoveEvent detects mouse move events to provide PAN,
    /// ORBIT and HEIGHT-PAN, changing the scene's camera position and orientation.
    /// \param event from mouse event (provides information about mouse state).
    ///
    void mouseMoveEvent(QMouseEvent *event);

    ///
    /// \brief RenderView::mousePressEvent detects mouse click events to display
    /// world coordinates on viewport, in the matching position where the scene
    /// has been clicked.
    /// \param event from mouse event (provides information about mouse state).
    ///
    void mousePressEvent(QMouseEvent *event);

    ///
    /// \brief RenderView::mouseReleaseEvent detects release of a button, that
    /// influences the orbit function, implemented in mousePressEvent.
    /// \param event from mouse event (provides information about mouse state).
    ///
    void mouseReleaseEvent(QMouseEvent *event);

    ///
    /// \brief RenderView::wheelEvent detects scrool wheel movement (up and down)
    /// to provide zoom-in and zoom-out abilites to control the view of the scene.
    /// \param event from mouse event (provides information about mouse state).
    ///
    void wheelEvent(QWheelEvent * event);

    ///
    /// \brief RenderView::updateProjectionMatrix updates camera's projection matrix based on it's field
    /// of view, aspect ratio and (far and near) clipping planes.
    /// \param nearDist - Near distance clipping plane (meters, keep high as possible) in meters.
    /// \param farDist - Far distance clipping plane (meters, keep low as possible) in meters.
    ///
    void updateProjectionMatrix(float nearDist, float farDist);

    ///
    /// \brief RenderView::updateViewMatrix - Updates camera's view matrix bases on it's position and
    /// orientation relative to the world origin (0,0,0).
    ///
    void updateViewMatrix();
private slots:

    ///
    /// \brief RenderView::rayCasting casts a ray from viewport (screen) to world space (simulated
    /// world) to allow the user to click/select positions and objects in the 3D scene from
    /// it's 2D screen visualization.
    /// \param x - X position of the mouse in the viewport where the ray should be casted.
    /// \param y - Y pPosition of the mouse in the viewport where the ray should be casted.
    /// \return - Returns a directional vector which contains the direction of the casted ray.
    ///
    glm::fvec3 rayCasting(int x, int y);

    ///
    /// \brief RenderView::hasIntersectionWithZPlane performs the intersection of the casted ray
    /// with a plane paralel to the XY plane (floor).
    /// \param _point - Point of intersection [x,y,z].
    /// \param ray - Direction of the casted ray, from the viewport selection.
    /// \param target_z - Target Z(height) of the plane to intersect with in meters.
    /// \return - Returns true if an intersection point exists (_point) and false if there is no
    /// solution.
    ///
    bool hasIntersectionWithZPlane(glm::vec3 *_point, glm::vec3 ray, float target_z);

    ///
    /// \brief RenderView::screenToWorldFloor subimplements RenderView::hasIntersectionWithZPlane
    /// to make the detection in the ground plane (Z=0).
    /// \param x - X position of the mouse in the viewport where the ray should be casted.
    /// \param y - Y position of the mouse in the viewport where the ray should be casted.
    /// \param _point - Intersection point between the casted ray and the floor plane (Z=0)
    /// \return - Returns true if an intersection point exists (_point) and false if there is no
    /// solution.
    ///
    bool screenToWorldFloor(int x, int y, glm::vec3 *point);
private:
    ///
    /// \brief _ptr_ Pointer to rendering camera used by rendering class - RenderingCamera.
    ///
    RenderingCamera *_ptr_;
    ///
    /// \brief current_ Current position of mouse.
    /// \brief last_ Last position of mouse.
    /// \brief delta_ Difference between current and last mouse's positions.
    ///
    QPoint current_,last_,delta_;
    ///
    /// \brief projection_matrix_ Projection matrix of the camera that renders the 3D simulation,
    /// used in ray casting and world space to viewport mapping.
    ///
    glm::mat4 projection_matrix_;
    ///
    /// \brief projection_matrix_ View matrix of the camera that renders the 3D simulation,
    /// used in ray casting and world space to viewport mapping.
    ///
    glm::mat4 view_matrix_;

    ///
    /// \brief in_orbit_ flags if we are in an orbit motion (scroll button pressed) to mantain
    /// the focus (orbit_point_) the same.
    ///
    bool in_orbit_;

    ///
    /// \brief orbit_point_ world point that the camera orbits during orbit motion of the camera.
    ///
    glm::vec3 orbit_point_;
};

#endif // RENDERVIEW_H
