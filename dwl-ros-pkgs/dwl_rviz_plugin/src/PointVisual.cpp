#include <OgreVector3.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <ros/ros.h>
#include <rviz/ogre_helpers/shape.h>
#include <dwl_rviz_plugin/PointVisual.h>


namespace dwl_rviz_plugin
{

PointVisual::PointVisual(Ogre::SceneManager* scene_manager,
						 Ogre::SceneNode* parent_node) : radius_(0.)
{
	scene_manager_ = scene_manager;

	// Ogre::SceneNode s form a tree, with each node storing the
	// transform (position and orientation) of itself relative to its
	// parent.  Ogre does the math of combining those transforms when it
	// is time to render.
	//
	// Here we create a node to store the pose of the Point's header frame
	// relative to the RViz fixed frame.
	frame_node_ = parent_node->createChildSceneNode();

	// We create the arrow object within the frame node so that we can
	// set its position and direction relative to its header frame.
	point_ = new rviz::Shape(rviz::Shape::Sphere, scene_manager_, frame_node_);
}


PointVisual::~PointVisual()
{
	// Delete the arrow to make it disappear.
	delete point_;

	// Destroy the frame node since we don't need it anymore.
	scene_manager_->destroySceneNode( frame_node_ );
}


void PointVisual::setPoint(const Ogre::Vector3& point)
{
	Ogre::Vector3 scale(radius_, radius_, radius_);
	point_->setScale(scale);

	// Set the orientation of the arrow to match the direction of the
	// acceleration vector.
	point_->setPosition(point);
}


void PointVisual::setFramePosition(const Ogre::Vector3& position)
{
	frame_node_->setPosition(position);
}


void PointVisual::setFrameOrientation(const Ogre::Quaternion& orientation)
{
	frame_node_->setOrientation(orientation);
}


void PointVisual::setColor(float r, float g, float b, float a)
{
	point_->setColor(r, g, b, a);
}


void PointVisual::setRadius(float r)
{
	radius_ = r;
}

} //@namespace dwl_rviz_plugin
