#include <dwl_rviz_plugin/ReducedTrajectoryDisplay.h>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>
#include <OgreBillboardSet.h>
#include <OgreMatrix4.h>

#include <tf/transform_listener.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/validate_floats.h>

#include <rviz/ogre_helpers/billboard_line.h>


using namespace rviz;

namespace dwl_rviz_plugin
{

ReducedTrajectoryDisplay::ReducedTrajectoryDisplay()
{
	// Category Groups
	com_category_ = new rviz::Property("Center of Mass", QVariant(), "", this);
	cop_category_ = new rviz::Property("Center of Pressure", QVariant(), "", this);
	support_category_ = new rviz::Property("Support Region", QVariant(), "", this);
	pendulum_category_ = new rviz::Property("Pendulum", QVariant(), "", this);


	// CoM properties
	com_alpha_property_ =
			new rviz::FloatProperty("Alpha", 1.0,
									"0 is fully transparent, 1.0 is fully opaque.",
									com_category_, SLOT(updateCoMRadiusAndAlpha()), this);
	com_alpha_property_->setMin(0);
	com_alpha_property_->setMax(1);

	com_radius_property_ =
			new rviz::FloatProperty("Radius", 0.04,
									"Radius of a point",
									com_category_, SLOT(updateCoMRadiusAndAlpha()), this);


	// CoP properties
	cop_alpha_property_ =
			new rviz::FloatProperty("Alpha", 1.0,
									"0 is fully transparent, 1.0 is fully opaque.",
									cop_category_, SLOT(updateCoPRadiusAndAlpha()), this);
	cop_alpha_property_->setMin(0);
	cop_alpha_property_->setMax(1);

	cop_radius_property_ =
			new rviz::FloatProperty("Radius", 0.04,
									"Radius of a point",
									cop_category_, SLOT(updateCoPRadiusAndAlpha()), this);


	// Pendulum properties
	pendulum_shaft_length_property_ =
			new FloatProperty("Shaft Length", 0.4,
							  "Length of the arrow's shaft, in meters.",
							  pendulum_category_, SLOT(updatePendulumArrowGeometry()), this);

	pendulum_shaft_radius_property_ =
			new FloatProperty("Shaft Radius", 0.02,
							  "Radius of the arrow's shaft, in meters.",
							  pendulum_category_, SLOT(updatePendulumArrowGeometry()), this);

	pendulum_head_length_property_ =
			new FloatProperty("Head Length", 0.08,
							  "Length of the arrow's head, in meters.",
							  pendulum_category_, SLOT(updatePendulumArrowGeometry()), this);

	pendulum_head_radius_property_ =
			new FloatProperty("Head Radius", 0.04,
							  "Radius of the arrow's head, in meters.",
							  pendulum_category_, SLOT(updatePendulumArrowGeometry()), this);
}


ReducedTrajectoryDisplay::~ReducedTrajectoryDisplay()
{
	destroyObjects();
}


void ReducedTrajectoryDisplay::onInitialize()
{
	MFDClass::onInitialize();
}

void ReducedTrajectoryDisplay::reset()
{
	MFDClass::reset();

	com_visual_.clear();
	cop_visual_.clear();
	pendulum_visual_.clear();
}


void ReducedTrajectoryDisplay::updateCoMRadiusAndAlpha()
{
	com_radius_ = com_radius_property_->getFloat();
	com_alpha_ = com_alpha_property_->getFloat();

	for (size_t i = 0; i < com_visual_.size(); i++)
		com_visual_[i]->setRadius(com_radius_);

	context_->queueRender();
}


void ReducedTrajectoryDisplay::updateCoPRadiusAndAlpha()
{
	cop_radius_ = cop_radius_property_->getFloat();
	cop_alpha_ = cop_alpha_property_->getFloat();

	for (size_t i = 0; i < cop_visual_.size(); i++)
		cop_visual_[i]->setRadius(cop_radius_);

	context_->queueRender();
}


void ReducedTrajectoryDisplay::updatePendulumArrowGeometry()
{
	float shaft_length = pendulum_shaft_length_property_->getFloat();
	float shaft_radius = pendulum_shaft_radius_property_->getFloat();
	float head_length = pendulum_head_length_property_->getFloat();
	float head_radius = pendulum_head_radius_property_->getFloat();

	for (size_t i = 0; i < pendulum_visual_.size(); i++)
		pendulum_visual_[i]->setProperties(shaft_length, shaft_radius,
										   head_length, head_radius);

	context_->queueRender();
}


void ReducedTrajectoryDisplay::processMessage(const dwl_msgs::ReducedTrajectory::ConstPtr& msg)
{
	// Here we call the rviz::FrameManager to get the transform from the
	// fixed frame to the frame in the header of this Point message.  If
	// it fails, we can't do anything else so we return.
	Ogre::Quaternion orientation;
	Ogre::Vector3 position;
	if (!context_->getFrameManager()->getTransform(msg->header.frame_id,
												   msg->header.stamp,
												   position, orientation)) {
		ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
				  msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
		return;
	}


	com_visual_.clear();
	cop_visual_.clear();
	for (unsigned int k = 0; k < msg->trajectory.size(); k++) {
		dwl_msgs::ReducedState state = msg->trajectory[k];

		// Getting the center of mass position
		geometry_msgs::Vector3 com_vec = state.center_of_mass;
		Ogre::Vector3 com_pos(com_vec.x, com_vec.y, com_vec.z);

		// Getting the center of pressure position
		geometry_msgs::Vector3 cop_vec = state.center_of_pressure;
		Ogre::Vector3 cop_pos(cop_vec.x, cop_vec.y, cop_vec.z);


		// We are keeping a vector of visual pointers. This creates the next one and stores it
		// in the vector
		boost::shared_ptr<PointVisual> com_visual;
		com_visual.reset(new PointVisual(context_->getSceneManager(), scene_node_));
		com_visual->setPoint(com_pos);
		com_visual->setFramePosition(position);
		com_visual->setFrameOrientation(orientation);

		// Setting the point properties
		updateCoMRadiusAndAlpha();
		Ogre::ColourValue color;// = com_color_property_->getOgreColor();
		color.a = com_alpha_;
		com_visual->setColor(color.r, color.g, color.b, color.a);

		// And send it to the end of the vector
		com_visual_.push_back(com_visual);
	}
}

} //@namespace dwl_rviz_plugin
