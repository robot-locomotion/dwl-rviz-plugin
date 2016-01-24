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
			new rviz::FloatProperty("Radius", 0.03,
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
			new rviz::FloatProperty("Radius", 0.03,
									"Radius of a point",
									cop_category_, SLOT(updateCoPRadiusAndAlpha()), this);


	// Support polygon properties
	support_line_alpha_property_ =
			new rviz::FloatProperty("Line Alpha", 1.0,
									"0 is fully transparent, 1.0 is fully opaque.",
									support_category_, SLOT(updateSupportAlpha()), this);
	support_line_alpha_property_->setMin(0);
	support_line_alpha_property_->setMax(1);

	support_mesh_alpha_property_ =
			new rviz::FloatProperty("Mesh Alpha", 0.2,
									"0 is fully transparent, 1.0 is fully opaque.",
									support_category_, SLOT(updateSupportAlpha()), this);
	support_mesh_alpha_property_->setMin(0);
	support_mesh_alpha_property_->setMax(1);


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
	destroyObjects();
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


void ReducedTrajectoryDisplay::updateSupportAlpha()
{
	support_line_alpha_ = support_line_alpha_property_->getFloat();
	support_mesh_alpha_ = support_mesh_alpha_property_->getFloat();

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


	// Compute the set of colors
	std::vector<Ogre::ColourValue> colors;
	generateSetOfColors(colors, msg->trajectory.size());

	com_visual_.clear();
	cop_visual_.clear();
	support_visual_.clear();
	pendulum_visual_.clear();
	for (unsigned int k = 0; k < msg->trajectory.size(); k++) {
		dwl_msgs::ReducedState state = msg->trajectory[k];

		// Getting the center of mass position
		geometry_msgs::Vector3 com_vec = state.center_of_mass;
		Ogre::Vector3 com_pos(com_vec.x, com_vec.y, com_vec.z);

		// Getting the actual color
		Ogre::ColourValue color = colors[k];//(1, 0., 0., 1.);// = com_color_property_->getOgreColor();

		// We are keeping a vector of CoM visual pointers. This creates the next
		// one and stores it in the vector
		boost::shared_ptr<PointVisual> com_visual;
		com_visual.reset(new PointVisual(context_->getSceneManager(), scene_node_));
		updateCoMRadiusAndAlpha();
		color.a = com_alpha_;
		com_visual->setColor(color.r, color.g, color.b, color.a);
		com_visual->setRadius(com_radius_);
		com_visual->setPoint(com_pos);
		com_visual->setFramePosition(position);
		com_visual->setFrameOrientation(orientation);

		// And send it to the end of the vector
		com_visual_.push_back(com_visual);


		// Getting the center of pressure position
		geometry_msgs::Vector3 cop_vec = state.center_of_pressure;
		Ogre::Vector3 cop_pos(cop_vec.x, cop_vec.y, cop_vec.z);

		// We are keeping a vector of CoP visual pointers. This creates the next
		// one and stores it in the vector
		boost::shared_ptr<PointVisual> cop_visual;
		cop_visual.reset(new PointVisual(context_->getSceneManager(), scene_node_));
		updateCoPRadiusAndAlpha();
		color.a = cop_alpha_;
		cop_visual->setColor(color.r, color.g, color.b, color.a);
		cop_visual->setRadius(cop_radius_);
		cop_visual->setPoint(cop_pos);
		cop_visual->setFramePosition(position);
		cop_visual->setFrameOrientation(orientation);

		// And send it to the end of the vector
		cop_visual_.push_back(cop_visual);


		// Getting the support region
		std::vector<Ogre::Vector3> support;
		support.resize(state.support_region.size());
		for (unsigned int v = 0; v < state.support_region.size(); v++) {
			support[v].x = state.support_region[v].x;
			support[v].y = state.support_region[v].y;
			support[v].z = state.support_region[v].z;
		}

		// Now set or update the contents of the chosen suppor visual
		// We are keeping a vector of support regions visual pointers. This
		// creates the next one and stores it in the vector
		boost::shared_ptr<PolygonVisual> polygon_visual;
		polygon_visual.reset(new PolygonVisual(context_->getSceneManager(), scene_node_));
		updateSupportAlpha();
		polygon_visual->setVertexs(support);
		polygon_visual->setLineColor(color.r, color.g, color.b, support_line_alpha_);
		polygon_visual->setScale(Ogre::Vector3(1., 1., 1.));
		polygon_visual->setMeshColor(color.r, color.g, color.b, support_mesh_alpha_);
		polygon_visual->setFramePosition(position);
		polygon_visual->setFrameOrientation(orientation);

		// And send it to the end of the vector
		support_visual_.push_back(polygon_visual);
	}
}


void ReducedTrajectoryDisplay::destroyObjects()
{
	com_visual_.clear();
	cop_visual_.clear();
	support_visual_.clear();
	pendulum_visual_.clear();
}


void ReducedTrajectoryDisplay::generateSetOfColors(std::vector<Ogre::ColourValue>& colors,
												   unsigned int num_points)
{
	colors.resize(num_points);
	for (int i = 0; i < num_points; i++) {
		float hue = (float) i / (float) num_points;
		float saturation = (float) (90 + rand() % 10) / 100;
		float brightness = (float) (50 + rand() % 10) / 100;

		colors[i].setHSB(hue, saturation, brightness);
	}
}

} //@namespace dwl_rviz_plugin


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(dwl_rviz_plugin::ReducedTrajectoryDisplay, rviz::Display)
