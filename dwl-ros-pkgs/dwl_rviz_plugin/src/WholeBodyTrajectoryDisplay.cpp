#include <dwl_rviz_plugin/WholeBodyTrajectoryDisplay.h>

#include <boost/bind.hpp>

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

WholeBodyTrajectoryDisplay::WholeBodyTrajectoryDisplay()
{
	// Category Groups
	base_category_ = new rviz::Property("Base", QVariant(), "", this);
	contact_category_ = new rviz::Property("End-Effector", QVariant(), "", this);

	// Base trajectory properties
	base_style_property_ =
			new EnumProperty("Line Style", "Lines",
							 "The rendering operation to use to draw the grid lines.",
							 base_category_, SLOT(updateBaseStyle()), this);

	base_style_property_->addOption("Lines", LINES);
	base_style_property_->addOption("Billboards", BILLBOARDS);

	base_line_width_property_ =
			new FloatProperty("Line Width", 0.01,
							  "The width, in meters, of each path line. "
							  "Only works with the 'Billboards' style.",
							  base_category_, SLOT(updateBaseLineWidth()), this);
	base_line_width_property_->setMin(0.001);
	base_line_width_property_->hide();

	base_color_property_ =
			new ColorProperty("Color", QColor(0, 85, 255),
							  "Color to draw the path.",
							  base_category_, NULL, this);

	base_alpha_property_ =
			new FloatProperty("Alpha", 1.0,
							  "Amount of transparency to apply to the path.",
							  base_category_, NULL, this);
	base_alpha_property_->setMin(0);
	base_alpha_property_->setMax(1);


	// End-effector trajectory properties
	contact_style_property_ =
			new EnumProperty("Line Style", "Lines",
							 "The rendering operation to use to draw the grid lines.",
							 contact_category_, SLOT(updateContactStyle()), this);

	contact_style_property_->addOption("Lines", LINES);
	contact_style_property_->addOption("Billboards", BILLBOARDS);

	contact_line_width_property_ =
			new FloatProperty("Line Width", 0.01,
							  "The width, in meters, of each path line. "
							  "Only works with the 'Billboards' style.",
							  contact_category_, SLOT(updateContactLineWidth()), this);
	contact_line_width_property_->setMin(0.001);
	contact_line_width_property_->hide();

	contact_color_property_ =
			new ColorProperty("Color", QColor(0, 85, 255),
							  "Color to draw the path.",
							  contact_category_, NULL, this);

	contact_alpha_property_ =
			new FloatProperty("Alpha", 1.0,
							  "Amount of transparency to apply to the path.",
							  contact_category_, NULL, this);
	contact_alpha_property_->setMin(0);
	contact_alpha_property_->setMax(1);
}


WholeBodyTrajectoryDisplay::~WholeBodyTrajectoryDisplay()
{
	// Delete the contact line to make it disappear.
	contact_manual_object_.clear();
	contact_billboard_line_.clear();
}


void WholeBodyTrajectoryDisplay::onInitialize()
{
	MFDClass::onInitialize();
}


void WholeBodyTrajectoryDisplay::reset()
{
	MFDClass::reset();
}


void WholeBodyTrajectoryDisplay::updateBaseStyle()
{
	LineStyle style = (LineStyle) base_style_property_->getOptionInt();

	switch (style)
	{
	case LINES:
	default:
		base_line_width_property_->hide();
		break;
	case BILLBOARDS:
		base_line_width_property_->show();
		break;
	}
}


void WholeBodyTrajectoryDisplay::updateBaseLineWidth()
{
	LineStyle style = (LineStyle) base_style_property_->getOptionInt();
	float line_width = base_line_width_property_->getFloat();

	if (style == BILLBOARDS)
		base_billboard_line_->setLineWidth(line_width);

	context_->queueRender();
}


void WholeBodyTrajectoryDisplay::updateContactStyle()
{
	LineStyle style = (LineStyle) contact_style_property_->getOptionInt();

	switch (style)
	{
	case LINES:
	default:
		contact_line_width_property_->hide();
		break;
	case BILLBOARDS:
		contact_line_width_property_->show();
		break;
	}
}


void WholeBodyTrajectoryDisplay::updateContactLineWidth()
{
	LineStyle style = (LineStyle) contact_style_property_->getOptionInt();
	float line_width = contact_line_width_property_->getFloat();

	if (style == BILLBOARDS) {
		unsigned int num_contact = contact_billboard_line_.size();
		for (unsigned int i = 0; i < num_contact; i++)
			contact_billboard_line_[i]->setLineWidth(line_width);
	}

	context_->queueRender();
}


void WholeBodyTrajectoryDisplay::processMessage(const dwl_msgs::WholeBodyTrajectory::ConstPtr& msg)
{
	// Lookup transform into fixed frame
	Ogre::Vector3 position;
	Ogre::Quaternion orientation;
	if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
		ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
				  msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
	}
	Ogre::Matrix4 transform(orientation);
	transform.setTrans(position);


	// Visualization of the base trajectory
	// Getting the base trajectory style
	LineStyle base_style = (LineStyle) base_style_property_->getOptionInt();

	// Getting the base trajectory color
	Ogre::ColourValue base_color = base_color_property_->getOgreColor();
	base_color.a = base_alpha_property_->getFloat();

	// Visualization of the base trajectory
	uint32_t num_points = msg->trajectory.size();
	uint32_t num_base = msg->trajectory[0].base.size();
	switch (base_style)
	{
	case LINES:
		base_manual_object_.reset(scene_manager_->createManualObject());
		base_manual_object_->setDynamic(true);
		scene_node_->attachObject(base_manual_object_.get());

		base_manual_object_->estimateVertexCount(num_points);
		base_manual_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);
		for (uint32_t i = 0; i < num_points; ++i) {
			Ogre::Vector3 pos(0., 0., 0.);
			for (uint32_t j = 0; j < num_base; j++) {
				dwl_msgs::BaseState base = msg->trajectory[i].base[j];
				if (base.id == dwl::rbd::LX)
					pos.x = base.position;
				else if (base.id == dwl::rbd::LY)
					pos.y = base.position;
				else if (base.id == dwl::rbd::LZ)
					pos.z = base.position;
			}

			Ogre::Vector3 xpos = transform * pos;
			base_manual_object_->position(xpos.x, xpos.y, xpos.z);
			base_manual_object_->colour(base_color);
		}

		base_manual_object_->end();
		break;

	case BILLBOARDS:
		// Getting the base line width
		float base_line_width = base_line_width_property_->getFloat();
		base_billboard_line_.reset(new rviz::BillboardLine(scene_manager_, scene_node_));
		base_billboard_line_->setNumLines(1);
		base_billboard_line_->setMaxPointsPerLine(num_points);
		base_billboard_line_->setLineWidth(base_line_width);

		for (uint32_t i = 0; i < num_points; ++i) {
			Ogre::Vector3 pos(0., 0., 0.);
			for (uint32_t j = 0; j < num_base; j++) {
				dwl_msgs::BaseState base = msg->trajectory[i].base[j];
				if (base.id == dwl::rbd::LX)
					pos.x = base.position;
				else if (base.id == dwl::rbd::LY)
					pos.y = base.position;
				else if (base.id == dwl::rbd::LZ)
					pos.z = base.position;
			}

			Ogre::Vector3 xpos = transform * pos;
			base_billboard_line_->addPoint(xpos, base_color);
		}
		break;
	}


	// Visualization of the edn-effector trajectory
	// Getting the end-effector trajectory style
	LineStyle contact_style = (LineStyle) contact_style_property_->getOptionInt();

	// Getting the end-effector trajectory color
	Ogre::ColourValue contact_color = contact_color_property_->getOgreColor();
	contact_color.a = contact_alpha_property_->getFloat();

	// Visualization of the contact trajectory
	uint32_t num_contact = msg->trajectory[0].contacts.size();
	switch (contact_style)
	{
	case LINES:
		contact_manual_object_.clear();
		contact_manual_object_.resize(num_contact);
		for (uint32_t j = 0; j < num_contact; j++) {
			contact_manual_object_[j].reset(scene_manager_->createManualObject());
			contact_manual_object_[j]->setDynamic(true);
			scene_node_->attachObject(contact_manual_object_[j].get());

			contact_manual_object_[j]->estimateVertexCount(num_points);
			contact_manual_object_[j]->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);
			for (uint32_t i = 0; i < num_points; ++i) {
				dwl_msgs::ContactState contact = msg->trajectory[i].contacts[j];

				Ogre::Vector3 xpos = transform * Ogre::Vector3(contact.position.x,
															   contact.position.y,
															   contact.position.z);
				contact_manual_object_[j]->position(xpos.x, xpos.y, xpos.z);
				contact_manual_object_[j]->colour(base_color);
			}
			contact_manual_object_[j]->end();
		}
		break;

	case BILLBOARDS:
		// Getting the end-effector line width
		float contact_line_width = contact_line_width_property_->getFloat();

		contact_billboard_line_.clear();
		contact_billboard_line_.resize(num_contact);
		for (uint32_t j = 0; j < num_contact; j++) {
			contact_billboard_line_[j].reset(new rviz::BillboardLine(scene_manager_, scene_node_));
			contact_billboard_line_[j]->setNumLines(1);
			contact_billboard_line_[j]->setMaxPointsPerLine(num_points);
			contact_billboard_line_[j]->setLineWidth(contact_line_width);

			for (uint32_t i = 0; i < num_points; ++i) {
				dwl_msgs::ContactState contact = msg->trajectory[i].contacts[j];

				Ogre::Vector3 xpos = transform * Ogre::Vector3(contact.position.x,
															   contact.position.y,
															   contact.position.z);

				contact_billboard_line_[j]->addPoint(xpos, base_color);
			}
		}
		break;
	}
}

} // namespace dwl_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(dwl_rviz_plugin::WholeBodyTrajectoryDisplay, rviz::Display)
