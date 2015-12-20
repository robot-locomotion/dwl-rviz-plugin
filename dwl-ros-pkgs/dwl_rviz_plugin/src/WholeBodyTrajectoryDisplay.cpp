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

	base_color_property_ = new ColorProperty("Color", QColor(0, 85, 255),
											 "Color to draw the path.",
											 base_category_, NULL, this);

	base_alpha_property_ =
			new FloatProperty("Alpha", 1.0,
							  "Amount of transparency to apply to the path.",
							  base_category_, NULL, this);
}


WholeBodyTrajectoryDisplay::~WholeBodyTrajectoryDisplay()
{

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

	if (style == BILLBOARDS) {
		base_billboard_line_->setLineWidth(line_width);
	}
	context_->queueRender();
}


void WholeBodyTrajectoryDisplay::processMessage(const dwl_msgs::WholeBodyTrajectory::ConstPtr& msg)
{
	LineStyle style = (LineStyle) base_style_property_->getOptionInt();
	// Lookup transform into fixed frame
	Ogre::Vector3 position;
	Ogre::Quaternion orientation;
	if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
		ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
				  msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
	}

	Ogre::Matrix4 transform(orientation);
	transform.setTrans(position);


	Ogre::ColourValue color = base_color_property_->getOgreColor();
	color.a = base_alpha_property_->getFloat();

	uint32_t num_points = msg->trajectory.size();
	float line_width = base_line_width_property_->getFloat();

	switch (style)
	{
	case LINES:
		base_manual_object_.reset(scene_manager_->createManualObject());
		base_manual_object_->setDynamic(true);
		scene_node_->attachObject(base_manual_object_.get());

		base_manual_object_->estimateVertexCount(num_points);
		base_manual_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);
		for (uint32_t i = 0; i < num_points; ++i) {
			unsigned int num_base = msg->trajectory[i].base.size();
			Eigen::Vector3d pos = Eigen::Vector3d::Zero();
			for (unsigned int j = 0; j < num_base; j++) {
				if (msg->trajectory[i].base[j].id == dwl::rbd::LX)
					pos(dwl::rbd::X) = msg->trajectory[i].base[j].position;
				else if (msg->trajectory[i].base[j].id == dwl::rbd::LY)
					pos(dwl::rbd::Y) = msg->trajectory[i].base[j].position;
				else if (msg->trajectory[i].base[j].id == dwl::rbd::LZ)
					pos(dwl::rbd::Z) = msg->trajectory[i].base[j].position;
			}

			Ogre::Vector3 xpos = transform * Ogre::Vector3(pos(dwl::rbd::X),
														   pos(dwl::rbd::Y),
														   pos(dwl::rbd::Z));
			base_manual_object_->position(xpos.x, xpos.y, xpos.z);
			base_manual_object_->colour(color);
		}

		base_manual_object_->end();
		break;

	case BILLBOARDS:
		base_billboard_line_.reset(new rviz::BillboardLine(scene_manager_, scene_node_));
		base_billboard_line_->setNumLines(1);
		base_billboard_line_->setMaxPointsPerLine(num_points);
		base_billboard_line_->setLineWidth(line_width);

		for (uint32_t i=0; i < num_points; ++i) {
			unsigned int num_base = msg->trajectory[i].base.size();
			Eigen::Vector3d pos = Eigen::Vector3d::Zero();
			for (unsigned int j = 0; j < num_base; j++) {
				if (msg->trajectory[i].base[j].id == dwl::rbd::LX)
					pos(dwl::rbd::X) = msg->trajectory[i].base[j].position;
				else if (msg->trajectory[i].base[j].id == dwl::rbd::LY)
					pos(dwl::rbd::Y) = msg->trajectory[i].base[j].position;
				else if (msg->trajectory[i].base[j].id == dwl::rbd::LZ)
					pos(dwl::rbd::Z) = msg->trajectory[i].base[j].position;
			}

			Ogre::Vector3 xpos = transform * Ogre::Vector3(pos(dwl::rbd::X),
														   pos(dwl::rbd::Y),
														   pos(dwl::rbd::Z));
			base_billboard_line_->addPoint(xpos, color);
		}
		break;
	}
}

} // namespace dwl_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(dwl_rviz_plugin::WholeBodyTrajectoryDisplay, rviz::Display)
