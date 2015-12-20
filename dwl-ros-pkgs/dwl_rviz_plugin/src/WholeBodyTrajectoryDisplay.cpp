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
			new FloatProperty("Line Width", 0.03,
							  "The width, in meters, of each path line. "
							  "Only works with the 'Billboards' style.",
							  base_category_, SLOT(updateBaseLineWidth()), this);
	base_line_width_property_->setMin(0.001);
	base_line_width_property_->hide();

	base_color_property_ = new ColorProperty("Color", QColor(25, 255, 0),
											 "Color to draw the path.",
											 base_category_, NULL, this);

	base_alpha_property_ =
			new FloatProperty("Alpha", 1.0,
							  "Amount of transparency to apply to the path.",
							  base_category_, NULL, this);

	base_buffer_length_property_ =
			new IntProperty("Buffer Length", 1,
							"Number of paths to display.",
							base_category_, SLOT(updateBaseBufferLength()), this);
	base_buffer_length_property_->setMin(1);

	base_offset_property_ =
			new VectorProperty("Offset", Ogre::Vector3::ZERO,
							   "Allows you to offset the path from the origin of the "
							   "reference frame.  In meters.",
							   base_category_, SLOT(updateBaseOffset()), this);
}


WholeBodyTrajectoryDisplay::~WholeBodyTrajectoryDisplay()
{
	destroyObjects();
}


void WholeBodyTrajectoryDisplay::onInitialize()
{
	MFDClass::onInitialize();
	updateBaseBufferLength();
}


void WholeBodyTrajectoryDisplay::reset()
{
	MFDClass::reset();
	updateBaseBufferLength();
}


void WholeBodyTrajectoryDisplay::updateBaseBufferLength()
{
	// Delete old path objects
	destroyObjects();

	// Read options
	int buffer_length = base_buffer_length_property_->getInt();
	LineStyle style = (LineStyle) base_style_property_->getOptionInt();

	// Create new path objects
	switch (style)
	{
	case LINES: // simple lines with fixed width of 1px
		base_manual_objects_.resize(buffer_length);
		for (size_t i = 0; i < base_manual_objects_.size(); i++) {
			Ogre::ManualObject* manual_object = scene_manager_->createManualObject();
			manual_object->setDynamic(true);
			scene_node_->attachObject(manual_object);

			base_manual_objects_[i] = manual_object;
		}
		break;

	case BILLBOARDS: // billboards with configurable width
		base_billboard_lines_.resize(buffer_length);
		for (size_t i = 0; i < base_billboard_lines_.size(); i++) {
			rviz::BillboardLine* billboard_line =
					new rviz::BillboardLine(scene_manager_, scene_node_);

			base_billboard_lines_[i] = billboard_line;
		}
		break;
	}
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

	updateBaseBufferLength();
}


void WholeBodyTrajectoryDisplay::updateBaseLineWidth()
{
	LineStyle style = (LineStyle) base_style_property_->getOptionInt();
	float line_width = base_line_width_property_->getFloat();

	if (style == BILLBOARDS) {
		for (size_t i = 0; i < base_billboard_lines_.size(); i++) {
			rviz::BillboardLine* billboard_line = base_billboard_lines_[i];
			if (billboard_line)
				billboard_line->setLineWidth(line_width);
		}
	}
	context_->queueRender();
}


void WholeBodyTrajectoryDisplay::updateBaseOffset()
{
	scene_node_->setPosition(base_offset_property_->getVector());
	context_->queueRender();
}


void WholeBodyTrajectoryDisplay::processMessage(const dwl_msgs::WholeBodyTrajectory::ConstPtr& msg)
{
	// Calculate index of oldest element in cyclic buffer
	size_t buffer_idx = messages_received_ % base_buffer_length_property_->getInt();

	LineStyle style = (LineStyle) base_style_property_->getOptionInt();
	Ogre::ManualObject* manual_object = NULL;
	rviz::BillboardLine* billboard_line = NULL;

	// Delete oldest element
	switch (style)
	{
	case LINES:
		manual_object = base_manual_objects_[buffer_idx];
		manual_object->clear();
		break;

	case BILLBOARDS:
		billboard_line = base_billboard_lines_[buffer_idx];
		billboard_line->clear();
		break;
	}

	// Lookup transform into fixed frame
	Ogre::Vector3 position;
	Ogre::Quaternion orientation;
	if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
		ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
				  msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
	}

	Ogre::Matrix4 transform(orientation);
	transform.setTrans(position);

//  scene_node_->setPosition( position );
//  scene_node_->setOrientation( orientation );

	Ogre::ColourValue color = base_color_property_->getOgreColor();
	color.a = base_alpha_property_->getFloat();

	uint32_t num_points = msg->trajectory.size();
	float line_width = base_line_width_property_->getFloat();

	switch (style)
	{
	case LINES:
		manual_object->estimateVertexCount(num_points);
		manual_object->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);
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
			manual_object->position(xpos.x, xpos.y, xpos.z);
			manual_object->colour(color);
		}

		manual_object->end();
		break;

	case BILLBOARDS:
		billboard_line->setNumLines(1);
		billboard_line->setMaxPointsPerLine(num_points);
		billboard_line->setLineWidth(line_width);

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
			billboard_line->addPoint(xpos, color);
		}
		break;
	}
}


void WholeBodyTrajectoryDisplay::destroyObjects()
{
	// Destroy all simple lines, if any
	for (size_t i = 0; i < base_manual_objects_.size(); i++) {
		Ogre::ManualObject*& manual_object = base_manual_objects_[i];
		if (manual_object) {
			manual_object->clear();
			scene_manager_->destroyManualObject(manual_object);
			manual_object = NULL; // ensure it doesn't get destroyed again
		}
	}

	// Destroy all billboards, if any
	for (size_t i = 0; i < base_billboard_lines_.size(); i++) {
		rviz::BillboardLine*& billboard_line = base_billboard_lines_[i];
		if (billboard_line) {
			delete billboard_line; // also destroys the corresponding scene node
			billboard_line = NULL; // ensure it doesn't get destroyed again
		}
	}
}

} // namespace dwl_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(dwl_rviz_plugin::WholeBodyTrajectoryDisplay, rviz::Display)
