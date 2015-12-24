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
	destroyObjects();
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

	// Destroy all the old elements
	destroyObjects();

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


	// Visualization of the end-effector trajectory
	// Getting the end-effector trajectory style
	LineStyle contact_style = (LineStyle) contact_style_property_->getOptionInt();

	// Getting the end-effector trajectory color
	Ogre::ColourValue contact_color = contact_color_property_->getOgreColor();
	contact_color.a = contact_alpha_property_->getFloat();

	// Getting the number of contact trajectories
	uint32_t num_traj = 0;
	std::map<std::string, uint32_t> contact_traj_id;
	for (uint32_t i = 0; i < num_points; ++i) {
		uint32_t num_contacts = msg->trajectory[i].contacts.size();
		for (uint32_t k = 0; k < num_contacts; k++) {
			dwl_msgs::ContactState contact = msg->trajectory[i].contacts[k];

			if (contact_traj_id.find(contact.name) == contact_traj_id.end()) {// a new swing trajectory
				contact_traj_id[contact.name] = num_traj;

				// Incrementing the counter (id) of swing trajectories
				num_traj++;
			}
		}
	}

	// Visualizing the different end-effector trajectories
	contact_traj_id.clear();
	std::map<uint32_t, uint32_t> contact_vec_id;
	switch (contact_style)
	{
	case LINES: {
		contact_manual_object_.clear();
		contact_manual_object_.resize(num_traj);

		uint32_t traj_id = 0;
		for (uint32_t i = 0; i < num_points; ++i) {
			uint32_t num_contacts = msg->trajectory[i].contacts.size();
			for (uint32_t k = 0; k < num_contacts; k++) {
				dwl_msgs::ContactState contact = msg->trajectory[i].contacts[k];

				if (contact_traj_id.find(contact.name) == contact_traj_id.end()) {// a new swing trajectory
					contact_traj_id[contact.name] = traj_id;
					contact_vec_id[traj_id] = k;

					contact_manual_object_[traj_id].reset(scene_manager_->createManualObject());
					contact_manual_object_[traj_id]->setDynamic(true);
					scene_node_->attachObject(contact_manual_object_[traj_id].get());

					contact_manual_object_[traj_id]->estimateVertexCount(num_points);
					contact_manual_object_[traj_id]->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);

					// Incrementing the counter (id) of swing trajectories
					traj_id++;
				} else {
					uint32_t swing_idx = contact_traj_id.find(contact.name)->second;

					if (k != contact_vec_id.find(swing_idx)->second) {// change the vector index
						contact_vec_id[swing_idx] = k;
					}
				}
			}

			// Computing the actual base position
			Ogre::Vector3 base_pos(0., 0., 0.);
			for (uint32_t j = 0; j < num_base; j++) {
				dwl_msgs::BaseState base = msg->trajectory[i].base[j];
				if (base.id == dwl::rbd::LX)
					base_pos.x = base.position;
				else if (base.id == dwl::rbd::LY)
					base_pos.y = base.position;
				else if (base.id == dwl::rbd::LZ)
					base_pos.z = base.position;
			}

			// Adding the contact points for the current swing trajectories
			for (std::map<std::string,uint32_t>::iterator traj_it = contact_traj_id.begin();
					traj_it != contact_traj_id.end(); traj_it++) {
				uint32_t traj_id = traj_it->second;
				uint32_t id = contact_vec_id.find(traj_id)->second;

				if (id < num_contacts) {
					dwl_msgs::ContactState contact = msg->trajectory[i].contacts[id];

					Ogre::Vector3 xpos = base_pos + transform * Ogre::Vector3(contact.position.x,
																			  contact.position.y,
																			  contact.position.z);

					contact_manual_object_[traj_id]->position(xpos.x, xpos.y, xpos.z);
					contact_manual_object_[traj_id]->colour(contact_color);
				}
			}
		}
		// Ending the contact manual objects
		for (uint32_t i = 0; i < traj_id; i++)
			contact_manual_object_[i]->end();

		break;}

	case BILLBOARDS: {
		// Getting the end-effector line width
		float contact_line_width = contact_line_width_property_->getFloat();
		contact_billboard_line_.clear();
		contact_billboard_line_.resize(num_traj);

		uint32_t traj_id = 0;
		for (uint32_t i = 0; i < num_points; ++i) {
			uint32_t num_contacts = msg->trajectory[i].contacts.size();
			for (uint32_t k = 0; k < num_contacts; k++) {
				dwl_msgs::ContactState contact = msg->trajectory[i].contacts[k];

				if (contact_traj_id.find(contact.name) == contact_traj_id.end()) {// a new swing trajectory
					contact_traj_id[contact.name] = traj_id;
					contact_vec_id[traj_id] = k;

					contact_billboard_line_[traj_id].reset(new rviz::BillboardLine(scene_manager_, scene_node_));
					contact_billboard_line_[traj_id]->setNumLines(1);
					contact_billboard_line_[traj_id]->setMaxPointsPerLine(num_points);
					contact_billboard_line_[traj_id]->setLineWidth(contact_line_width);

					// Incrementing the counter (id) of swing trajectories
					traj_id++;
				} else {
					uint32_t swing_idx = contact_traj_id.find(contact.name)->second;

					if (k != contact_vec_id.find(swing_idx)->second) {// change the vector index
						contact_vec_id[swing_idx] = k;
					}
				}
			}

			// Computing the actual base position
			Ogre::Vector3 base_pos(0., 0., 0.);
			for (uint32_t j = 0; j < num_base; j++) {
				dwl_msgs::BaseState base = msg->trajectory[i].base[j];
				if (base.id == dwl::rbd::LX)
					base_pos.x = base.position;
				else if (base.id == dwl::rbd::LY)
					base_pos.y = base.position;
				else if (base.id == dwl::rbd::LZ)
					base_pos.z = base.position;
			}

			// Adding the contact points for the current swing trajectories
			for (std::map<std::string,uint32_t>::iterator traj_it = contact_traj_id.begin();
					traj_it != contact_traj_id.end(); traj_it++) {
				uint32_t traj_id = traj_it->second;
				uint32_t id = contact_vec_id.find(traj_id)->second;

				if (id < num_contacts) {
					dwl_msgs::ContactState contact = msg->trajectory[i].contacts[id];

					Ogre::Vector3 xpos = base_pos + transform * Ogre::Vector3(contact.position.x,
																			  contact.position.y,
																			  contact.position.z);

					contact_billboard_line_[traj_id]->addPoint(xpos, contact_color);
				}
			}
		}
		break;}
	}
}


void WholeBodyTrajectoryDisplay::destroyObjects()
{
	base_manual_object_.reset();
	base_billboard_line_.reset();
	contact_manual_object_.clear();
	contact_billboard_line_.clear();
}

} // namespace dwl_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(dwl_rviz_plugin::WholeBodyTrajectoryDisplay, rviz::Display)
