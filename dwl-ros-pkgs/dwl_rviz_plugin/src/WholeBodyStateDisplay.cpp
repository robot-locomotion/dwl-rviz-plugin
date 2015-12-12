#include <dwl_rviz_plugin/WholeBodyStateDisplay.h>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>
#include <rviz/validate_floats.h>


using namespace rviz;

namespace dwl_rviz_plugin
{

WholeBodyStateDisplay::WholeBodyStateDisplay()
{
	color_property_ =
			new rviz::ColorProperty("Color", QColor(204, 41, 204),
									"Color of a point",
									this, SLOT(updateColorAndAlpha()));

	alpha_property_ =
			new rviz::FloatProperty("Alpha", 1.0,
									"0 is fully transparent, 1.0 is fully opaque.",
									this, SLOT(updateColorAndAlpha()));

	radius_property_ =
			new rviz::FloatProperty("Radius", 0.04,
									"Radius of a point",
									this, SLOT(updateColorAndAlpha()));



	robot_model_property_ = new StringProperty("Robot Description", "robot_model",
												"Name of the parameter to search for to load"
												" the robot description.",
												this, SLOT(updateRobotDescription()));
}


WholeBodyStateDisplay::~WholeBodyStateDisplay()
{

}


void WholeBodyStateDisplay::clear()
{
//	robot_->clear();
	clearStatuses();
	robot_model_.clear();
}


void WholeBodyStateDisplay::onInitialize()
{
	MFDClass::onInitialize();
}


void WholeBodyStateDisplay::reset()
{
	MFDClass::reset();
}


void WholeBodyStateDisplay::load()
{
	std::string content;
	if (!update_nh_.getParam(robot_model_property_->getStdString(), content)) {
		std::string loc;
		if (update_nh_.searchParam(robot_model_property_->getStdString(), loc)) {
			update_nh_.getParam(loc, content);
		} else {
			clear();
			setStatus(StatusProperty::Error, "URDF",
					  "Parameter [" + robot_model_property_->getString()
					  + "] does not exist, and was not found by searchParam()");
			return;
		}
	}

	if (content.empty()) {
		clear();
		setStatus(StatusProperty::Error, "URDF", "URDF is empty");
		return;
	}

	if (content == robot_model_) {
		return;
	}

	robot_model_ = content;

	// Initializing the dynamics from the URDF model
	dynamics_.modelFromURDFModel(robot_model_);

//	TiXmlDocument doc;
//	doc.Parse(robot_model_.c_str());
//	if (!doc.RootElement()) {
//		clear();
//		setStatus(StatusProperty::Error, "URDF", "URDF failed XML parse");
//		return;
//	}

//	urdf::Model descr;
//	if (!descr.initXml(doc.RootElement())) {
//		clear();
//		setStatus(StatusProperty::Error, "URDF", "URDF failed Model parse");
//		return;
//	}

	setStatus(StatusProperty::Ok, "URDF", "URDF parsed OK");
//	robot_->load( descr );
//	robot_->update( TFLinkUpdater(context_->getFrameManager(),
//								  boost::bind( linkUpdaterStatusFunction, _1, _2, _3, this ),
//								  tf_prefix_property_->getStdString() ));
}



void WholeBodyStateDisplay::updateColorAndAlpha()
{
	float alpha = alpha_property_->getFloat();
	float radius = radius_property_->getFloat();
	Ogre::ColourValue color = color_property_->getOgreColor();

	visual_->setColor(color.r, color.g, color.b, alpha);
	visual_->setRadius(radius);
}


void WholeBodyStateDisplay::updateRobotDescription()
{
	if (isEnabled()) {
		load();
		context_->queueRender();
	}
}


void WholeBodyStateDisplay::processMessage(const dwl_msgs::WholeBodyState::ConstPtr& msg)
{
	// Getting the contact wrenches and positions
	dwl::rbd::BodySelector contact_names;
	dwl::rbd::BodyVector contact_pos;
	dwl::rbd::BodyWrench contact_for;
	unsigned int num_contacts = msg->contacts.size();
	for (unsigned int i = 0; i < num_contacts; i++) {
		dwl_msgs::ContactState contact = msg->contacts[i];

		// Getting the name
		std::string name = contact.name;
		contact_names.push_back(name);

		// Getting the contact position
		Eigen::VectorXd position = Eigen::VectorXd::Zero(3);
		position << contact.position.x, contact.position.y, contact.position.z;
		contact_pos[name] = position;

		// Getting the contact wrench
		dwl::rbd::Vector6d wrench;
		wrench(dwl::rbd::AX) = contact.wrench.torque.x;
		wrench(dwl::rbd::AY) = contact.wrench.torque.y;
		wrench(dwl::rbd::AZ) = contact.wrench.torque.z;
		wrench(dwl::rbd::LX) = contact.wrench.force.x;
		wrench(dwl::rbd::LY) = contact.wrench.force.y;
		wrench(dwl::rbd::LZ) = contact.wrench.force.z;
		contact_for[name] = wrench;
	}

	// Computing the center of pressure position
	Eigen::Vector3d com_pos;
	dynamics_.computeCenterOfPressure(com_pos, contact_for, contact_pos, contact_names);

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

	visual_.reset(new PointVisual(context_->getSceneManager(), scene_node_));

	// Defining the center of mass as Ogre::Vector3
	Ogre::Vector3 com_point;
	com_point.x = com_pos(dwl::rbd::X);
	com_point.y = com_pos(dwl::rbd::Y);
	com_point.z = com_pos(dwl::rbd::Z);

	// Now set or update the contents of the chosen visual.
	updateColorAndAlpha();
	visual_->setPoint(com_point);
	visual_->setFramePosition(position);
	visual_->setFrameOrientation(orientation);
}

} //@namespace dwl_rviz_plugin


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(dwl_rviz_plugin::WholeBodyStateDisplay, rviz::Display)
