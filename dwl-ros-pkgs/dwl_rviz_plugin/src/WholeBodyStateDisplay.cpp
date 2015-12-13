#include <dwl_rviz_plugin/WholeBodyStateDisplay.h>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <rviz/frame_manager.h>
#include <rviz/validate_floats.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>


using namespace rviz;

namespace dwl_rviz_plugin
{

WholeBodyStateDisplay::WholeBodyStateDisplay()
{
	// Category Groups
	cop_category_  = new rviz::Property("Center Of Pressure", QVariant(), "", this);
	grf_category_  = new rviz::Property("Contact Forces", QVariant(), "", this);

	robot_model_property_ = new StringProperty("Robot Description", "robot_model",
												"Name of the parameter to search for to load"
												" the robot description.",
												this, SLOT(updateRobotModel()));

	cop_color_property_ =
			new rviz::ColorProperty("Color", QColor(204, 41, 204),
									"Color of a point",
									cop_category_, SLOT(updateCoPColorAndAlpha()), this);

	cop_alpha_property_ =
			new rviz::FloatProperty("Alpha", 1.0,
									"0 is fully transparent, 1.0 is fully opaque.",
									cop_category_, SLOT(updateCoPColorAndAlpha()), this);
	cop_alpha_property_->setMin(0);
	cop_alpha_property_->setMax(1);

	cop_radius_property_ =
			new rviz::FloatProperty("Radius", 0.04,
									"Radius of a point",
									cop_category_, SLOT(updateCoPColorAndAlpha()), this);


	grf_color_property_ =
			new ColorProperty("Color", QColor(255, 25, 0),
							  "Color to draw the arrow.",
							  grf_category_, SLOT(updateGRFColorAndAlpha()), this);

	grf_alpha_property_ =
			new FloatProperty("Alpha", 1.0,
							  "Amount of transparency to apply to the arrow.",
							  grf_category_, SLOT(updateGRFColorAndAlpha()), this);
	grf_alpha_property_->setMin(0);
	grf_alpha_property_->setMax(1);

	grf_shaft_length_property_ =
			new FloatProperty("Shaft Length", 1.0,
							  "Length of the arrow's shaft, in meters.",
							  grf_category_, SLOT(updateGRFArrowGeometry()), this);

	// aleeper: default changed from 0.1 to match change in arrow.cpp
	grf_shaft_radius_property_ =
			new FloatProperty("Shaft Radius", 0.05,
							  "Radius of the arrow's shaft, in meters.",
							  grf_category_, SLOT(updateGRFArrowGeometry()), this);

	grf_head_length_property_ =
			new FloatProperty("Head Length", 0.3,
							  "Length of the arrow's head, in meters.",
							  grf_category_, SLOT(updateGRFArrowGeometry()), this);

	// aleeper: default changed from 0.2 to match change in arrow.cpp
	grf_head_radius_property_ =
			new FloatProperty("Head Radius", 0.1,
							  "Radius of the arrow's head, in meters.",
							  grf_category_, SLOT(updateGRFArrowGeometry()), this);
}


WholeBodyStateDisplay::~WholeBodyStateDisplay()
{
	if (initialized()) {
		delete arrow_;
	}
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

	arrow_ = new rviz::Arrow(scene_manager_, scene_node_,
							 grf_shaft_length_property_->getFloat(),
							 grf_shaft_radius_property_->getFloat(),
							 grf_head_length_property_->getFloat(),
							 grf_head_radius_property_->getFloat());
	// Arrow points in -Z direction, so rotate the orientation before display.
	// TODO: is it safe to change Arrow to point in +X direction?
	arrow_->setOrientation( Ogre::Quaternion( Ogre::Degree( -90 ), Ogre::Vector3::UNIT_Y ));

	updateGRFColorAndAlpha();
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


void WholeBodyStateDisplay::updateRobotModel()
{
	if (isEnabled()) {
		load();
		context_->queueRender();
	}
}


void WholeBodyStateDisplay::updateCoPColorAndAlpha()
{
	float alpha = cop_alpha_property_->getFloat();
	float radius = cop_radius_property_->getFloat();
	Ogre::ColourValue color = cop_color_property_->getOgreColor();

	visual_->setColor(color.r, color.g, color.b, alpha);
	visual_->setRadius(radius);
}


void WholeBodyStateDisplay::updateGRFColorAndAlpha()
{
	Ogre::ColourValue color = grf_color_property_->getOgreColor();
	color.a = grf_alpha_property_->getFloat();

	arrow_->setColor(color);

	context_->queueRender();
}


void WholeBodyStateDisplay::updateGRFArrowGeometry()
{
	arrow_->set(grf_shaft_length_property_->getFloat(),
				grf_shaft_radius_property_->getFloat(),
				grf_head_length_property_->getFloat(),
				grf_head_radius_property_->getFloat());
	context_->queueRender();
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
	Eigen::Vector3d cop_pos;
	dynamics_.computeCenterOfPressure(cop_pos, contact_for, contact_pos, contact_names);

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
	Ogre::Vector3 cop_point;
	cop_point.x = cop_pos(dwl::rbd::X);
	cop_point.y = cop_pos(dwl::rbd::Y);
	cop_point.z = cop_pos(dwl::rbd::Z);

	// Now set or update the contents of the chosen visual.
	updateCoPColorAndAlpha();
	visual_->setPoint(cop_point);
	visual_->setFramePosition(position);
	visual_->setFrameOrientation(orientation);
}

} //@namespace dwl_rviz_plugin


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(dwl_rviz_plugin::WholeBodyStateDisplay, rviz::Display)
