#include <dwl_rviz_plugin/WholeBodyStateDisplay.h>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <rviz/frame_manager.h>
#include <rviz/validate_floats.h>
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
	com_category_ = new rviz::Property("Center Of Mass", QVariant(), "", this);
	cop_category_ = new rviz::Property("Center Of Pressure", QVariant(), "", this);
	grf_category_ = new rviz::Property("Contact Forces", QVariant(), "", this);

	// Robot properties
	robot_model_property_ = new StringProperty("Robot Description", "robot_model",
												"Name of the parameter to search for to load"
												" the robot description.",
												this, SLOT(updateRobotModel()));

	// CoM properties
	com_color_property_ =
			new rviz::ColorProperty("Color", QColor(255, 85, 0),
									"Color of a point",
									com_category_, SLOT(updateCoMColorAndAlpha()), this);

	com_alpha_property_ =
			new rviz::FloatProperty("Alpha", 1.0,
									"0 is fully transparent, 1.0 is fully opaque.",
									com_category_, SLOT(updateCoMColorAndAlpha()), this);
	com_alpha_property_->setMin(0);
	com_alpha_property_->setMax(1);

	com_radius_property_ =
			new rviz::FloatProperty("Radius", 0.04,
									"Radius of a point",
									com_category_, SLOT(updateCoMColorAndAlpha()), this);

	// CoP properties
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

	// GRF properties
	grf_color_property_ =
			new ColorProperty("Color", QColor(85, 0, 255),
							  "Color to draw the arrow.",
							  grf_category_, SLOT(updateGRFColorAndAlpha()), this);

	grf_alpha_property_ =
			new FloatProperty("Alpha", 1.0,
							  "Amount of transparency to apply to the arrow.",
							  grf_category_, SLOT(updateGRFColorAndAlpha()), this);
	grf_alpha_property_->setMin(0);
	grf_alpha_property_->setMax(1);

	grf_shaft_length_property_ =
			new FloatProperty("Shaft Length", 0.4,
							  "Length of the arrow's shaft, in meters.",
							  grf_category_, SLOT(updateGRFArrowGeometry()), this);

	grf_shaft_radius_property_ =
			new FloatProperty("Shaft Radius", 0.02,
							  "Radius of the arrow's shaft, in meters.",
							  grf_category_, SLOT(updateGRFArrowGeometry()), this);

	grf_head_length_property_ =
			new FloatProperty("Head Length", 0.08,
							  "Length of the arrow's head, in meters.",
							  grf_category_, SLOT(updateGRFArrowGeometry()), this);

	grf_head_radius_property_ =
			new FloatProperty("Head Radius", 0.04,
							  "Radius of the arrow's head, in meters.",
							  grf_category_, SLOT(updateGRFArrowGeometry()), this);
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

	updateGRFColorAndAlpha();
}


void WholeBodyStateDisplay::reset()
{
	MFDClass::reset();
	grf_visual_.clear();
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


void WholeBodyStateDisplay::updateCoMColorAndAlpha()
{
	float radius = com_radius_property_->getFloat();
	Ogre::ColourValue color = com_color_property_->getOgreColor();
	color.a = com_alpha_property_->getFloat();

	com_visual_->setColor(color.r, color.g, color.b, color.a);
	com_visual_->setRadius(radius);
}


void WholeBodyStateDisplay::updateCoPColorAndAlpha()
{
	float radius = cop_radius_property_->getFloat();
	Ogre::ColourValue color = cop_color_property_->getOgreColor();
	color.a = cop_alpha_property_->getFloat();

	cop_visual_->setColor(color.r, color.g, color.b, color.a);
	cop_visual_->setRadius(radius);
}


void WholeBodyStateDisplay::updateGRFColorAndAlpha()
{
	Ogre::ColourValue color = grf_color_property_->getOgreColor();
	color.a = grf_alpha_property_->getFloat();

	for (size_t i = 0; i < grf_visual_.size(); i++ )
		grf_visual_[i]->setColor(color.r, color.g, color.b, color.a);

	context_->queueRender();
}


void WholeBodyStateDisplay::updateGRFArrowGeometry()
{
	float shaft_length = grf_shaft_length_property_->getFloat();
	float shaft_radius = grf_shaft_radius_property_->getFloat();
	float head_length = grf_head_length_property_->getFloat();
	float head_radius = grf_head_radius_property_->getFloat();

	for (size_t i = 0; i < grf_visual_.size(); i++ )
		grf_visual_[i]->setProperties(shaft_length, shaft_radius, head_length, head_radius);

	context_->queueRender();
}


void WholeBodyStateDisplay::processMessage(const dwl_msgs::WholeBodyState::ConstPtr& msg)
{
	// Getting the joint position
	unsigned int num_joints = msg->joints.size();
	Eigen::VectorXd joint_pos = Eigen::VectorXd::Zero(num_joints);
	for (unsigned int i = 0; i < num_joints; i++) {
		dwl_msgs::JointState joint = msg->joints[i];

		// Getting the joint name
		std::string name  = joint.name;

		// Getting the joint id
		unsigned int id = dynamics_.getFloatingBaseSystem().getJoints().find(name)->second;

		// Setting the joint position
		joint_pos(id) = joint.position;
	}

	// Getting the contact wrenches and positions
	Eigen::Vector3d total_force = Eigen::Vector3d::Zero();
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

		// Computing the total force
		total_force += dwl::rbd::linearPart(wrench);
	}

	// Computing the normalized force per contact which is uses for scaling the arrows
	double norm_force = total_force.norm() / num_contacts;


	// Computing the center of mass
	dwl::rbd::Vector6d null_base_pos = dwl::rbd::Vector6d::Zero();
	Eigen::Vector3d com_pos = dynamics_.getFloatingBaseSystem().getSystemCoM(null_base_pos,
																			 joint_pos);


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

	com_visual_.reset(new PointVisual(context_->getSceneManager(), scene_node_));
	cop_visual_.reset(new PointVisual(context_->getSceneManager(), scene_node_));

	// Defining the center of mass as Ogre::Vector3
	Ogre::Vector3 com_point;
	com_point.x = com_pos(dwl::rbd::X);
	com_point.y = com_pos(dwl::rbd::Y);
	com_point.z = com_pos(dwl::rbd::Z);

	// Now set or update the contents of the chosen CoM visual
	updateCoMColorAndAlpha();
	com_visual_->setPoint(com_point);
	com_visual_->setFramePosition(position);
	com_visual_->setFrameOrientation(orientation);

	// Defining the center of pressure as Ogre::Vector3
	Ogre::Vector3 cop_point;
	cop_point.x = cop_pos(dwl::rbd::X);
	cop_point.y = cop_pos(dwl::rbd::Y);
	cop_point.z = cop_pos(dwl::rbd::Z);

	// Now set or update the contents of the chosen CoP visual
	updateCoPColorAndAlpha();
	cop_visual_->setPoint(cop_point);
	cop_visual_->setFramePosition(position);
	cop_visual_->setFrameOrientation(orientation);

	// Now set or update the contents of the chosen GRF visual
	grf_visual_.clear();
	for (unsigned int i = 0; i < num_contacts; i++) {
		dwl_msgs::ContactState contact = msg->contacts[i];

		// Getting the name
		std::string name = contact.name;

		// Getting the contact position
		Ogre::Vector3 contact_pos(contact.position.x, contact.position.y, contact.position.z);

		// Getting the force direction
		Eigen::Vector3d ref_dir = -Eigen::Vector3d::UnitZ();
		Eigen::Vector3d for_dir;
		for_dir << contact.wrench.force.x, contact.wrench.force.y, contact.wrench.force.z;
		Eigen::Quaterniond q;
		q.setFromTwoVectors(ref_dir, for_dir);
		Ogre::Quaternion contact_for_orientation(q.w(), q.x(), q.y(), q.z());

		// We are keeping a vector of visual pointers. This creates the next one and stores it
		// in the vector
		boost::shared_ptr<ArrowVisual> arrow;
		arrow.reset(new ArrowVisual(context_->getSceneManager(), scene_node_));
		arrow->setArrow(contact_pos, contact_for_orientation);
		arrow->setFramePosition(position);
		arrow->setFrameOrientation(orientation);

		// Setting the arrow color and properties
		Ogre::ColourValue color = grf_color_property_->getOgreColor();
		color.a = grf_alpha_property_->getFloat();
		arrow->setColor(color.r, color.g, color.b, color.a);
		float shaft_length = grf_shaft_length_property_->getFloat() * for_dir.norm() / norm_force;
		float shaft_radius = grf_shaft_radius_property_->getFloat();
		float head_length = grf_head_length_property_->getFloat();
		float head_radius = grf_head_radius_property_->getFloat();
		arrow->setProperties(shaft_length, shaft_radius, head_length, head_radius);

		// And send it to the end of the vector
		grf_visual_.push_back(arrow);
	}
}

} //@namespace dwl_rviz_plugin


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(dwl_rviz_plugin::WholeBodyStateDisplay, rviz::Display)
