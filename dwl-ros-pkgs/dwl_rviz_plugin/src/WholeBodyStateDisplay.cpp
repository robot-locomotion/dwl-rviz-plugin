#include <dwl_rviz_plugin/WholeBodyStateDisplay.h>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <rviz/frame_manager.h>
#include <rviz/validate_floats.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>


using namespace rviz;

namespace dwl_rviz_plugin
{

WholeBodyStateDisplay::WholeBodyStateDisplay() : is_info_(false),
		initialized_model_(false), force_threshold_(0.), weight_(0.),
		com_real_(true)
{
	// Robot properties
	robot_model_property_ = new StringProperty("Robot Description", "robot_model",
												"Name of the parameter to search for to load"
												" the robot description.",
												this, SLOT(updateRobotModel()));

	// Category Groups
	com_category_ = new rviz::Property("Center Of Mass", QVariant(), "", this);
	cop_category_ = new rviz::Property("Center Of Pressure", QVariant(), "", this);
	cmp_category_ = new rviz::Property("Centroidal Momentum Pivot", QVariant(), "", this);
	inst_cp_category_ = new rviz::Property("Instantaneous Capture Point", QVariant(), "", this);
	grf_category_ = new rviz::Property("Contact Forces", QVariant(), "", this);
	support_category_ = new rviz::Property("Support Region", QVariant(), "", this);


	// CoM position and velocity properties
	com_style_property_ = new EnumProperty("CoM Style", "Real",
											"The rendering operation to use to draw the CoM.",
											com_category_, SLOT(updateCoMStyle()), this);
	com_style_property_->addOption("Real", REAL);
	com_style_property_->addOption("Projected", PROJECTED);

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

	com_shaft_length_property_ =
			new FloatProperty("Shaft Length", 0.4,
							  "Length of the arrow's shaft, in meters.",
							  com_category_, SLOT(updateCoMArrowGeometry()), this);

	com_shaft_radius_property_ =
			new FloatProperty("Shaft Radius", 0.02,
							  "Radius of the arrow's shaft, in meters.",
							  com_category_, SLOT(updateCoMArrowGeometry()), this);

	com_head_length_property_ =
			new FloatProperty("Head Length", 0.08,
							  "Length of the arrow's head, in meters.",
							  com_category_, SLOT(updateCoMArrowGeometry()), this);

	com_head_radius_property_ =
			new FloatProperty("Head Radius", 0.04,
							  "Radius of the arrow's head, in meters.",
							  com_category_, SLOT(updateCoMArrowGeometry()), this);


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

	// CMP properties
	cmp_color_property_ =
			new rviz::ColorProperty("Color", QColor(200, 41, 10),
									"Color of a point",
									cmp_category_, SLOT(updateCoPColorAndAlpha()), this);

	cmp_alpha_property_ =
			new rviz::FloatProperty("Alpha", 1.0,
									"0 is fully transparent, 1.0 is fully opaque.",
									cmp_category_, SLOT(updateCoPColorAndAlpha()), this);
	cmp_alpha_property_->setMin(0);
	cmp_alpha_property_->setMax(1);

	cmp_radius_property_ =
			new rviz::FloatProperty("Radius", 0.04,
									"Radius of a point",
									cmp_category_, SLOT(updateCoPColorAndAlpha()), this);

	// Instantaneous Capture Point properties
	inst_cp_color_property_ =
			new rviz::ColorProperty("Color", QColor(10, 41, 10),
									"Color of a point",
									inst_cp_category_, SLOT(updateCoPColorAndAlpha()), this);

	inst_cp_alpha_property_ =
			new rviz::FloatProperty("Alpha", 1.0,
									"0 is fully transparent, 1.0 is fully opaque.",
									inst_cp_category_, SLOT(updateCoPColorAndAlpha()), this);
	inst_cp_alpha_property_->setMin(0);
	inst_cp_alpha_property_->setMax(1);

	inst_cp_radius_property_ =
			new rviz::FloatProperty("Radius", 0.04,
									"Radius of a point",
									inst_cp_category_, SLOT(updateCoPColorAndAlpha()), this);


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
			new FloatProperty("Shaft Length", 0.8,
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

	// Support region properties
	support_line_color_property_ =
			new ColorProperty("Line Color", QColor(85, 0, 255),
							  "Color to draw the line.",
							  support_category_, SLOT(updateSupportLineColorAndAlpha()), this);

	support_line_alpha_property_ =
			new FloatProperty("Line Alpha", 1.0,
							  "Amount of transparency to apply to the line.",
							  support_category_, SLOT(updateSupportLineColorAndAlpha()), this);
	support_line_alpha_property_->setMin(0);
	support_line_alpha_property_->setMax(1);

	support_line_radius_property_ =
			new FloatProperty("Line Radius", 0.005,
							  "Radius of the line in m.",
							  support_category_, SLOT(updateSupportLineColorAndAlpha()), this);

	support_mesh_color_property_ =
			new ColorProperty("Mesh Color", QColor(85, 0, 255),
							  "Color to draw the mesh.",
							  support_category_, SLOT(updateSupportMeshColorAndAlpha()), this);

	support_mesh_alpha_property_ =
			new FloatProperty("Mesh Alpha", 0.2,
							  "Amount of transparency to apply to the mesh.",
							  support_category_, SLOT(updateSupportMeshColorAndAlpha()), this);
	support_mesh_alpha_property_->setMin(0);
	support_mesh_alpha_property_->setMax(1);

	support_force_threshold_property_ =
			new FloatProperty("Force Threshold", 1.0,
							  "Threshold for defining active contacts.",
							  support_category_, SLOT(updateSupportLineColorAndAlpha()), this);
}


WholeBodyStateDisplay::~WholeBodyStateDisplay()
{

}


void WholeBodyStateDisplay::clear()
{
	clearStatuses();
	robot_model_.clear();
	initialized_model_ = false;
}


void WholeBodyStateDisplay::onInitialize()
{
	MFDClass::onInitialize();
	updateGRFColorAndAlpha();
}


void WholeBodyStateDisplay::onEnable()
{
	MFDClass::onEnable();
	load();
}


void WholeBodyStateDisplay::onDisable()
{
	MFDClass::onDisable();
	clear();
}


void WholeBodyStateDisplay::fixedFrameChanged()
{
	if (is_info_)
		processWholeBodyState();
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
	wdyn_.modelFromURDFModel(robot_model_);
	fbs_ = wdyn_.getFloatingBaseSystem();
	weight_ = fbs_.getTotalMass() * fabs(fbs_.getGravityAcceleration());
	initialized_model_ = true;

	setStatus(StatusProperty::Ok, "URDF", "URDF parsed OK");
}


void WholeBodyStateDisplay::updateRobotModel()
{
	if (isEnabled()) {
		load();
		context_->queueRender();
	}
}


void WholeBodyStateDisplay::updateCoMStyle()
{
	CoMStyle style = (CoMStyle) com_style_property_->getOptionInt();

	switch (style)
	{
	case REAL:
	default:
		com_real_ = true;
		break;

	case PROJECTED:
		com_real_ = false;
		break;
	}
}


void WholeBodyStateDisplay::updateCoMColorAndAlpha()
{
	float radius = com_radius_property_->getFloat();
	Ogre::ColourValue color = com_color_property_->getOgreColor();
	color.a = com_alpha_property_->getFloat();

	com_visual_->setColor(color.r, color.g, color.b, color.a);
	comd_visual_->setColor(color.r, color.g, color.b, color.a);
	com_visual_->setRadius(radius);

	context_->queueRender();
}


void WholeBodyStateDisplay::updateCoMArrowGeometry()
{
	float shaft_length = com_shaft_length_property_->getFloat();
	float shaft_radius = com_shaft_radius_property_->getFloat();
	float head_length = com_head_length_property_->getFloat();
	float head_radius = com_head_radius_property_->getFloat();

	comd_visual_->setProperties(shaft_length, shaft_radius,
								head_length, head_radius);

	context_->queueRender();
}


void WholeBodyStateDisplay::updateCoPColorAndAlpha()
{
	float radius = cop_radius_property_->getFloat();
	Ogre::ColourValue color = cop_color_property_->getOgreColor();
	color.a = cop_alpha_property_->getFloat();

	cop_visual_->setColor(color.r, color.g, color.b, color.a);
	cop_visual_->setRadius(radius);

	context_->queueRender();
}

void WholeBodyStateDisplay::updateCMPColorAndAlpha()
{
	float radius = cmp_radius_property_->getFloat();
	Ogre::ColourValue color = cmp_color_property_->getOgreColor();
	color.a = cmp_alpha_property_->getFloat();

	cmp_visual_->setColor(color.r, color.g, color.b, color.a);
	cmp_visual_->setRadius(radius);

	context_->queueRender();
}

void WholeBodyStateDisplay::updateInstCPColorAndAlpha()
{
	float radius = inst_cp_radius_property_->getFloat();
	Ogre::ColourValue color = inst_cp_color_property_->getOgreColor();
	color.a = inst_cp_alpha_property_->getFloat();

	inst_cp_visual_->setColor(color.r, color.g, color.b, color.a);
	inst_cp_visual_->setRadius(radius);

	context_->queueRender();
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

	for (size_t i = 0; i < grf_visual_.size(); i++)
		grf_visual_[i]->setProperties(shaft_length, shaft_radius,
									  head_length, head_radius);

	context_->queueRender();
}


void WholeBodyStateDisplay::updateSupportLineColorAndAlpha()
{
	Ogre::ColourValue color = support_line_color_property_->getOgreColor();
	color.a = support_line_alpha_property_->getFloat();
	force_threshold_ = support_force_threshold_property_->getFloat();

	float radius = support_line_radius_property_->getFloat();
	if (is_info_) {
		support_visual_->setLineColor(color.r, color.g, color.b, color.a);
		support_visual_->setLineRadius(radius);
	}

	context_->queueRender();
}


void WholeBodyStateDisplay::updateSupportMeshColorAndAlpha()
{
	Ogre::ColourValue color = support_mesh_color_property_->getOgreColor();
	color.a = support_mesh_alpha_property_->getFloat();

	support_visual_->setMeshColor(color.r, color.g, color.b, color.a);

	context_->queueRender();
}


void WholeBodyStateDisplay::processMessage(const dwl_msgs::WholeBodyState::ConstPtr& msg)
{
	msg_ = msg;
	is_info_ = true;

	processWholeBodyState();
}


void WholeBodyStateDisplay::processWholeBodyState()
{
	// Checking if the urdf model was initialized
	if (!initialized_model_)
		return;

	// Getting the base velocity
	unsigned int num_base_joints = msg_->base.size();
	dwl::rbd::Vector6d base_pos = dwl::rbd::Vector6d::Zero();
	dwl::rbd::Vector6d base_vel = dwl::rbd::Vector6d::Zero();
	for (unsigned int i = 0; i < num_base_joints; i++) {
		dwl_msgs::BaseState base = msg_->base[i];

		// Getting the base joint id
		unsigned int id = base.id;

		// Setting the base position
		base_pos(id) = base.position;

		// Setting the base velocity
		base_vel(id) = base.velocity;
	}

	// Getting the joint position and velocity
	unsigned int num_joints = msg_->joints.size();
	Eigen::VectorXd joint_pos = Eigen::VectorXd::Zero(num_joints);
	Eigen::VectorXd joint_vel = Eigen::VectorXd::Zero(num_joints);
	for (unsigned int i = 0; i < num_joints; i++) {
		dwl_msgs::JointState joint = msg_->joints[i];

		// Getting the joint name
		std::string name  = joint.name;

		// Getting the joint id
		unsigned int id = fbs_.getJointId(name);

		// Setting the joint position and velocity
		joint_pos(id) = joint.position;
		joint_vel(id) = joint.velocity;
	}

	// Getting the contact wrenches and positions
	Eigen::Vector3d total_force = Eigen::Vector3d::Zero();
	dwl::rbd::BodySelector contact_names;
	dwl::rbd::BodyVectorXd contact_pos;
	dwl::rbd::BodyVector6d contact_for;
	std::vector<Ogre::Vector3> support;
	unsigned int num_contacts = msg_->contacts.size();
	for (unsigned int i = 0; i < num_contacts; i++) {
		dwl_msgs::ContactState contact = msg_->contacts[i];

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

		// Detecting active contacts
		if (wrench.norm() > force_threshold_) {
			support.push_back(Ogre::Vector3(position(dwl::rbd::X),
											position(dwl::rbd::Y),
											position(dwl::rbd::Z)));
		}
	}

	// Computing the center of mass position and velocity
	dwl::rbd::Vector6d null_base_pos = dwl::rbd::Vector6d::Zero();
	Eigen::Vector3d com_pos = fbs_.getSystemCoM(null_base_pos, joint_pos);
	Eigen::Vector3d com_vel = fbs_.getSystemCoMRate(null_base_pos, joint_pos,
													base_vel, joint_vel);

	// Computing the center of pressure position
	Eigen::Vector3d cop_pos;
	wdyn_.computeCenterOfPressure(cop_pos, contact_for, contact_pos, contact_names);

	// Computing the centroidal moment pivot position
	Eigen::Vector3d cmp_pos;
	wdyn_.computeCentroidalMomentPivot(cmp_pos, com_pos, cop_pos, contact_for, contact_pos, contact_names);

	// Computing the instantaneous capture point position
	Eigen::Vector3d icp_pos;
	wdyn_.computeInstantaneousCapturePoint(icp_pos, com_pos, com_vel, cop_pos);


	// Here we call the rviz::FrameManager to get the transform from the
	// fixed frame to the frame in the header of this Point message.  If
	// it fails, we can't do anything else so we return.
	Ogre::Quaternion orientation;
	Ogre::Vector3 position;
	if (!context_->getFrameManager()->getTransform(msg_->header.frame_id,
												   msg_->header.stamp,
												   position, orientation)) {
		ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
				  msg_->header.frame_id.c_str(), qPrintable(fixed_frame_));
		return;
	}

	// Resetting the point visualizers
	com_visual_.reset(new PointVisual(context_->getSceneManager(), scene_node_));
	comd_visual_.reset(new ArrowVisual(context_->getSceneManager(), scene_node_));
	cop_visual_.reset(new PointVisual(context_->getSceneManager(), scene_node_));
	cmp_visual_.reset(new PointVisual(context_->getSceneManager(), scene_node_));
	inst_cp_visual_.reset(new PointVisual(context_->getSceneManager(), scene_node_));
	support_visual_.reset(new PolygonVisual(context_->getSceneManager(), scene_node_));

	// Defining the center of mass as Ogre::Vector3
	Ogre::Vector3 com_point;
	if (com_real_) {
		com_point.x = com_pos(dwl::rbd::X);
		com_point.y = com_pos(dwl::rbd::Y);
		com_point.z = com_pos(dwl::rbd::Z);
	} else {
		Eigen::Vector3d cop_z = Eigen::Vector3d::Zero();
		cop_z(dwl::rbd::Z) = cop_pos(dwl::rbd::Z);
		Eigen::Vector3d rot_cop_z =
				dwl::math::getRotationMatrix(dwl::rbd::angularPart(base_pos)).transpose() * cop_z;
		com_point.x = com_pos(dwl::rbd::X) + rot_cop_z(dwl::rbd::X);
		com_point.y = com_pos(dwl::rbd::Y) + rot_cop_z(dwl::rbd::Y);
		com_point.z = cop_pos(dwl::rbd::Z);
	}

	// Defining the center of mass velocity orientation
	Eigen::Vector3d com_ref_dir = -Eigen::Vector3d::UnitZ();
	Eigen::Quaterniond com_q;
	com_q.setFromTwoVectors(com_ref_dir, com_vel);
	Ogre::Quaternion comd_for_orientation(com_q.w(), com_q.x(),
										  com_q.y(), com_q.z());

	// Now set or update the contents of the chosen CoM visual
	updateCoMColorAndAlpha();
	com_visual_->setPoint(com_point);
	com_visual_->setFramePosition(position);
	com_visual_->setFrameOrientation(orientation);
	float shaft_length = com_shaft_length_property_->getFloat() * com_vel.norm();
	float shaft_radius = com_shaft_radius_property_->getFloat();
	float head_length = com_head_length_property_->getFloat();
	float head_radius = com_head_radius_property_->getFloat();
	comd_visual_->setProperties(shaft_length, shaft_radius,
								head_length, head_radius);
	comd_visual_->setArrow(com_point, comd_for_orientation);
	comd_visual_->setFramePosition(position);
	comd_visual_->setFrameOrientation(orientation);

	// Defining the center of pressure as Ogre::Vector3
	Ogre::Vector3 cop_point;
	cop_point.x = cop_pos(dwl::rbd::X);
	cop_point.y = cop_pos(dwl::rbd::Y);
	cop_point.z = cop_pos(dwl::rbd::Z);

	// Defining the Centroidal Moment Pivot as Ogre::Vector3
	Ogre::Vector3 cmp_point;
	cmp_point.x = cmp_pos(dwl::rbd::X);
	cmp_point.y = cmp_pos(dwl::rbd::Y);
	cmp_point.z = cmp_pos(dwl::rbd::Z);

	// Defining the Instantaneous Capture Point as Ogre::Vector3
	Ogre::Vector3 inst_cp_point;
	inst_cp_point.x = icp_pos(dwl::rbd::X);
	inst_cp_point.y = icp_pos(dwl::rbd::Y);
	inst_cp_point.z = icp_pos(dwl::rbd::Z);

	// Now set or update the contents of the chosen CoP visual
	updateCoPColorAndAlpha();
	cop_visual_->setPoint(cop_point);
	cop_visual_->setFramePosition(position);
	cop_visual_->setFrameOrientation(orientation);

	// Now set or update the contents of the chosen CMP visual
	updateCMPColorAndAlpha();
	cmp_visual_->setPoint(cmp_point);
	cmp_visual_->setFramePosition(position);
	cmp_visual_->setFrameOrientation(orientation);

	// Now set or update the contents of the chosen Inst CP visual
	updateInstCPColorAndAlpha();
	inst_cp_visual_->setPoint(inst_cp_point);
	inst_cp_visual_->setFramePosition(position);
	inst_cp_visual_->setFrameOrientation(orientation);

	// Now set or update the contents of the chosen GRF visual
	grf_visual_.clear();
	for (unsigned int i = 0; i < num_contacts; i++) {
		dwl_msgs::ContactState contact = msg_->contacts[i];

		// Getting the name
		std::string name = contact.name;

		// Getting the contact position
		Ogre::Vector3 contact_pos(contact.position.x,
								  contact.position.y,
								  contact.position.z);

		// Getting the force direction
		Eigen::Vector3d for_ref_dir = -Eigen::Vector3d::UnitZ();
		Eigen::Vector3d for_dir(contact.wrench.force.x,
								contact.wrench.force.y,
								contact.wrench.force.z);

		// Detecting active contacts
		if (for_dir.norm() > force_threshold_) {
			Eigen::Quaterniond for_q;
			for_q.setFromTwoVectors(for_ref_dir, for_dir);
			Ogre::Quaternion contact_for_orientation(for_q.w(), for_q.x(),
													 for_q.y(), for_q.z());

			// We are keeping a vector of visual pointers. This creates the next
			// one and stores it in the vector
			boost::shared_ptr<ArrowVisual> arrow;
			arrow.reset(new ArrowVisual(context_->getSceneManager(), scene_node_));
			arrow->setArrow(contact_pos, contact_for_orientation);
			arrow->setFramePosition(position);
			arrow->setFrameOrientation(orientation);

			// Setting the arrow color and properties
			Ogre::ColourValue color = grf_color_property_->getOgreColor();
			color.a = grf_alpha_property_->getFloat();
			arrow->setColor(color.r, color.g, color.b, color.a);
			float shaft_length = grf_shaft_length_property_->getFloat() *
					for_dir.norm() / weight_;
			float shaft_radius = grf_shaft_radius_property_->getFloat();
			float head_length = grf_head_length_property_->getFloat();
			float head_radius = grf_head_radius_property_->getFloat();
			arrow->setProperties(shaft_length, shaft_radius,
								 head_length, head_radius);

			// And send it to the end of the vector
			grf_visual_.push_back(arrow);
		}
	}

	// Now set or update the contents of the chosen CoP visual
	support_visual_->setVertexs(support);
	updateSupportLineColorAndAlpha();
	updateSupportMeshColorAndAlpha();
	support_visual_->setFramePosition(position);
	support_visual_->setFrameOrientation(orientation);
}

} //@namespace dwl_rviz_plugin


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(dwl_rviz_plugin::WholeBodyStateDisplay, rviz::Display)
