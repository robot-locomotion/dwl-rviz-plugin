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

ReducedTrajectoryDisplay::ReducedTrajectoryDisplay() : received_msg_(false),
		new_msg_(false), display_idx_(0), next_(false), mode_display_(REALTIME),
		rt_factor_(1.)
{
	// Mode display properties
	mode_display_property_ =
			new rviz::EnumProperty("Mode Display", "Realtime",
								   "Mode display of a received reduced trajectory",
								   this, SLOT(updateModeDisplay()), this);
	mode_display_property_->addOption("Realtime", REALTIME);
	mode_display_property_->addOption("Full", FULL);
	mode_display_property_->addOption("Loop", LOOP);

	// Real-time factor properties
	rt_factor_property_ =
			new rviz::FloatProperty("RT Factor", 1.0,
									"0.01 is 1% of speed, 1.0 is real-time speed.",
									this, SLOT(updateModeDisplay()), this);
	rt_factor_property_->setMin(0.01);
	rt_factor_property_->setMax(1);

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

	support_line_radius_property_ =
			new FloatProperty("Line Radius", 0.005,
							  "Radius of the line in m.",
							  support_category_, SLOT(updateSupportAlpha()), this);

	support_mesh_alpha_property_ =
			new rviz::FloatProperty("Mesh Alpha", 0.2,
									"0 is fully transparent, 1.0 is fully opaque.",
									support_category_, SLOT(updateSupportAlpha()), this);
	support_mesh_alpha_property_->setMin(0);
	support_mesh_alpha_property_->setMax(1);


	// Pendulum properties
	pendulum_alpha_property_ =
			new rviz::FloatProperty("Alpha", 0.2,
									"0 is fully transparent, 1.0 is fully opaque.",
									pendulum_category_, SLOT(updatePendulumArrowGeometry()), this);
	pendulum_alpha_property_->setMin(0);
	pendulum_alpha_property_->setMax(1);

	pendulum_line_radius_property_ =
			new FloatProperty("Line Radius", 0.02,
							  "Radius of the line, in meters.",
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


void ReducedTrajectoryDisplay::fixedFrameChanged()
{
	if (received_msg_) {
		// Setting up the message
		new_msg_ = true;

		// Resetting the values for the new message display
		msg_time_ = msg_->actual.time;
		display_idx_ = -1;

		// Destroying the old displays
		destroyObjects();

		// Compute the set of colors
		generateSetOfColors(colours_, msg_->trajectory.size());
	}
}


void ReducedTrajectoryDisplay::reset()
{
	MFDClass::reset();
	destroyObjects();
}


void ReducedTrajectoryDisplay::updateModeDisplay()
{
	mode_display_ = (ModeDisplay) mode_display_property_->getOptionInt();

	if (mode_display_ == LOOP) {
		rt_factor_property_->show();
		rt_factor_ = rt_factor_property_->getFloat();
	} else
		rt_factor_property_->hide();

	// Updating the display if there is old information
	if (received_msg_) {
		// Resetting the values for the new message display
		msg_time_ = msg_->actual.time;
		display_idx_ = -1;
		next_ = true;
		new_msg_ = true;

		// Destroying the old displays
		destroyObjects();

		// Updating the display
		updateDisplay();
	}
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

	float radius = support_line_radius_property_->getFloat();
	for (unsigned int i = 0; i < support_visual_.size(); i++) {
		support_visual_[i]->setLineRadius(radius);
	}

	context_->queueRender();
}


void ReducedTrajectoryDisplay::updatePendulumArrowGeometry()
{
	pendulum_alpha_ = pendulum_alpha_property_->getFloat();

	context_->queueRender();
}


void ReducedTrajectoryDisplay::processMessage(const dwl_msgs::ReducedBodyTrajectory::ConstPtr& msg)
{
	// Setting up the message
	msg_ = msg;
	received_msg_ = true;
	new_msg_ = true;

	// Resetting the values for the new message display
	msg_time_ = msg_->actual.time;
	display_idx_ = -1;

	// Destroying the old displays
	destroyObjects();

	// Compute the set of colors
	generateSetOfColors(colours_, msg_->trajectory.size() + 1);
}


void ReducedTrajectoryDisplay::update(float wall_dt, float ros_dt)
{
	// Display the state when a new message arrives
	if (new_msg_) {
		// Increment the message time
		msg_time_ += wall_dt;

		// Updating the display
		updateDisplay();
	}
}


void ReducedTrajectoryDisplay::destroyObjects()
{
	com_visual_.clear();
	cop_visual_.clear();
	for (std::vector<PolygonVisual*>::iterator it = support_visual_.begin();
			it != support_visual_.end(); ++it)
		delete (*it);
	support_visual_.clear();
	pendulum_visual_.clear();
}


void ReducedTrajectoryDisplay::updateDisplay()
{
	// Visualization of the reduced trajectory
	dwl_msgs::ReducedBodyState state;
	if (mode_display_ == FULL) {
		for (unsigned int k = 0; k < msg_->trajectory.size() + 1; k++) {
			// Setting up the actual display index
			display_idx_ = k - 1;

			// Getting the actual state to display
			if (k == 0)
				state = msg_->actual;
			else
				state = msg_->trajectory[display_idx_];

			// Display the state
			displayState(state);
		}

		// No new message to process it
		new_msg_ = false;
	} else { // realtime or loop
		// Getting the actual state to display
		if (display_idx_ == -1)
			state = msg_->actual;
		else
			state = msg_->trajectory[display_idx_];

		if (next_) {
			// Destroy all the visual information
			destroyObjects();

			displayState(state);
		}

		// Visualization according to the defined mode (realtime / loop)
		if (mode_display_ == REALTIME) {
			if (msg_time_ >= state.time) {
				next_ = true;
				display_idx_++;

				if (display_idx_ > (int) msg_->trajectory.size() - 1) {
					display_idx_ = msg_->trajectory.size() - 1;
					new_msg_ = false;
				}
			} else
				next_ = false;
		} else { // loop mode
			if (msg_time_ >= state.time / rt_factor_) {
				next_ = true;
				display_idx_++;

				if (display_idx_ > (int) msg_->trajectory.size() - 1) {
					msg_time_ = msg_->actual.time;
					display_idx_ = -1;
				}
			} else
				next_ = false;
		}
	}
}


void ReducedTrajectoryDisplay::displayState(dwl_msgs::ReducedBodyState& state)
{
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

	// Getting the center of mass position
	geometry_msgs::Vector3 com_vec = state.center_of_mass;
	Ogre::Vector3 com_pos(com_vec.x, com_vec.y, com_vec.z);

	// Getting the actual color
	Ogre::ColourValue colour = colours_[display_idx_ + 1];

	// We are keeping a vector of CoM visual pointers. This creates the next
	// one and stores it in the vector
	boost::shared_ptr<PointVisual> com_visual;
	com_visual.reset(new PointVisual(context_->getSceneManager(), scene_node_));
	updateCoMRadiusAndAlpha();
	colour.a = com_alpha_;
	com_visual->setColor(colour.r, colour.g, colour.b, colour.a);
	com_visual->setRadius(com_radius_);
	com_visual->setPoint(com_pos);
	com_visual->setFramePosition(position);
	com_visual->setFrameOrientation(orientation);

	// And send it to the end of the vector
	com_visual_.push_back(com_visual);


	// Getting the center of pressure position
	geometry_msgs::Vector3 cop_vec = state.center_of_pressure;
	Ogre::Vector3 cop_pos(cop_vec.x, cop_vec.y, cop_vec.z);

	// We are keeping a vector of CoP visual pointers. This creates the
	// next one and stores it in the vector
	boost::shared_ptr<PointVisual> cop_visual;
	cop_visual.reset(new PointVisual(context_->getSceneManager(), scene_node_));
	updateCoPRadiusAndAlpha();
	colour.a = cop_alpha_;
	cop_visual->setColor(colour.r, colour.g, colour.b, colour.a);
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

	// Now set or update the contents of the chosen support visual
	// We are keeping a vector of support regions visual pointers. This
	// creates the next one and stores it in the vector
	PolygonVisual* polygon_visual = new PolygonVisual(context_->getSceneManager(), scene_node_);;
	updateSupportAlpha();
	polygon_visual->setVertexs(support);
	polygon_visual->setLineColor(colour.r, colour.g, colour.b, support_line_alpha_);
	polygon_visual->setLineRadius(support_line_radius_property_->getFloat());
	polygon_visual->setMeshColor(colour.r, colour.g, colour.b, support_mesh_alpha_);
	polygon_visual->setFramePosition(position);
	polygon_visual->setFrameOrientation(orientation);

	// And send it to the end of the vector
	support_visual_.push_back(polygon_visual);


	// Getting the pendulum direction
	Eigen::Vector3d ref_dir = -Eigen::Vector3d::UnitZ();
	Eigen::Vector3d pendulum_dir(com_pos.x - cop_pos.x,
								 com_pos.y - cop_pos.y,
								 com_pos.z - cop_pos.z);

	Eigen::Quaterniond pendulum_q;
	pendulum_q.setFromTwoVectors(ref_dir, pendulum_dir);
	Ogre::Quaternion pendulum_orientation(pendulum_q.w(),
										  pendulum_q.x(),
										  pendulum_q.y(),
										  pendulum_q.z());

	// We are keeping a vector of visual pointers. This creates the next
	// one and stores it in the vector
	boost::shared_ptr<ArrowVisual> arrow;
	arrow.reset(new ArrowVisual(context_->getSceneManager(), scene_node_));
	arrow->setArrow(cop_pos, pendulum_orientation);
	arrow->setFramePosition(position);
	arrow->setFrameOrientation(orientation);

	// Setting the arrow color and properties
	updatePendulumArrowGeometry();
	arrow->setColor(colour.r, colour.g, colour.b, pendulum_alpha_);
	float line_length = pendulum_dir.norm();
	float line_radius = pendulum_line_radius_property_->getFloat();
	arrow->setProperties(line_length, line_radius, 0., 0.);

	// And send it to the end of the vector
	pendulum_visual_.push_back(arrow);
}


void ReducedTrajectoryDisplay::generateSetOfColors(std::vector<Ogre::ColourValue>& colors,
												   unsigned int num_points)
{
	colors.resize(num_points);
	for (unsigned int i = 0; i < num_points; i++) {
		float hue = (float) i / (float) num_points;
		float saturation = (float) (90 + rand() % 10) / 100;
		float brightness = (float) (50 + rand() % 10) / 100;

		colors[i].setHSB(hue, saturation, brightness);
	}
}

} //@namespace dwl_rviz_plugin


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(dwl_rviz_plugin::ReducedTrajectoryDisplay, rviz::Display)
