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

ReducedTrajectoryDisplay::ReducedTrajectoryDisplay() : received_msg_(false), idx_(0), next_(false)
{
	// Category Groups
	com_category_ = new rviz::Property("Center of Mass", QVariant(), "", this);
	cop_category_ = new rviz::Property("Center of Pressure", QVariant(), "", this);
	support_category_ = new rviz::Property("Support Region", QVariant(), "", this);
	pendulum_category_ = new rviz::Property("Pendulum", QVariant(), "", this);
	
	
	mode_display_property_ = 
			new rviz::EnumProperty("Mode Display", "Realtime",
								   "Mode display of a received reduced trajectory",
								   this, SLOT(updateModeDisplay()), this);
	mode_display_property_->addOption("Realtime", REALTIME);
	mode_display_property_->addOption("Full", FULL);
	

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

void ReducedTrajectoryDisplay::reset()
{
	MFDClass::reset();
	destroyObjects();
}


void ReducedTrajectoryDisplay::updateModeDisplay()
{
	mode_display_ = (ModeDisplay) mode_display_property_->getOptionInt();
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
	float line_radius = pendulum_line_radius_property_->getFloat();
	pendulum_alpha_ = pendulum_alpha_property_->getFloat();

	context_->queueRender();
}


void ReducedTrajectoryDisplay::processMessage(const dwl_msgs::ReducedTrajectory::ConstPtr& msg)
{
	msg_ = msg;
	received_msg_ = true;
	std::cout << "#############" << std::endl;
	msg_time_ = 0.;
	destroyObjects();
}


void ReducedTrajectoryDisplay::update(float wall_dt, float ros_dt)
{
	if (received_msg_) {
		msg_time_ += wall_dt;
		std::cout << "testing " << msg_time_ << std::endl;
	
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
	
	
	
		// Compute the set of colors
		std::vector<Ogre::ColourValue> colors;
		generateSetOfColors(colors, msg_->trajectory.size());

		
		dwl_msgs::ReducedState state = msg_->trajectory[idx_];

		// Visualization of the reduced trajectory
		if (next_) {
			destroyObjects();
			
			// Getting the center of mass position
			geometry_msgs::Vector3 com_vec = state.center_of_mass;
			Ogre::Vector3 com_pos(com_vec.x, com_vec.y, com_vec.z);

			// Getting the actual color
			Ogre::ColourValue color = colors[idx_];

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

			// We are keeping a vector of CoP visual pointers. This creates the
			// next one and stores it in the vector
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

			// Now set or update the contents of the chosen support visual
			// We are keeping a vector of support regions visual pointers. This
			// creates the next one and stores it in the vector
			PolygonVisual* polygon_visual = new PolygonVisual(context_->getSceneManager(), scene_node_);;
			updateSupportAlpha();
			polygon_visual->setVertexs(support);
			polygon_visual->setLineColor(color.r, color.g, color.b, support_line_alpha_);
			polygon_visual->setLineRadius(support_line_radius_property_->getFloat());
			polygon_visual->setMeshColor(color.r, color.g, color.b, support_mesh_alpha_);
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
			arrow->setColor(color.r, color.g, color.b, pendulum_alpha_);
			float line_length = pendulum_dir.norm();
			float line_radius = pendulum_line_radius_property_->getFloat();
			arrow->setProperties(line_length, line_radius, 0., 0.);

			// And send it to the end of the vector
			pendulum_visual_.push_back(arrow);
		}
				
//		if (mode_display_ == REALTIME) {
//			context_->queueRender();
//			context_->queueRender();
//			context_->queueRender();
//			context_->queueRender();
//			std::cout << k << " " << state.time << std::endl;
//			wait(1.);//(state.time);
//		}
	//}
		
		
		
		if (msg_time_ >= state.time) {
			next_ = true;
			idx_++;
			
			if (idx_ > msg_->trajectory.size()-1) {
				idx_ = msg_->trajectory.size()-1;
				received_msg_ = false;
			}
		} else {
			next_ = false;
		}
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
