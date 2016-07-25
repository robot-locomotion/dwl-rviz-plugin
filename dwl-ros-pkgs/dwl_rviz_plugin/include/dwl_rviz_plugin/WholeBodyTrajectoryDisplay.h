#ifndef DWL_RVIZ_PLUGIN__WHOLE_BODY_TRAJECTORY_DISPLAY__H
#define DWL_RVIZ_PLUGIN__WHOLE_BODY_TRAJECTORY_DISPLAY__H

#include <rviz/message_filter_display.h>
#include <dwl_rviz_plugin/PointVisual.h>
#include <dwl/utils/RigidBodyDynamics.h>
#include <dwl_msgs/WholeBodyTrajectory.h>


namespace Ogre
{
class ManualObject;
}

namespace rviz
{

class ColorProperty;
class FloatProperty;
class IntProperty;
class EnumProperty;
class BillboardLine;
class VectorProperty;
class Axes;

} //@namespace rviz


namespace dwl_rviz_plugin
{

/**
 * @class WholeBodyTrajectoryDisplay
 * @brief Displays a dwl_msgs::WholeBodyTrajectory message
 */
class WholeBodyTrajectoryDisplay :
		public rviz::MessageFilterDisplay<dwl_msgs::WholeBodyTrajectory>
{
	Q_OBJECT
	public:
		/** @brief Constructor function */
		WholeBodyTrajectoryDisplay();

		/** @brief Destructor function */
		~WholeBodyTrajectoryDisplay();

		/** @brief Clear the visuals by deleting their objects */
		void reset();

		/** @brief Overridden from Display. */
		void onInitialize();

		/**
		 * @brief Function to handle an incoming ROS message
		 * This is our callback to handle an incoming message
		 * @param const dwl_msgs::WholeBodyTrajectory::ConstPtr& Whole-body trajectory msg
		 */
		void processMessage(const dwl_msgs::WholeBodyTrajectory::ConstPtr& msg);


	private Q_SLOTS:
		/** @brief Helper function to apply color and alpha to all visuals.
		/* Set the current color and alpha values for each visual */
		void updateBaseStyle();
		void updateBaseLineProperties();
		void updateContactStyle();
		void updateContactLineProperties();


	private:
		/** @brief Process the trajectories */
		void processBaseTrajectory();
		void processContactTrajectory();

		/** Destroy all the objects for visualization */
		void destroyObjects();

		/** @brief Whole-body trajectory message */
		dwl_msgs::WholeBodyTrajectory::ConstPtr msg_;

		/** @brief Indicates if it's been received a message */
		bool is_info_;

		/** @brief Properties to show on side panel */
		rviz::Property* base_category_;
		rviz::Property* contact_category_;

		/** @brief Object for visualization of the data */
		boost::shared_ptr<Ogre::ManualObject> base_manual_object_;
		boost::shared_ptr<rviz::BillboardLine> base_billboard_line_;
		std::vector<boost::shared_ptr<PointVisual> > base_points_;
		std::vector<boost::shared_ptr<rviz::Axes> > base_axes_;
		std::vector<boost::shared_ptr<Ogre::ManualObject> > contact_manual_object_;
		std::vector<boost::shared_ptr<rviz::BillboardLine> > contact_billboard_line_;
		std::vector<std::vector<boost::shared_ptr<PointVisual> > > contact_points_;

		/** @brief Property objects for user-editable properties */
		rviz::EnumProperty* base_style_property_;
		rviz::ColorProperty* base_color_property_;
		rviz::FloatProperty* base_alpha_property_;
		rviz::FloatProperty* base_line_width_property_;
		rviz::FloatProperty* base_scale_property_;

		rviz::EnumProperty* contact_style_property_;
		rviz::ColorProperty* contact_color_property_;
		rviz::FloatProperty* contact_alpha_property_;
		rviz::FloatProperty* contact_line_width_property_;

		Ogre::Vector3 last_point_position_;

		enum LineStyle {LINES, BILLBOARDS, POINTS};
};

} //@namespace dwl_rviz_plugin

#endif
