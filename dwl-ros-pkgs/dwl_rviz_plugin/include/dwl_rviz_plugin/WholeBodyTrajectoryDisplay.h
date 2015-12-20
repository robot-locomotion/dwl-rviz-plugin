#ifndef DWL_RVIZ_PLUGIN__WHOLE_BODY_TRAJECTORY_DISPLAY__H
#define DWL_RVIZ_PLUGIN__WHOLE_BODY_TRAJECTORY_DISPLAY__H

#include <rviz/message_filter_display.h>
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

} //@namespace rviz


namespace dwl_rviz_plugin
{

/**
 * @class WholeBodyTrajectoryDisplay
 * @brief Displays a dwl_msgs::WholeBodyTrajectory message
 */
class WholeBodyTrajectoryDisplay: public rviz::MessageFilterDisplay<dwl_msgs::WholeBodyTrajectory>
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

		/** @brief Destroy the object for visualization */
		void destroyObjects();


	private Q_SLOTS:
		/** @brief Helper function to apply color and alpha to all visuals.
		/* Set the current color and alpha values for each visual */
		void updateBufferLength();
		void updateStyle();
		void updateLineWidth();
		void updateOffset();


	private:
		/** @brief Properties to show on side panel */
		rviz::Property* base_category_;
		rviz::Property* contact_category_;

		/** @brief Object for visualization of the data */
		std::vector<Ogre::ManualObject*> manual_objects_;
		std::vector<rviz::BillboardLine*> billboard_lines_;

		/** @brief Property objects for user-editable properties */
		rviz::EnumProperty* style_property_;
		rviz::ColorProperty* color_property_;
		rviz::FloatProperty* alpha_property_;
		rviz::FloatProperty* line_width_property_;
		rviz::IntProperty* buffer_length_property_;
		rviz::VectorProperty* offset_property_;

		enum LineStyle {LINES, BILLBOARDS};
};

} //@namespace dwl_rviz_plugin

#endif
