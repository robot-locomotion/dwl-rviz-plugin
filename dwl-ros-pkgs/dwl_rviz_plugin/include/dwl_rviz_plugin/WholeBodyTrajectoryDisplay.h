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
 * \brief Displays a dwl_msgs::WholeBodyTrajectory message
 */
class WholeBodyTrajectoryDisplay: public rviz::MessageFilterDisplay<dwl_msgs::WholeBodyTrajectory>
{
	Q_OBJECT
	public:
		WholeBodyTrajectoryDisplay();
		virtual ~WholeBodyTrajectoryDisplay();

		/** @brief Overridden from Display. */
		virtual void reset();

	protected:
		/** @brief Overridden from Display. */
		virtual void onInitialize();

		/** @brief Overridden from MessageFilterDisplay. */
		void processMessage(const dwl_msgs::WholeBodyTrajectory::ConstPtr& msg);

	private Q_SLOTS:
		void updateBufferLength();
		void updateStyle();
		void updateLineWidth();
		void updateOffset();

	private:
		void destroyObjects();

		std::vector<Ogre::ManualObject*> manual_objects_;
		std::vector<rviz::BillboardLine*> billboard_lines_;

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
