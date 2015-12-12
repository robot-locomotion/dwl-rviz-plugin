#ifndef DWL_RVIZ_PLUGIN__WHOLE_BODY_STATE_DISPLAY__H
#define DWL_RVIZ_PLUGIN__WHOLE_BODY_STATE_DISPLAY__H

#ifndef Q_MOC_RUN
#include <boost/circular_buffer.hpp>
#endif

#include <rviz/message_filter_display.h>
#include <dwl_msgs/WholeBodyState.h>


namespace Ogre
{
class SceneNode;
}

namespace rviz
{

class ColorProperty;
class FloatProperty;
class IntProperty;
class PointStampedVisual;

} //@namespace rviz


namespace dwl_rviz_plugin
{


class WholeBodyStateDisplay: public rviz::MessageFilterDisplay<dwl_msgs::WholeBodyState>
{
	Q_OBJECT
	public:
		// Constructor.  pluginlib::ClassLoader creates instances by calling
		// the default constructor, so make sure you have one.
		WholeBodyStateDisplay();
		virtual ~WholeBodyStateDisplay();
		void clear();

	protected:
		// Overrides of public virtual functions from the Display class.
		virtual void onInitialize();

		// Clear the visuals by deleting their objects.
		virtual void reset();
		/** @brief Loads a URDF from the ros-param named by our
		 * "Robot Description" property, iterates through the links, and
		 * loads any necessary models. */
		virtual void load();

		std::string robot_model_;

	private Q_SLOTS:
		// Helper function to apply color and alpha to all visuals.
		// Set the current color and alpha values for each visual.
		void updateColorAndAlpha();
		// Set the number of past visuals to show.
		void updateHistoryLength();
		void updateRobotDescription();

	// Function to handle an incoming ROS message.
	private:
		// This is our callback to handle an incoming message.
		void processMessage(const dwl_msgs::WholeBodyState::ConstPtr& msg);

		// Storage for the list of visuals.  It is a circular buffer where
		// data gets popped from the front (oldest) and pushed to the back (newest)
		boost::circular_buffer<boost::shared_ptr<rviz::PointStampedVisual> > visuals_;

		// Property objects for user-editable properties.
		rviz::ColorProperty *color_property_;
        rviz::FloatProperty *alpha_property_, *radius_property_;
        rviz::IntProperty *history_length_property_;
        rviz::StringProperty* robot_model_property_;
};

} //@namespace dwl_rviz_plugin

#endif
